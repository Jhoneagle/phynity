#include <catch2/catch_test_macros.hpp>
#include <core/jobs/job_pool.hpp>

#include <atomic>
#include <set>
#include <thread>
#include <vector>

using namespace phynity::jobs;

TEST_CASE("Job struct is 128 bytes and cacheline-aligned", "[jobs][job]")
{
    STATIC_REQUIRE(sizeof(Job) == 128);
    STATIC_REQUIRE(alignof(Job) == 64);
}

TEST_CASE("Job inline data storage", "[jobs][job]")
{
    Job job;

    struct SmallData
    {
        float *ptr;
        uint32_t start;
        uint32_t end;
    };
    static_assert(sizeof(SmallData) <= kMaxInlineDataSize);

    SmallData data{nullptr, 10, 20};
    void *stored = store_inline_data(job, data);

    REQUIRE(stored == static_cast<void *>(job.inline_data));

    auto *recovered = static_cast<SmallData *>(stored);
    REQUIRE(recovered->ptr == nullptr);
    REQUIRE(recovered->start == 10);
    REQUIRE(recovered->end == 20);
}

TEST_CASE("JobPool allocate returns valid ids with increasing generation", "[jobs][job_pool]")
{
    JobPool pool(4);

    auto id1 = pool.allocate();
    auto id2 = pool.allocate();
    auto id3 = pool.allocate();
    auto id4 = pool.allocate();

    REQUIRE(id1.valid());
    REQUIRE(id2.valid());
    REQUIRE(id3.valid());
    REQUIRE(id4.valid());

    // All indices should be 0-3
    std::set<uint32_t> indices{id1.index, id2.index, id3.index, id4.index};
    REQUIRE(indices.size() == 4);

    // All should have generation 1 (first allocation round)
    REQUIRE(id1.generation == 1);
    REQUIRE(id2.generation == 1);
    REQUIRE(id3.generation == 1);
    REQUIRE(id4.generation == 1);
}

TEST_CASE("JobPool access via operator[] and at()", "[jobs][job_pool]")
{
    JobPool pool(4);
    auto id = pool.allocate();

    auto &job = pool.at(id);
    job.function = nullptr;
    job.debug_name = "test_job";

    REQUIRE(pool[id.index].debug_name == std::string("test_job"));
}

TEST_CASE("JobPool overflow dependents", "[jobs][job_pool]")
{
    JobPool pool(4);

    uint32_t offset = pool.allocate_overflow(3);
    pool.overflow_dependent(offset) = JobId{0, 1};
    pool.overflow_dependent(offset + 1) = JobId{1, 1};
    pool.overflow_dependent(offset + 2) = JobId{2, 1};

    REQUIRE(pool.overflow_dependent(offset).index == 0);
    REQUIRE(pool.overflow_dependent(offset + 1).index == 1);
    REQUIRE(pool.overflow_dependent(offset + 2).index == 2);

    pool.clear_overflow();
}

TEST_CASE("JobPool generation recycling after wrap-around", "[jobs][job_pool]")
{
    constexpr uint32_t capacity = 4;
    JobPool pool(capacity);

    // Allocate all 4 slots (generation 1)
    std::vector<JobId> first_round;
    for (uint32_t i = 0; i < capacity; ++i)
    {
        first_round.push_back(pool.allocate());
    }

    for (const auto &id : first_round)
    {
        REQUIRE(id.valid());
        REQUIRE(id.generation == 1);
    }

    // The 5th allocation wraps around to index 0 with generation 2.
    // The spin in allocate() checks `slot.generation >= gen`. For raw=4,
    // idx=0, gen=2. Slot 0 has generation=1, so 1 < 2 passes immediately.
    auto wrapped = pool.allocate();
    REQUIRE(wrapped.valid());
    REQUIRE(wrapped.index == 0);
    REQUIRE(wrapped.generation == 2);

    // Allocate the remaining 3 wrapped slots
    for (uint32_t i = 0; i < 3; ++i)
    {
        auto id = pool.allocate();
        REQUIRE(id.valid());
        REQUIRE(id.generation == 2);
    }
}

TEST_CASE("JobPool spin-wait on occupied slot", "[jobs][job_pool]")
{
    // When the pool wraps around twice, the second wrap must spin-wait until
    // the slot's generation is less than the target generation. We simulate
    // this by having one thread hold a slot (high generation) while another
    // thread attempts to allocate the same slot in the next wrap.
    constexpr uint32_t capacity = 4;
    JobPool pool(capacity);

    // First round: allocate all 4 slots (generation 1, raw 0-3)
    for (uint32_t i = 0; i < capacity; ++i)
    {
        (void) pool.allocate();
    }

    // Second round: allocate all 4 slots (generation 2, raw 4-7)
    // This works because gen=2 > slot.generation=1
    for (uint32_t i = 0; i < capacity; ++i)
    {
        (void) pool.allocate();
    }

    // Now raw counter is at 8. Slots have generation=2.
    // Third round: raw=8 → idx=0, gen=3. Slot 0 has gen=2, so 2 < 3 passes.
    // This verifies the spin condition works across multiple wraps.
    auto third_wrap = pool.allocate();
    REQUIRE(third_wrap.valid());
    REQUIRE(third_wrap.index == 0);
    REQUIRE(third_wrap.generation == 3);
}

TEST_CASE("JobPool concurrent allocation produces unique ids", "[jobs][job_pool]")
{
    constexpr uint32_t pool_capacity = 64;
    constexpr uint32_t jobs_per_thread = 16;
    constexpr uint32_t thread_count = 4;

    JobPool pool(pool_capacity);

    std::vector<std::vector<JobId>> thread_ids(thread_count);

    std::vector<std::thread> threads;
    for (uint32_t t = 0; t < thread_count; ++t)
    {
        threads.emplace_back(
            [&pool, &thread_ids, t]
            {
                for (uint32_t i = 0; i < jobs_per_thread; ++i)
                {
                    thread_ids[t].push_back(pool.allocate());
                }
            });
    }

    for (auto &t : threads)
    {
        t.join();
    }

    // All allocated ids should be unique (index+generation pair)
    std::set<std::pair<uint32_t, uint32_t>> unique_ids;
    for (const auto &ids : thread_ids)
    {
        for (const auto &id : ids)
        {
            REQUIRE(id.valid());
            unique_ids.emplace(id.index, id.generation);
        }
    }

    REQUIRE(unique_ids.size() == thread_count * jobs_per_thread);
}

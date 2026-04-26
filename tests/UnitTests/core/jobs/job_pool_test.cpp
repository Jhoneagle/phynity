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

TEST_CASE("JobPool concurrent allocation produces unique ids", "[jobs][job_pool]")
{
    constexpr uint32_t pool_capacity = 64;
    constexpr uint32_t jobs_per_thread = 16;
    constexpr int thread_count = 4;

    JobPool pool(pool_capacity);

    std::vector<std::vector<JobId>> thread_ids(thread_count);

    std::vector<std::thread> threads;
    for (int t = 0; t < thread_count; ++t)
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

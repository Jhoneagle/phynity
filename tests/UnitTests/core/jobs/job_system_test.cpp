#include <catch2/catch_test_macros.hpp>
#include <core/jobs/job_system.hpp>

#include <atomic>
#include <thread>
#include <vector>

using namespace phynity::jobs;

TEST_CASE("JobSystem basic submission and wait", "[jobs]")
{
    JobSystemConfig config{.worker_count = 1, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    REQUIRE(js.is_running());
    REQUIRE(js.worker_count() == 1);

    std::atomic<int> counter{0};
    auto handle = js.submit([&counter] { counter++; });

    REQUIRE(handle.valid());
    js.wait(handle);

    REQUIRE(counter == 1);

    js.shutdown();
    REQUIRE(!js.is_running());
}

TEST_CASE("JobSystem multiple jobs", "[jobs]")
{
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> counter{0};
    std::vector<JobHandle> handles;
    handles.reserve(10);

    for (int i = 0; i < 10; ++i)
    {
        handles.push_back(js.submit([&counter] { counter++; }));
    }

    js.wait_all(handles);
    REQUIRE(counter == 10);

    js.shutdown();
}

TEST_CASE("JobSystem deterministic mode", "[jobs]")
{
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Deterministic};
    JobSystem js(config);

    REQUIRE(js.scheduling_mode() == SchedulingMode::Deterministic);

    std::atomic<int> counter{0};
    std::vector<JobHandle> handles;
    handles.reserve(5);

    for (int i = 0; i < 5; ++i)
    {
        handles.push_back(js.submit([&counter] { counter++; }));
    }

    js.wait_all(handles);
    REQUIRE(counter == 5);

    js.shutdown();
}

TEST_CASE("JobSystem parallel_for serial fallback", "[jobs]")
{
    // Default-constructed JobSystem is not started, so parallel_for falls back to serial
    JobSystem js;
    std::vector<int> results(10, 0);

    js.parallel_for(0, 10, 1, [&results](uint32_t i) { results[static_cast<size_t>(i)] = static_cast<int>(i) * 2; });

    for (size_t i = 0; i < 10; ++i)
    {
        REQUIRE(results[i] == static_cast<int>(i) * 2);
    }

    js.shutdown();
}

TEST_CASE("JobSystem parallel_for with workers visits all indices", "[jobs]")
{
    // Start a real job system with 4 workers to test parallel execution
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);
    REQUIRE(js.is_running());

    constexpr uint32_t count = 1000;
    std::atomic<uint32_t> visit_count{0};
    std::vector<std::atomic<int>> visited(count);
    for (auto &v : visited)
    {
        v.store(0);
    }

    js.parallel_for(0,
                    count,
                    100,
                    [&](uint32_t i)
                    {
                        visited[i].fetch_add(1);
                        visit_count.fetch_add(1);
                    });

    // Every index must be visited exactly once
    REQUIRE(visit_count.load() == count);
    for (uint32_t i = 0; i < count; ++i)
    {
        REQUIRE(visited[i].load() == 1);
    }

    js.shutdown();
}

TEST_CASE("JobSystem parallel_for deterministic mode runs serially", "[jobs]")
{
    // Deterministic mode should fall back to serial execution for reproducibility
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Deterministic};
    JobSystem js(config);
    REQUIRE(js.is_running());

    // In serial execution, indices are visited in order
    std::vector<uint32_t> order;
    order.reserve(100);

    js.parallel_for(0, 100, 10, [&order](uint32_t i) { order.push_back(i); });

    REQUIRE(order.size() == 100);
    for (uint32_t i = 0; i < 100; ++i)
    {
        REQUIRE(order[i] == i);
    }

    js.shutdown();
}

TEST_CASE("JobSystem parallel_for small range uses serial", "[jobs]")
{
    // When range is <= grain, should run serially even with workers
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::vector<uint32_t> order;
    order.reserve(5);

    // grain=10 but range is only 5, so serial path
    js.parallel_for(0, 5, 10, [&order](uint32_t i) { order.push_back(i); });

    REQUIRE(order.size() == 5);
    for (uint32_t i = 0; i < 5; ++i)
    {
        REQUIRE(order[i] == i);
    }

    js.shutdown();
}

TEST_CASE("JobSystem concurrent submit/complete stress", "[jobs]")
{
    // Stress test: submit many jobs from multiple threads to exercise slot reuse.
    // With 4 workers and 8000 jobs (2x the 4096 pool), every slot is reused at least once.
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);
    REQUIRE(js.is_running());

    constexpr uint32_t total_jobs = 8000;
    std::atomic<uint32_t> counter{0};

    // Submit from 4 threads concurrently
    constexpr uint32_t num_submitters = 4;
    constexpr uint32_t jobs_per_submitter = total_jobs / num_submitters;
    std::vector<std::thread> submitters;
    submitters.reserve(num_submitters);

    for (uint32_t t = 0; t < num_submitters; ++t)
    {
        submitters.emplace_back(
            [&js, &counter]
            {
                std::vector<JobHandle> handles;
                handles.reserve(jobs_per_submitter);

                for (uint32_t i = 0; i < jobs_per_submitter; ++i)
                {
                    handles.push_back(js.submit([&counter] { counter.fetch_add(1, std::memory_order_relaxed); }));
                }

                js.wait_all(handles);
            });
    }

    for (auto &t : submitters)
    {
        t.join();
    }

    REQUIRE(counter.load() == total_jobs);
    js.shutdown();
}

TEST_CASE("JobSystem wait on completed job returns immediately", "[jobs]")
{
    // Waiting on a job that has already completed should return without blocking
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> value{0};
    auto handle = js.submit([&value] { value.store(42, std::memory_order_relaxed); });
    js.wait(handle);
    REQUIRE(value.load() == 42);

    // Second wait on the same handle should return immediately (slot is Completed or Free)
    js.wait(handle);
    REQUIRE(value.load() == 42);

    js.shutdown();
}

TEST_CASE("JobSystem shutdown unblocks waiting threads", "[jobs]")
{
    // Verify that shutdown() wakes threads blocked in wait() so they don't hang.
    JobSystemConfig config{.worker_count = 1, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<bool> blocker_started{false};
    std::atomic<bool> blocker_done{false};

    // Submit a job that blocks indefinitely until we tell it to stop
    std::atomic<bool> release_blocker{false};
    js.submit(
        [&blocker_started, &release_blocker]
        {
            blocker_started.store(true, std::memory_order_release);
            while (!release_blocker.load(std::memory_order_acquire))
            {
                std::this_thread::yield();
            }
        });

    // Wait for blocker to start executing
    while (!blocker_started.load(std::memory_order_acquire))
    {
        std::this_thread::yield();
    }

    // Submit a second job that will be queued behind the blocker
    auto queued_handle = js.submit([] {});

    // Start a thread that waits on the queued job
    std::thread waiter(
        [&js, queued_handle, &blocker_done]
        {
            js.wait(queued_handle);
            blocker_done.store(true, std::memory_order_release);
        });

    // Release the blocker so shutdown can proceed, then shutdown
    release_blocker.store(true, std::memory_order_release);
    js.shutdown();

    // The waiter thread must have been unblocked by shutdown
    waiter.join();
    REQUIRE(blocker_done.load());
}

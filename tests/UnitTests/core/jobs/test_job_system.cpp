#include <catch2/catch_test_macros.hpp>
#include <core/jobs/job_system.hpp>

#include <atomic>
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

    js.parallel_for(0, count, 100, [&](uint32_t i)
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

    js.parallel_for(0, 100, 10, [&order](uint32_t i)
                    {
                        order.push_back(i);
                    });

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
    js.parallel_for(0, 5, 10, [&order](uint32_t i)
                    {
                        order.push_back(i);
                    });

    REQUIRE(order.size() == 5);
    for (uint32_t i = 0; i < 5; ++i)
    {
        REQUIRE(order[i] == i);
    }

    js.shutdown();
}

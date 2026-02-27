#include <catch2/catch_test_macros.hpp>

#include <core/jobs/job_system.hpp>

#include <atomic>
#include <vector>

using namespace phynity::jobs;

TEST_CASE("JobSystem basic submission and wait", "[jobs]") {
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

TEST_CASE("JobSystem multiple jobs", "[jobs]") {
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> counter{0};
    std::vector<JobHandle> handles;

    for (int i = 0; i < 10; ++i) {
        handles.push_back(js.submit([&counter] { counter++; }));
    }

    js.wait_all(handles);
    REQUIRE(counter == 10);

    js.shutdown();
}

TEST_CASE("JobSystem deterministic mode", "[jobs]") {
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Deterministic};
    JobSystem js(config);

    REQUIRE(js.scheduling_mode() == SchedulingMode::Deterministic);

    std::atomic<int> counter{0};
    std::vector<JobHandle> handles;

    for (int i = 0; i < 5; ++i) {
        handles.push_back(js.submit([&counter] { counter++; }));
    }

    js.wait_all(handles);
    REQUIRE(counter == 5);

    js.shutdown();
}

TEST_CASE("JobSystem parallel_for", "[jobs]") {
    JobSystem js;
    std::vector<int> results(10, 0);

    js.parallel_for(0, 10, 1, [&results](uint32_t i) { results[static_cast<size_t>(i)] = static_cast<int>(i) * 2; });

    for (size_t i = 0; i < 10; ++i) {
        REQUIRE(results[i] == static_cast<int>(i) * 2);
    }

    js.shutdown();
}

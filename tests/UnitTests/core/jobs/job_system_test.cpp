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
    auto id = js.submit([&counter] { counter++; });

    REQUIRE(id.valid());
    js.wait(id);

    REQUIRE(counter == 1);

    js.shutdown();
    REQUIRE(!js.is_running());
}

TEST_CASE("JobSystem multiple jobs", "[jobs]")
{
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> counter{0};
    std::vector<JobId> ids;
    ids.reserve(10);

    for (int i = 0; i < 10; ++i)
    {
        ids.push_back(js.submit([&counter] { counter++; }));
    }

    js.wait_all(ids);
    REQUIRE(counter == 10);

    js.shutdown();
}

TEST_CASE("JobSystem deterministic mode", "[jobs]")
{
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Deterministic};
    JobSystem js(config);

    REQUIRE(js.scheduling_mode() == SchedulingMode::Deterministic);

    std::atomic<int> counter{0};
    std::vector<JobId> ids;
    ids.reserve(5);

    for (int i = 0; i < 5; ++i)
    {
        ids.push_back(js.submit([&counter] { counter++; }));
    }

    js.wait_all(ids);
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

    REQUIRE(visit_count.load() == count);
    for (uint32_t i = 0; i < count; ++i)
    {
        REQUIRE(visited[i].load() == 1);
    }

    js.shutdown();
}

TEST_CASE("JobSystem parallel_for deterministic mode runs serially", "[jobs]")
{
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Deterministic};
    JobSystem js(config);
    REQUIRE(js.is_running());

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
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::vector<uint32_t> order;
    order.reserve(5);

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
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);
    REQUIRE(js.is_running());

    constexpr uint32_t total_jobs = 8000;
    std::atomic<uint32_t> counter{0};

    constexpr uint32_t num_submitters = 4;
    constexpr uint32_t jobs_per_submitter = total_jobs / num_submitters;
    std::vector<std::thread> submitters;
    submitters.reserve(num_submitters);

    for (uint32_t t = 0; t < num_submitters; ++t)
    {
        submitters.emplace_back(
            [&js, &counter]
            {
                std::vector<JobId> ids;
                ids.reserve(jobs_per_submitter);

                for (uint32_t i = 0; i < jobs_per_submitter; ++i)
                {
                    ids.push_back(js.submit([&counter] { counter.fetch_add(1, std::memory_order_relaxed); }));
                }

                js.wait_all(ids);
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
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> value{0};
    auto id = js.submit([&value] { value.store(42, std::memory_order_relaxed); });
    js.wait(id);
    REQUIRE(value.load() == 42);

    // Second wait should return immediately
    js.wait(id);
    REQUIRE(value.load() == 42);

    js.shutdown();
}

TEST_CASE("JobSystem shutdown unblocks waiting threads", "[jobs]")
{
    JobSystemConfig config{.worker_count = 1, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<bool> blocker_started{false};
    std::atomic<bool> blocker_done{false};

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

    while (!blocker_started.load(std::memory_order_acquire))
    {
        std::this_thread::yield();
    }

    auto queued_id = js.submit([] {});

    std::thread waiter(
        [&js, queued_id, &blocker_done]
        {
            js.wait(queued_id);
            blocker_done.store(true, std::memory_order_release);
        });

    release_blocker.store(true, std::memory_order_release);
    js.shutdown();

    waiter.join();
    REQUIRE(blocker_done.load());
}

// =============================================================================
// Counter-based tests
// =============================================================================

TEST_CASE("JobSystem counter-based wait", "[jobs][counter]")
{
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> counter{0};
    auto done = js.create_counter(5);

    for (int i = 0; i < 5; ++i)
    {
        js.submit([&counter] { counter.fetch_add(1); }, done);
    }

    js.wait(done);
    REQUIRE(counter.load() == 5);

    js.shutdown();
}

// =============================================================================
// Graph submission tests
// =============================================================================

static void increment_fn(void *data)
{
    static_cast<std::atomic<int> *>(data)->fetch_add(1);
}

TEST_CASE("JobSystem submit_graph empty graph", "[jobs][graph]")
{
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    JobGraph graph;
    auto counter = js.submit_graph(graph);

    // Empty graph returns invalid counter
    REQUIRE_FALSE(counter.valid());

    js.shutdown();
}

TEST_CASE("JobSystem submit_graph single job", "[jobs][graph]")
{
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> counter{0};
    JobGraph graph;
    graph.add({.function = increment_fn, .data = &counter, .debug_name = "single"});

    auto done = js.submit_graph(graph);
    js.wait(done);

    REQUIRE(counter.load() == 1);

    js.shutdown();
}

TEST_CASE("JobSystem submit_graph diamond DAG respects dependencies", "[jobs][graph]")
{
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> counter{0};
    int order_a = -1, order_b = -1, order_c = -1, order_d = -1;

    auto make_fn = [](int *order_ptr, std::atomic<int> *counter_ptr)
    {
        return [order_ptr, counter_ptr](void *)
        { *order_ptr = counter_ptr->fetch_add(1); };
    };

    // Can't use lambdas with captures as JobFnPtr directly, so use a different approach
    struct OrderData
    {
        int *order;
        std::atomic<int> *counter;
    };
    auto order_fn = [](void *data)
    {
        auto *d = static_cast<OrderData *>(data);
        *d->order = d->counter->fetch_add(1);
    };

    OrderData data_a{&order_a, &counter};
    OrderData data_b{&order_b, &counter};
    OrderData data_c{&order_c, &counter};
    OrderData data_d{&order_d, &counter};

    JobGraph graph;
    auto a = graph.add({.function = +order_fn, .data = &data_a, .debug_name = "A"});
    auto b = graph.add({.function = +order_fn, .data = &data_b, .debug_name = "B"});
    auto c = graph.add({.function = +order_fn, .data = &data_c, .debug_name = "C"});
    auto d = graph.add({.function = +order_fn, .data = &data_d, .debug_name = "D"});

    graph.depend(a, b);
    graph.depend(a, c);
    graph.depend(b, d);
    graph.depend(c, d);

    auto done = js.submit_graph(graph);
    js.wait(done);

    REQUIRE(counter.load() == 4);
    REQUIRE(order_a < order_b);
    REQUIRE(order_a < order_c);
    REQUIRE(order_d > order_b);
    REQUIRE(order_d > order_c);

    js.shutdown();
}

TEST_CASE("JobSystem submit_graph deterministic mode", "[jobs][graph]")
{
    JobSystemConfig config{.worker_count = 2, .mode = SchedulingMode::Deterministic};
    JobSystem js(config);

    std::vector<uint32_t> execution_order;

    struct PushData
    {
        std::vector<uint32_t> *order;
        uint32_t value;
    };
    auto push_fn = [](void *data)
    {
        auto *d = static_cast<PushData *>(data);
        d->order->push_back(d->value);
    };

    PushData data_a{&execution_order, 0};
    PushData data_b{&execution_order, 1};
    PushData data_c{&execution_order, 2};
    PushData data_d{&execution_order, 3};

    JobGraph graph;
    auto a = graph.add({.function = +push_fn, .data = &data_a});
    auto b = graph.add({.function = +push_fn, .data = &data_b});
    auto c = graph.add({.function = +push_fn, .data = &data_c});
    auto d = graph.add({.function = +push_fn, .data = &data_d});

    graph.depend(a, b);
    graph.depend(a, c);
    graph.depend(b, d);
    graph.depend(c, d);

    auto done = js.submit_graph(graph);
    // Deterministic mode returns invalid counter (executes inline)
    REQUIRE_FALSE(done.valid());

    // In deterministic mode: A(0), B(1), C(2), D(3) — Kahn's with stable tie-breaking
    REQUIRE(execution_order.size() == 4);
    REQUIRE(execution_order[0] == 0); // A
    REQUIRE(execution_order[1] == 1); // B
    REQUIRE(execution_order[2] == 2); // C
    REQUIRE(execution_order[3] == 3); // D

    js.shutdown();
}

TEST_CASE("JobSystem submit_graph all independent tasks", "[jobs][graph]")
{
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    std::atomic<int> counter{0};
    constexpr int task_count = 16;

    JobGraph graph;
    for (int i = 0; i < task_count; ++i)
    {
        graph.add({.function = increment_fn, .data = &counter});
    }

    auto done = js.submit_graph(graph);
    js.wait(done);

    REQUIRE(counter.load() == task_count);

    js.shutdown();
}

TEST_CASE("JobSystem submit_graph physics-like pipeline", "[jobs][graph]")
{
    JobSystemConfig config{.worker_count = 4, .mode = SchedulingMode::Concurrent};
    JobSystem js(config);

    // Simulate: clear(4) -> forces(4) -> integrate(4) -> collisions(1)
    constexpr uint32_t items = 100;
    constexpr uint32_t parts = 4;
    std::atomic<int> counter{0};

    auto noop_fn = [](void *data) { static_cast<std::atomic<int> *>(data)->fetch_add(1); };

    JobGraph graph;

    auto clear = graph.add_partitioned(
        items, parts, {},
        [&](uint32_t s, uint32_t e) -> JobDesc { return {.function = +noop_fn, .data = &counter}; },
        "clear");

    auto forces = graph.add_partitioned(
        items, parts, clear,
        [&](uint32_t s, uint32_t e) -> JobDesc { return {.function = +noop_fn, .data = &counter}; },
        "forces");

    auto integrate = graph.add_partitioned(
        items, parts, forces,
        [&](uint32_t s, uint32_t e) -> JobDesc { return {.function = +noop_fn, .data = &counter}; },
        "integrate");

    graph.add_serial_after(integrate, {.function = +noop_fn, .data = &counter, .debug_name = "collisions"});

    auto done = js.submit_graph(graph);
    js.wait(done);

    REQUIRE(counter.load() == 13); // 4+4+4+1

    js.shutdown();
}

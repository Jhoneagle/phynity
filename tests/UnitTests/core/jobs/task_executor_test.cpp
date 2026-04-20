#include <catch2/catch_test_macros.hpp>
#include <core/jobs/task_executor.hpp>

#include <atomic>
#include <vector>

using namespace phynity::jobs;

TEST_CASE("TaskExecutor empty graph", "[jobs][task_executor]")
{
    JobSystem js;
    js.start({.worker_count = 2, .mode = SchedulingMode::Concurrent});

    TaskExecutor executor(js);
    TaskGraph graph;

    // Should not crash
    executor.execute(graph);

    js.shutdown();
}

TEST_CASE("TaskExecutor single task executes", "[jobs][task_executor]")
{
    JobSystem js;
    js.start({.worker_count = 2, .mode = SchedulingMode::Concurrent});

    TaskExecutor executor(js);
    TaskGraph graph;

    bool executed = false;
    graph.add_task({.fn = [&] { executed = true; }});

    executor.execute(graph);
    REQUIRE(executed);

    js.shutdown();
}

TEST_CASE("TaskExecutor diamond DAG respects dependencies", "[jobs][task_executor]")
{
    JobSystem js;
    js.start({.worker_count = 2, .mode = SchedulingMode::Concurrent});

    TaskExecutor executor(js);
    TaskGraph graph;

    // Track completion order using an atomic counter
    std::atomic<int> counter{0};
    int order_a = -1, order_b = -1, order_c = -1, order_d = -1;

    auto a = graph.add_task({.fn = [&] { order_a = counter.fetch_add(1); }});
    auto b = graph.add_task({.fn = [&] { order_b = counter.fetch_add(1); }});
    auto c = graph.add_task({.fn = [&] { order_c = counter.fetch_add(1); }});
    auto d = graph.add_task({.fn = [&] { order_d = counter.fetch_add(1); }});

    graph.add_dependency(a, b);
    graph.add_dependency(a, c);
    graph.add_dependency(b, d);
    graph.add_dependency(c, d);

    executor.execute(graph);

    // All tasks executed
    REQUIRE(counter.load() == 4);

    // A must come before B and C
    REQUIRE(order_a < order_b);
    REQUIRE(order_a < order_c);

    // D must come after both B and C
    REQUIRE(order_d > order_b);
    REQUIRE(order_d > order_c);
}

TEST_CASE("TaskExecutor serial mode executes in sequence order", "[jobs][task_executor]")
{
    JobSystem js;
    js.start({.worker_count = 2, .mode = SchedulingMode::Deterministic});

    TaskExecutor executor(js);
    TaskGraph graph;

    std::vector<uint32_t> execution_order;

    auto a = graph.add_task({.fn = [&] { execution_order.push_back(0); }});
    auto b = graph.add_task({.fn = [&] { execution_order.push_back(1); }});
    auto c = graph.add_task({.fn = [&] { execution_order.push_back(2); }});
    auto d = graph.add_task({.fn = [&] { execution_order.push_back(3); }});

    graph.add_dependency(a, b);
    graph.add_dependency(a, c);
    graph.add_dependency(b, d);
    graph.add_dependency(c, d);

    executor.execute(graph);

    // In deterministic mode: A(seq=0), B(seq=1), C(seq=2), D(seq=3)
    REQUIRE(execution_order.size() == 4);
    REQUIRE(execution_order[0] == 0); // A
    REQUIRE(execution_order[1] == 1); // B
    REQUIRE(execution_order[2] == 2); // C
    REQUIRE(execution_order[3] == 3); // D

    js.shutdown();
}

TEST_CASE("TaskExecutor execute_serial without job system running", "[jobs][task_executor]")
{
    JobSystem js; // not started
    TaskExecutor executor(js);
    TaskGraph graph;

    int sum = 0;
    graph.add_task({.fn = [&] { sum += 10; }});
    graph.add_task({.fn = [&] { sum += 20; }});

    executor.execute(graph);

    REQUIRE(sum == 30);
}

TEST_CASE("TaskExecutor all tasks in same tier run", "[jobs][task_executor]")
{
    JobSystem js;
    js.start({.worker_count = 4, .mode = SchedulingMode::Concurrent});

    TaskExecutor executor(js);
    TaskGraph graph;

    std::atomic<int> counter{0};
    constexpr int task_count = 16;

    for (int i = 0; i < task_count; ++i)
    {
        graph.add_task({.fn = [&] { counter.fetch_add(1); }});
    }

    executor.execute(graph);

    REQUIRE(counter.load() == task_count);

    js.shutdown();
}

TEST_CASE("TaskExecutor pre-built schedule", "[jobs][task_executor]")
{
    JobSystem js;
    js.start({.worker_count = 2, .mode = SchedulingMode::Concurrent});

    TaskExecutor executor(js);
    TaskGraph graph;

    std::atomic<int> counter{0};
    graph.add_task({.fn = [&] { counter.fetch_add(1); }});
    graph.add_task({.fn = [&] { counter.fetch_add(1); }});

    auto schedule = TaskScheduler::build_schedule(graph);

    // Execute using pre-built schedule
    executor.execute(schedule, graph);

    REQUIRE(counter.load() == 2);

    js.shutdown();
}

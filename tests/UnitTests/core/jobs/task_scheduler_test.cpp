#include <catch2/catch_test_macros.hpp>
#include <core/jobs/task_scheduler.hpp>

using namespace phynity::jobs;

TEST_CASE("TaskScheduler empty graph", "[jobs][task_scheduler]")
{
    TaskGraph graph;
    auto schedule = TaskScheduler::build_schedule(graph);

    REQUIRE(schedule.entries.empty());
    REQUIRE(schedule.tier_count == 0);
}

TEST_CASE("TaskScheduler single task", "[jobs][task_scheduler]")
{
    TaskGraph graph;
    auto a = graph.add_task({.fn = [] {}});

    auto schedule = TaskScheduler::build_schedule(graph);

    REQUIRE(schedule.entries.size() == 1);
    REQUIRE(schedule.tier_count == 1);
    REQUIRE(schedule.entries[0].id == a);
    REQUIRE(schedule.entries[0].tier == 0);
    REQUIRE(schedule.entries[0].sequence == 0);
}

TEST_CASE("TaskScheduler diamond DAG", "[jobs][task_scheduler]")
{
    //   A
    //  / \.
    // B   C
    //  \ /
    //   D
    TaskGraph graph;
    auto a = graph.add_task({.fn = [] {}});
    auto b = graph.add_task({.fn = [] {}});
    auto c = graph.add_task({.fn = [] {}});
    auto d = graph.add_task({.fn = [] {}});

    graph.add_dependency(a, b);
    graph.add_dependency(a, c);
    graph.add_dependency(b, d);
    graph.add_dependency(c, d);

    auto schedule = TaskScheduler::build_schedule(graph);

    REQUIRE(schedule.entries.size() == 4);
    REQUIRE(schedule.tier_count == 3);

    // Tier 0: A
    REQUIRE(schedule.entries[0].id == a);
    REQUIRE(schedule.entries[0].tier == 0);

    // Tier 1: B then C (sorted by TaskId value: B=1, C=2)
    REQUIRE(schedule.entries[1].id == b);
    REQUIRE(schedule.entries[1].tier == 1);
    REQUIRE(schedule.entries[2].id == c);
    REQUIRE(schedule.entries[2].tier == 1);

    // Tier 2: D
    REQUIRE(schedule.entries[3].id == d);
    REQUIRE(schedule.entries[3].tier == 2);

    // Sequence numbers are monotonic
    for (uint32_t i = 0; i < schedule.entries.size(); ++i)
    {
        REQUIRE(schedule.entries[i].sequence == i);
    }
}

TEST_CASE("TaskScheduler linear chain", "[jobs][task_scheduler]")
{
    // A -> B -> C -> D
    TaskGraph graph;
    auto a = graph.add_task({.fn = [] {}});
    auto b = graph.add_task({.fn = [] {}});
    auto c = graph.add_task({.fn = [] {}});
    auto d = graph.add_task({.fn = [] {}});

    graph.add_dependency(a, b);
    graph.add_dependency(b, c);
    graph.add_dependency(c, d);

    auto schedule = TaskScheduler::build_schedule(graph);

    REQUIRE(schedule.entries.size() == 4);
    REQUIRE(schedule.tier_count == 4);

    // Each task in its own tier
    for (uint32_t i = 0; i < 4; ++i)
    {
        REQUIRE(schedule.entries[i].tier == i);
    }
}

TEST_CASE("TaskScheduler two independent tasks", "[jobs][task_scheduler]")
{
    TaskGraph graph;
    auto a = graph.add_task({.fn = [] {}});
    auto b = graph.add_task({.fn = [] {}});

    auto schedule = TaskScheduler::build_schedule(graph);

    REQUIRE(schedule.entries.size() == 2);
    REQUIRE(schedule.tier_count == 1);

    // Both in tier 0, ordered by TaskId
    REQUIRE(schedule.entries[0].id == a);
    REQUIRE(schedule.entries[0].tier == 0);
    REQUIRE(schedule.entries[1].id == b);
    REQUIRE(schedule.entries[1].tier == 0);
}

TEST_CASE("TaskScheduler deterministic output", "[jobs][task_scheduler]")
{
    // Running build_schedule twice on the same graph must produce identical results
    TaskGraph graph;
    auto a = graph.add_task({.fn = [] {}});
    auto b = graph.add_task({.fn = [] {}});
    auto c = graph.add_task({.fn = [] {}});

    graph.add_dependency(a, b);
    graph.add_dependency(a, c);

    auto s1 = TaskScheduler::build_schedule(graph);
    auto s2 = TaskScheduler::build_schedule(graph);

    REQUIRE(s1.entries.size() == s2.entries.size());
    REQUIRE(s1.tier_count == s2.tier_count);

    for (size_t i = 0; i < s1.entries.size(); ++i)
    {
        REQUIRE(s1.entries[i].id == s2.entries[i].id);
        REQUIRE(s1.entries[i].tier == s2.entries[i].tier);
        REQUIRE(s1.entries[i].sequence == s2.entries[i].sequence);
    }
}

TEST_CASE("TaskScheduler wide fan-out", "[jobs][task_scheduler]")
{
    // Root -> 8 independent leaves
    TaskGraph graph;
    auto root = graph.add_task({.fn = [] {}});

    std::vector<TaskId> leaves;
    for (int i = 0; i < 8; ++i)
    {
        auto leaf = graph.add_task({.fn = [] {}});
        graph.add_dependency(root, leaf);
        leaves.push_back(leaf);
    }

    auto schedule = TaskScheduler::build_schedule(graph);

    REQUIRE(schedule.entries.size() == 9);
    REQUIRE(schedule.tier_count == 2);

    // Root in tier 0
    REQUIRE(schedule.entries[0].id == root);
    REQUIRE(schedule.entries[0].tier == 0);

    // All 8 leaves in tier 1, sorted by TaskId
    for (uint32_t i = 1; i <= 8; ++i)
    {
        REQUIRE(schedule.entries[i].tier == 1);
        REQUIRE(schedule.entries[i].id == leaves[i - 1]);
    }
}

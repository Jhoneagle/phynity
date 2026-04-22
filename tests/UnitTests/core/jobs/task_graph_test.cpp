#include <catch2/catch_test_macros.hpp>
#include <core/jobs/task_graph.hpp>

using namespace phynity::jobs;

TEST_CASE("TaskGraph empty graph validates", "[jobs][task_graph]")
{
    TaskGraph graph;

    REQUIRE(graph.task_count() == 0);
    REQUIRE(graph.validate());
    REQUIRE(graph.roots().empty());
}

TEST_CASE("TaskGraph single task", "[jobs][task_graph]")
{
    TaskGraph graph;
    bool executed = false;

    auto id = graph.add_task({.fn = [&] { executed = true; }, .debug_name = "single"});

    REQUIRE(id.valid());
    REQUIRE(id.value == 0);
    REQUIRE(graph.task_count() == 1);
    REQUIRE(graph.predecessor_count(id) == 0);
    REQUIRE(graph.dependents(id).empty());
    REQUIRE(graph.validate());

    auto roots = graph.roots();
    REQUIRE(roots.size() == 1);
    REQUIRE(roots[0] == id);

    graph.descriptor(id).fn();
    REQUIRE(executed);
}

TEST_CASE("TaskGraph diamond DAG", "[jobs][task_graph]")
{
    //   A
    //  / \.
    // B   C
    //  \ /
    //   D
    TaskGraph graph;

    auto a = graph.add_task({.fn = [] {}, .debug_name = "A"});
    auto b = graph.add_task({.fn = [] {}, .debug_name = "B"});
    auto c = graph.add_task({.fn = [] {}, .debug_name = "C"});
    auto d = graph.add_task({.fn = [] {}, .debug_name = "D"});

    graph.add_dependency(a, b);
    graph.add_dependency(a, c);
    graph.add_dependency(b, d);
    graph.add_dependency(c, d);

    REQUIRE(graph.task_count() == 4);
    REQUIRE(graph.validate());

    // Roots
    auto roots = graph.roots();
    REQUIRE(roots.size() == 1);
    REQUIRE(roots[0] == a);

    // Predecessor counts
    REQUIRE(graph.predecessor_count(a) == 0);
    REQUIRE(graph.predecessor_count(b) == 1);
    REQUIRE(graph.predecessor_count(c) == 1);
    REQUIRE(graph.predecessor_count(d) == 2);

    // Dependents of A
    auto a_deps = graph.dependents(a);
    REQUIRE(a_deps.size() == 2);
    REQUIRE(a_deps[0] == b);
    REQUIRE(a_deps[1] == c);

    // Dependents of B and C each point to D
    REQUIRE(graph.dependents(b).size() == 1);
    REQUIRE(graph.dependents(b)[0] == d);
    REQUIRE(graph.dependents(c).size() == 1);
    REQUIRE(graph.dependents(c)[0] == d);

    // D has no dependents
    REQUIRE(graph.dependents(d).empty());
}

TEST_CASE("TaskGraph cycle detection", "[jobs][task_graph]")
{
    SECTION("simple cycle A -> B -> C -> A")
    {
        TaskGraph graph;
        auto a = graph.add_task({.fn = [] {}});
        auto b = graph.add_task({.fn = [] {}});
        auto c = graph.add_task({.fn = [] {}});

        graph.add_dependency(a, b);
        graph.add_dependency(b, c);
        graph.add_dependency(c, a);

        REQUIRE_FALSE(graph.validate());
    }

    SECTION("self-loop is prevented by assertion but two-node cycle detected")
    {
        TaskGraph graph;
        auto a = graph.add_task({.fn = [] {}});
        auto b = graph.add_task({.fn = [] {}});

        graph.add_dependency(a, b);
        graph.add_dependency(b, a);

        REQUIRE_FALSE(graph.validate());
    }
}

TEST_CASE("TaskGraph two independent tasks", "[jobs][task_graph]")
{
    TaskGraph graph;
    auto a = graph.add_task({.fn = [] {}, .debug_name = "A"});
    auto b = graph.add_task({.fn = [] {}, .debug_name = "B"});

    REQUIRE(graph.validate());

    auto roots = graph.roots();
    REQUIRE(roots.size() == 2);
    REQUIRE(roots[0] == a);
    REQUIRE(roots[1] == b);
}

TEST_CASE("TaskGraph affinity hints are stored", "[jobs][task_graph]")
{
    TaskGraph graph;
    auto a = graph.add_task({.fn = [] {}, .affinity_hint = 0, .debug_name = "pinned_0"});
    auto b = graph.add_task({.fn = [] {}, .affinity_hint = 3, .debug_name = "pinned_3"});

    REQUIRE(graph.descriptor(a).affinity_hint == 0);
    REQUIRE(graph.descriptor(b).affinity_hint == 3);
}

TEST_CASE("TaskGraph clear resets state", "[jobs][task_graph]")
{
    TaskGraph graph;
    graph.add_task({.fn = [] {}});
    graph.add_task({.fn = [] {}});

    REQUIRE(graph.task_count() == 2);

    graph.clear();

    REQUIRE(graph.task_count() == 0);
    REQUIRE(graph.roots().empty());
    REQUIRE(graph.validate());
}

TEST_CASE("TaskGraph linear chain", "[jobs][task_graph]")
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

    REQUIRE(graph.validate());

    auto roots = graph.roots();
    REQUIRE(roots.size() == 1);
    REQUIRE(roots[0] == a);

    REQUIRE(graph.predecessor_count(a) == 0);
    REQUIRE(graph.predecessor_count(b) == 1);
    REQUIRE(graph.predecessor_count(c) == 1);
    REQUIRE(graph.predecessor_count(d) == 1);
}

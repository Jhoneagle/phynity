#include <catch2/catch_test_macros.hpp>
#include <core/jobs/job_graph.hpp>

using namespace phynity::jobs;

static void noop_fn(void * /*data*/)
{
}

TEST_CASE("JobGraph empty graph validates", "[jobs][job_graph]")
{
    JobGraph graph;

    REQUIRE(graph.size() == 0);
    REQUIRE(graph.validate());
    REQUIRE(graph.roots().empty());
}

TEST_CASE("JobGraph single job", "[jobs][job_graph]")
{
    JobGraph graph;
    bool executed = false;
    auto fn = [](void *ctx) { *static_cast<bool *>(ctx) = true; };

    auto id = graph.add({.function = fn, .data = &executed, .debug_name = "single"});

    REQUIRE(id.valid());
    REQUIRE(id.index == 0);
    REQUIRE(graph.size() == 1);
    REQUIRE(graph.predecessor_count(id) == 0);
    REQUIRE(graph.dependents(id).empty());
    REQUIRE(graph.validate());

    auto roots = graph.roots();
    REQUIRE(roots.size() == 1);
    REQUIRE(roots[0] == id);

    // Execute manually
    const auto &desc = graph.desc(id);
    desc.function(desc.data);
    REQUIRE(executed);
}

TEST_CASE("JobGraph diamond DAG", "[jobs][job_graph]")
{
    //   A
    //  / \.
    // B   C
    //  \ /
    //   D
    JobGraph graph;

    auto a = graph.add({.function = noop_fn, .debug_name = "A"});
    auto b = graph.add({.function = noop_fn, .debug_name = "B"});
    auto c = graph.add({.function = noop_fn, .debug_name = "C"});
    auto d = graph.add({.function = noop_fn, .debug_name = "D"});

    graph.depend(a, b);
    graph.depend(a, c);
    graph.depend(b, d);
    graph.depend(c, d);

    REQUIRE(graph.size() == 4);
    REQUIRE(graph.validate());

    auto roots = graph.roots();
    REQUIRE(roots.size() == 1);
    REQUIRE(roots[0] == a);

    REQUIRE(graph.predecessor_count(a) == 0);
    REQUIRE(graph.predecessor_count(b) == 1);
    REQUIRE(graph.predecessor_count(c) == 1);
    REQUIRE(graph.predecessor_count(d) == 2);

    auto a_deps = graph.dependents(a);
    REQUIRE(a_deps.size() == 2);
    REQUIRE(a_deps[0] == b);
    REQUIRE(a_deps[1] == c);

    REQUIRE(graph.dependents(b).size() == 1);
    REQUIRE(graph.dependents(b)[0] == d);
    REQUIRE(graph.dependents(c).size() == 1);
    REQUIRE(graph.dependents(c)[0] == d);

    REQUIRE(graph.dependents(d).empty());
}

TEST_CASE("JobGraph cycle detection", "[jobs][job_graph]")
{
    SECTION("simple cycle A -> B -> C -> A")
    {
        JobGraph graph;
        auto a = graph.add({.function = noop_fn});
        auto b = graph.add({.function = noop_fn});
        auto c = graph.add({.function = noop_fn});

        graph.depend(a, b);
        graph.depend(b, c);
        graph.depend(c, a);

        REQUIRE_FALSE(graph.validate());
    }

    SECTION("two-node cycle")
    {
        JobGraph graph;
        auto a = graph.add({.function = noop_fn});
        auto b = graph.add({.function = noop_fn});

        graph.depend(a, b);
        graph.depend(b, a);

        REQUIRE_FALSE(graph.validate());
    }
}

TEST_CASE("JobGraph self-loop depend is rejected by assert", "[jobs][job_graph]")
{
    // depend(a, a) hits assert(before.index != after.index) in debug builds.
    // In release builds the assert is stripped, so we verify the graph still
    // validates (self-loop creates a cycle that validate() detects).
    JobGraph graph;
    auto a = graph.add({.function = noop_fn});

#ifdef NDEBUG
    // Release build: assert is stripped, depend succeeds but creates a cycle
    graph.depend(a, a);
    REQUIRE_FALSE(graph.validate());
#else
    // Debug build: just verify the node exists and has no self-dependency
    REQUIRE(graph.predecessor_count(a) == 0);
    REQUIRE(graph.dependents(a).empty());
#endif
}

TEST_CASE("JobGraph two independent jobs", "[jobs][job_graph]")
{
    JobGraph graph;
    auto a = graph.add({.function = noop_fn, .debug_name = "A"});
    auto b = graph.add({.function = noop_fn, .debug_name = "B"});

    REQUIRE(graph.validate());

    auto roots = graph.roots();
    REQUIRE(roots.size() == 2);
    REQUIRE(roots[0] == a);
    REQUIRE(roots[1] == b);
}

TEST_CASE("JobGraph affinity hints stored", "[jobs][job_graph]")
{
    JobGraph graph;
    auto a = graph.add({.function = noop_fn, .affinity_hint = 0, .debug_name = "pinned_0"});
    auto b = graph.add({.function = noop_fn, .affinity_hint = 3, .debug_name = "pinned_3"});

    REQUIRE(graph.desc(a).affinity_hint == 0);
    REQUIRE(graph.desc(b).affinity_hint == 3);
}

TEST_CASE("JobGraph clear resets state", "[jobs][job_graph]")
{
    JobGraph graph;
    (void) graph.add({.function = noop_fn});
    (void) graph.add({.function = noop_fn});

    REQUIRE(graph.size() == 2);

    graph.clear();

    REQUIRE(graph.size() == 0);
    REQUIRE(graph.roots().empty());
    REQUIRE(graph.validate());
}

TEST_CASE("JobGraph linear chain", "[jobs][job_graph]")
{
    // A -> B -> C -> D
    JobGraph graph;
    auto a = graph.add({.function = noop_fn});
    auto b = graph.add({.function = noop_fn});
    auto c = graph.add({.function = noop_fn});
    auto d = graph.add({.function = noop_fn});

    graph.depend(a, b);
    graph.depend(b, c);
    graph.depend(c, d);

    REQUIRE(graph.validate());

    auto roots = graph.roots();
    REQUIRE(roots.size() == 1);
    REQUIRE(roots[0] == a);

    REQUIRE(graph.predecessor_count(a) == 0);
    REQUIRE(graph.predecessor_count(b) == 1);
    REQUIRE(graph.predecessor_count(c) == 1);
    REQUIRE(graph.predecessor_count(d) == 1);
}

TEST_CASE("JobGraph wide fan-out", "[jobs][job_graph]")
{
    // Root -> 8 independent leaves
    JobGraph graph;
    auto root = graph.add({.function = noop_fn});

    std::vector<JobId> leaves;
    for (int i = 0; i < 8; ++i)
    {
        auto leaf = graph.add({.function = noop_fn});
        graph.depend(root, leaf);
        leaves.push_back(leaf);
    }

    REQUIRE(graph.size() == 9);
    REQUIRE(graph.validate());

    auto roots = graph.roots();
    REQUIRE(roots.size() == 1);
    REQUIRE(roots[0] == root);

    for (const auto &leaf : leaves)
    {
        REQUIRE(graph.predecessor_count(leaf) == 1);
    }

    REQUIRE(graph.dependents(root).size() == 8);
}

TEST_CASE("JobGraph duplicate edge is silently ignored", "[jobs][job_graph]")
{
    // Duplicate edges are rejected to prevent inflated predecessor counts
    // that would cause jobs to never become ready during graph execution.
    JobGraph graph;
    auto a = graph.add({.function = noop_fn});
    auto b = graph.add({.function = noop_fn});

    graph.depend(a, b);
    graph.depend(a, b); // duplicate — should be ignored

    REQUIRE(graph.predecessor_count(b) == 1);
    REQUIRE(graph.dependents(a).size() == 1);
    REQUIRE(graph.validate());
}

TEST_CASE("JobGraph wide fan-out exceeding kMaxInlineDependents", "[jobs][job_graph]")
{
    // kMaxInlineDependents = 6. A root with 10 children exercises the overflow
    // path when the graph is later submitted through submit_graph.
    JobGraph graph;
    auto root = graph.add({.function = noop_fn, .debug_name = "root"});

    constexpr int child_count = 10;
    std::vector<JobId> children;
    for (int i = 0; i < child_count; ++i)
    {
        auto child = graph.add({.function = noop_fn});
        graph.depend(root, child);
        children.push_back(child);
    }

    REQUIRE(graph.size() == 11);
    REQUIRE(graph.validate());
    REQUIRE(graph.dependents(root).size() == child_count);

    auto roots = graph.roots();
    REQUIRE(roots.size() == 1);
    REQUIRE(roots[0] == root);

    for (const auto &child : children)
    {
        REQUIRE(graph.predecessor_count(child) == 1);
    }
}

// ========================================================================
// Builder helper tests
// ========================================================================

TEST_CASE("JobGraph partition_range divides evenly", "[jobs][job_graph]")
{
    auto [s0, e0] = JobGraph::partition_range(12, 3, 0);
    auto [s1, e1] = JobGraph::partition_range(12, 3, 1);
    auto [s2, e2] = JobGraph::partition_range(12, 3, 2);

    REQUIRE(s0 == 0);
    REQUIRE(e0 == 4);
    REQUIRE(s1 == 4);
    REQUIRE(e1 == 8);
    REQUIRE(s2 == 8);
    REQUIRE(e2 == 12);
}

TEST_CASE("JobGraph partition_range uneven division", "[jobs][job_graph]")
{
    auto [s0, e0] = JobGraph::partition_range(10, 3, 0);
    auto [s1, e1] = JobGraph::partition_range(10, 3, 1);
    auto [s2, e2] = JobGraph::partition_range(10, 3, 2);

    REQUIRE(s0 == 0);
    REQUIRE(e0 == 3);
    REQUIRE(s1 == 3);
    REQUIRE(e1 == 6);
    REQUIRE(s2 == 6);
    REQUIRE(e2 == 10);

    // No gaps or overlaps
    REQUIRE(e0 == s1);
    REQUIRE(e1 == s2);
}

TEST_CASE("JobGraph add_partitioned creates tier with dependencies", "[jobs][job_graph]")
{
    JobGraph graph;
    constexpr uint32_t items = 100;
    constexpr uint32_t parts = 4;

    auto tier0 = graph.add_partitioned(
        items,
        parts,
        {},
        [](uint32_t /*start*/, uint32_t /*end*/) -> JobDesc { return {.function = noop_fn, .debug_name = "tier0"}; },
        "tier0");

    REQUIRE(tier0.size() == parts);
    for (const auto &id : tier0)
    {
        REQUIRE(graph.predecessor_count(id) == 0); // no prev_tier
    }

    auto tier1 = graph.add_partitioned(
        items,
        parts,
        tier0,
        [](uint32_t /*start*/, uint32_t /*end*/) -> JobDesc { return {.function = noop_fn, .debug_name = "tier1"}; },
        "tier1");

    REQUIRE(tier1.size() == parts);
    for (const auto &id : tier1)
    {
        REQUIRE(graph.predecessor_count(id) == 1); // one dep from prev_tier
    }

    REQUIRE(graph.validate());
    REQUIRE(graph.size() == 8);
}

TEST_CASE("JobGraph add_partitioned sets affinity hints", "[jobs][job_graph]")
{
    JobGraph graph;

    auto tier = graph.add_partitioned(
        100, 4, {}, [](uint32_t /*start*/, uint32_t /*end*/) -> JobDesc { return {.function = noop_fn}; }, "test");

    for (uint32_t p = 0; p < 4; ++p)
    {
        REQUIRE(graph.desc(tier[p]).affinity_hint == p);
    }
}

TEST_CASE("JobGraph add_serial_after fans in from all prev_tier", "[jobs][job_graph]")
{
    JobGraph graph;

    auto tier0 = graph.add_partitioned(
        100, 4, {}, [](uint32_t /*start*/, uint32_t /*end*/) -> JobDesc { return {.function = noop_fn}; }, "par");

    auto serial = graph.add_serial_after(tier0, {.function = noop_fn, .debug_name = "serial"});

    REQUIRE(graph.predecessor_count(serial) == 4);
    REQUIRE(graph.validate());
}

TEST_CASE("JobGraph physics-like pipeline structure", "[jobs][job_graph]")
{
    // Simulate: clear -> forces -> integrate -> collisions (serial)
    JobGraph graph;
    constexpr uint32_t items = 50;
    constexpr uint32_t parts = 4;

    auto clear = graph.add_partitioned(
        items, parts, {}, [](uint32_t /*s*/, uint32_t /*e*/) -> JobDesc { return {.function = noop_fn}; }, "clear");

    auto forces = graph.add_partitioned(
        items, parts, clear, [](uint32_t /*s*/, uint32_t /*e*/) -> JobDesc { return {.function = noop_fn}; }, "forces");

    auto integrate = graph.add_partitioned(
        items,
        parts,
        forces,
        [](uint32_t /*s*/, uint32_t /*e*/) -> JobDesc { return {.function = noop_fn}; },
        "integrate");

    auto collisions = graph.add_serial_after(integrate, {.function = noop_fn, .debug_name = "collisions"});

    REQUIRE(graph.size() == 13); // 4+4+4+1
    REQUIRE(graph.validate());

    // collisions depends on all 4 integrate partitions
    REQUIRE(graph.predecessor_count(JobId{static_cast<uint32_t>(collisions.index), 0}) == 4);

    // Each partition chain: clear[p] -> forces[p] -> integrate[p]
    for (uint32_t p = 0; p < parts; ++p)
    {
        REQUIRE(graph.predecessor_count(forces[p]) == 1);
        REQUIRE(graph.predecessor_count(integrate[p]) == 1);
    }
}

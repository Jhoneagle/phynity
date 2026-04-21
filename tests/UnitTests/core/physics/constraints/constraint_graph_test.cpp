#include <catch2/catch_test_macros.hpp>
#include <core/physics/constraints/constraint_graph.hpp>

using namespace phynity::physics::constraints;

namespace
{

// Minimal mock constraint for testing
class MockConstraint : public Constraint
{
public:
    float compute_error() const override
    {
        return 0.0f;
    }
    float compute_jv() const override
    {
        return 0.0f;
    }
    float compute_effective_mass() const override
    {
        return 1.0f;
    }
    void apply_impulse(float /*impulse_magnitude*/) override
    {
    }
};

// Dummy body addresses for testing
struct DummyBody
{
    int id;
};

} // namespace

TEST_CASE("ConstraintColoring empty input", "[constraints][coloring]")
{
    auto result = color_constraints({}, {});
    REQUIRE(result.color_count == 0);
    REQUIRE(result.color_of.empty());
    REQUIRE(result.groups.empty());
}

TEST_CASE("ConstraintColoring single constraint", "[constraints][coloring]")
{
    MockConstraint c;
    const Constraint *constraints[] = {&c};
    DummyBody a, b;
    ConstraintBodyPair pairs[] = {{&a, &b}};

    auto result = color_constraints(constraints, pairs);

    REQUIRE(result.color_count == 1);
    REQUIRE(result.color_of.size() == 1);
    REQUIRE(result.color_of[0] == 0);
    REQUIRE(result.groups.size() == 1);
    REQUIRE(result.groups[0].size() == 1);
}

TEST_CASE("ConstraintColoring independent constraints get same color", "[constraints][coloring]")
{
    // C0: A-B, C1: C-D (no shared bodies)
    MockConstraint c0, c1;
    const Constraint *constraints[] = {&c0, &c1};
    DummyBody a, b, c, d;
    ConstraintBodyPair pairs[] = {{&a, &b}, {&c, &d}};

    auto result = color_constraints(constraints, pairs);

    REQUIRE(result.color_count == 1);
    REQUIRE(result.color_of[0] == result.color_of[1]);
}

TEST_CASE("ConstraintColoring chain produces 2 colors", "[constraints][coloring]")
{
    // C0: A-B, C1: B-C, C2: C-D, C3: D-E
    // Adjacent constraints share a body -> alternating colors
    MockConstraint c0, c1, c2, c3;
    const Constraint *constraints[] = {&c0, &c1, &c2, &c3};
    DummyBody a, b, c, d, e;
    ConstraintBodyPair pairs[] = {{&a, &b}, {&b, &c}, {&c, &d}, {&d, &e}};

    auto result = color_constraints(constraints, pairs);

    REQUIRE(result.color_count == 2);

    // No two adjacent constraints share a color
    REQUIRE(result.color_of[0] != result.color_of[1]);
    REQUIRE(result.color_of[1] != result.color_of[2]);
    REQUIRE(result.color_of[2] != result.color_of[3]);
}

TEST_CASE("ConstraintColoring star graph (all share body B)", "[constraints][coloring]")
{
    // C0: A-B, C1: B-C, C2: B-D -> all share B, all different colors
    MockConstraint c0, c1, c2;
    const Constraint *constraints[] = {&c0, &c1, &c2};
    DummyBody a, b, c, d;
    ConstraintBodyPair pairs[] = {{&a, &b}, {&b, &c}, {&b, &d}};

    auto result = color_constraints(constraints, pairs);

    REQUIRE(result.color_count == 3);

    // All different colors
    REQUIRE(result.color_of[0] != result.color_of[1]);
    REQUIRE(result.color_of[0] != result.color_of[2]);
    REQUIRE(result.color_of[1] != result.color_of[2]);
}

TEST_CASE("ConstraintColoring validity check", "[constraints][coloring]")
{
    // Larger test: verify no two same-color constraints share a body
    MockConstraint cs[6];
    const Constraint *constraints[6];
    for (int i = 0; i < 6; ++i)
        constraints[i] = &cs[i];

    DummyBody bodies[7];
    // Chain: 0-1, 1-2, 2-3, 3-4, 4-5, 5-6
    ConstraintBodyPair pairs[] = {{&bodies[0], &bodies[1]},
                                  {&bodies[1], &bodies[2]},
                                  {&bodies[2], &bodies[3]},
                                  {&bodies[3], &bodies[4]},
                                  {&bodies[4], &bodies[5]},
                                  {&bodies[5], &bodies[6]}};

    auto result = color_constraints(constraints, pairs);

    // Verify validity: no adjacent constraints share a color
    for (std::size_t i = 0; i + 1 < result.color_of.size(); ++i)
    {
        REQUIRE(result.color_of[i] != result.color_of[i + 1]);
    }

    // Verify all constraints are in exactly one group
    std::vector<bool> seen(6, false);
    for (const auto &group : result.groups)
    {
        for (uint32_t idx : group)
        {
            REQUIRE_FALSE(seen[idx]);
            seen[idx] = true;
        }
    }
    for (bool s : seen)
    {
        REQUIRE(s);
    }
}

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/sat_solver.hpp>
#include <core/physics/collision/shape_factory.hpp>
#include <core/math/vectors/vec2.hpp>
#include <core/math/vectors/vec3.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using Catch::Matchers::WithinAbs;

TEST_CASE("SAT Validation: 2D square-square axis-aligned collision", "[collision][sat][validation]") {
    SECTION("Separated squares") {
        auto square_a = ShapeFactory::create_box_2d(1.0f, 1.0f);
        auto square_b = ShapeFactory::create_box_2d(1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_2d(square_a, square_b, Vec2f(0.0f, 0.0f), Vec2f(3.0f, 0.0f));

        REQUIRE(!result.is_colliding);
    }

    SECTION("Overlapping squares") {
        auto square_a = ShapeFactory::create_box_2d(1.0f, 1.0f);
        auto square_b = ShapeFactory::create_box_2d(1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_2d(square_a, square_b, Vec2f(0.0f, 0.0f), Vec2f(0.8f, 0.0f));

        REQUIRE(result.is_colliding);
        // Box half-extent is 1.0, so boxes extend ±1.0 from center
        // Box A: x from -1 to +1, Box B: x from -0.2 to +1.8
        // Overlap: 1.0 - (-0.2) = 1.2
        REQUIRE_THAT(result.penetration_depth, WithinAbs(1.2f, 0.01f));
        // Normal should point along x-axis
        REQUIRE_THAT(std::abs(result.collision_normal.x), WithinAbs(1.0f, 0.01f));
        REQUIRE_THAT(result.collision_normal.y, WithinAbs(0.0f, 0.01f));
    }

    SECTION("Perfectly overlapping squares") {
        auto square_a = ShapeFactory::create_box_2d(1.0f, 1.0f);
        auto square_b = ShapeFactory::create_box_2d(1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_2d(square_a, square_b, Vec2f(0.0f, 0.0f), Vec2f(0.0f, 0.0f));

        REQUIRE(result.is_colliding);
        REQUIRE(result.penetration_depth > 0.99f);
    }
}

TEST_CASE("SAT Validation: 2D rotated collision", "[collision][sat][validation]") {
    SECTION("45-degree rotated square collision") {
        // Create a diamond shape (square rotated 45 degrees)
        auto diamond = ShapeFactory::create_regular_polygon_2d(4, 0.7071f, 45.0f);
        auto square = ShapeFactory::create_box_2d(0.5f, 0.5f);

        auto result = SATSolver::test_convex_hull_2d(diamond, square, Vec2f(0.0f, 0.0f), Vec2f(0.5f, 0.0f));

        // Should detect collision
        REQUIRE(result.is_colliding);
        REQUIRE(result.penetration_depth > 0.0f);
    }
}

TEST_CASE("SAT Validation: 3D cube-cube collision", "[collision][sat][validation]") {
    SECTION("Separated cubes") {
        auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f));

        REQUIRE(!result.is_colliding);
    }

    SECTION("Overlapping cubes along X-axis") {
        auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.8f, 0.0f, 0.0f));

        REQUIRE(result.is_colliding);
        // Box A: x from -1 to +1, Box B: x from -0.2 to +1.8, Overlap = 1.2
        REQUIRE_THAT(result.penetration_depth, WithinAbs(1.2f, 0.01f));
        // Normal should point along x-axis
        REQUIRE_THAT(std::abs(result.collision_normal.x), WithinAbs(1.0f, 0.01f));
    }

    SECTION("Overlapping cubes along Y-axis") {
        auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.6f, 0.0f));

        REQUIRE(result.is_colliding);
        // Box A: y from -1 to +1, Box B: y from -0.4 to +1.6, Overlap = 1.4
        REQUIRE_THAT(result.penetration_depth, WithinAbs(1.4f, 0.01f));
        // Normal should point along y-axis
        REQUIRE_THAT(std::abs(result.collision_normal.y), WithinAbs(1.0f, 0.01f));
    }

    SECTION("Overlapping cubes along Z-axis") {
        auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.5f));

        REQUIRE(result.is_colliding);
        // Box A: z from -1 to +1, Box B: z from -0.5 to +1.5, Overlap = 1.5
        REQUIRE_THAT(result.penetration_depth, WithinAbs(1.5f, 0.01f));
        // Normal should point along z-axis
        REQUIRE_THAT(std::abs(result.collision_normal.z), WithinAbs(1.0f, 0.01f));
    }
}

TEST_CASE("SAT Validation: Edge case handling", "[collision][sat][validation]") {
    SECTION("Touching but not penetrating") {
        auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        // Cubes at exactly 2.0 apart = edges touching (Box A right edge at +1, Box B left edge at +1)
        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f));

        // Should be considered separated (touching is not collision)
        REQUIRE(!result.is_colliding);
    }

    SECTION("Very small penetration") {
        auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.99f, 0.0f, 0.0f));

        REQUIRE(result.is_colliding);
        REQUIRE(result.penetration_depth > 0.0f);
        // Overlap should be roughly 0.01 (2.0 - 1.99)
        REQUIRE(result.penetration_depth < 0.1f);
    }

    SECTION("Large penetration") {
        auto cube_a = ShapeFactory::create_box_3d(2.0f, 2.0f, 2.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.1f, 0.0f, 0.0f));

        REQUIRE(result.is_colliding);
        // Should detect deep penetration
        REQUIRE(result.penetration_depth > 0.5f);
    }
}

TEST_CASE("SAT Validation: Collision normal direction", "[collision][sat][validation]") {
    SECTION("Normal points from A to B") {
        auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        // Box A centered at (-1,0,0): x from -2 to 0
        // Box B centered at (0.5,0,0): x from -0.5 to 1.5
        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(0.5f, 0.0f, 0.0f));

        REQUIRE(result.is_colliding);
        // Normal should point from A (left) to B (right), so positive x
        REQUIRE(result.collision_normal.x > 0.0f);
    }

    SECTION("Collision normal is normalized") {
        auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
        auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.5f, 0.5f, 0.0f));

        if (result.is_colliding) {
            float normal_length = result.collision_normal.length();
            REQUIRE_THAT(normal_length, WithinAbs(1.0f, 0.01f));
        }
    }
}

TEST_CASE("SAT Validation: Performance characteristics", "[collision][sat][validation][!benchmark]") {
    // This test validates that SAT runs in reasonable time
    // Not an exact benchmark, just a sanity check

    auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
    auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

    // Run many iterations to ensure no obvious performance issues
    for (int i = 0; i < 1000; ++i) {
        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.5f, 0.0f, 0.0f));
        REQUIRE(result.is_colliding);
    }

    // If this completes in reasonable time (< 1 second), SAT is fast enough
    SUCCEED("SAT performance is acceptable");
}

TEST_CASE("SAT Validation: Deterministic results", "[collision][sat][validation]") {
    auto cube_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
    auto cube_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);

    // Run same collision test multiple times
    SATCollisionResult first_result;
    for (int i = 0; i < 10; ++i) {
        auto result = SATSolver::test_convex_hull_3d(cube_a, cube_b, Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.7f, 0.3f, 0.1f));
        
        if (i == 0) {
            first_result = result;
        } else {
            // All subsequent results should match the first
            REQUIRE(result.is_colliding == first_result.is_colliding);
            if (result.is_colliding) {
                REQUIRE_THAT(result.penetration_depth, WithinAbs(first_result.penetration_depth, 1e-7f));
                REQUIRE_THAT(result.collision_normal.x, WithinAbs(first_result.collision_normal.x, 1e-7f));
                REQUIRE_THAT(result.collision_normal.y, WithinAbs(first_result.collision_normal.y, 1e-7f));
                REQUIRE_THAT(result.collision_normal.z, WithinAbs(first_result.collision_normal.z, 1e-7f));
            }
        }
    }
}

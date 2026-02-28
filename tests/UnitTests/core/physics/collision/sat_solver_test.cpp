#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/narrowphase/sat_solver.hpp>
#include <core/physics/collision/shapes/shape_factory.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;

// ============================================================================
// SAT 2D Tests - Basic Collision Detection
// ============================================================================

TEST_CASE("SAT2D - Two separated boxes do not collide", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_a, box_b, Vec2f(0.0f), Vec2f(5.0f)
    );
    
    REQUIRE(!result.is_colliding);
}

TEST_CASE("SAT2D - Two overlapping boxes collide", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_a, box_b, Vec2f(0.0f), Vec2f(0.5f)
    );
    
    REQUIRE(result.is_colliding);
    REQUIRE(result.penetration_depth > 0.0f);
}

TEST_CASE("SAT2D - Touching boxes (edge-to-edge) do not collide", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_a, box_b, Vec2f(0.0f), Vec2f(2.0f)
    );
    
    REQUIRE(!result.is_colliding);
}

TEST_CASE("SAT2D - Rotated box vs axis-aligned box", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto diamond = ShapeFactory::create_regular_polygon_2d(4, 1.5f, 45.0f);
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_a, diamond, Vec2f(0.0f), Vec2f(0.0f)
    );
    
    REQUIRE(result.is_colliding);
}

TEST_CASE("SAT2D - Two separate polygons", "[collision][sat]") {
    auto hex_a = ShapeFactory::create_regular_polygon_2d(6, 1.0f, 0.0f);
    auto hex_b = ShapeFactory::create_regular_polygon_2d(6, 1.0f, 30.0f);
    
    // Move hex_b far away
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        hex_a, hex_b, Vec2f(0.0f), Vec2f(10.0f, 0.0f)
    );
    
    REQUIRE(!result.is_colliding);
}

TEST_CASE("SAT2D - Two overlapping polygons", "[collision][sat]") {
    auto tri_a = ShapeFactory::create_triangle_2d(
        Vec2f(0.0f, 0.0f),
        Vec2f(2.0f, 0.0f),
        Vec2f(1.0f, 2.0f)
    );
    
    auto tri_b = ShapeFactory::create_triangle_2d(
        Vec2f(1.0f, 0.0f),
        Vec2f(3.0f, 0.0f),
        Vec2f(2.0f, 2.0f)
    );
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        tri_a, tri_b, Vec2f(0.0f), Vec2f(0.0f)
    );
    
    REQUIRE(result.is_colliding);
}

// ============================================================================
// SAT 2D Tests - Collision Normal & Penetration
// ============================================================================

TEST_CASE("SAT2D - Collision normal points from A to B", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_a, box_b, Vec2f(-2.0f, 0.0f), Vec2f(-0.5f, 0.0f)
    );
    
    REQUIRE(result.is_colliding);
    // Normal should point roughly in the +X direction (from A to B)
    REQUIRE(result.collision_normal.x > 0.5f);
}

TEST_CASE("SAT2D - Penetration depth is positive", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f);
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_a, box_b, Vec2f(0.0f), Vec2f(0.3f, 0.0f)
    );
    
    REQUIRE(result.is_colliding);
    REQUIRE(result.penetration_depth > 0.0f);
    REQUIRE(result.penetration_depth < 2.0f);  // Should be less than sum of radii
}

// ============================================================================
// SAT 3D Tests - Basic Collision Detection
// ============================================================================

TEST_CASE("SAT3D - Two separated boxes", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
    auto box_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
    
    SATCollisionResult result = SATSolver::test_convex_hull_3d(
        box_a, box_b, Vec3f(0.0f), Vec3f(5.0f)
    );
    
    REQUIRE(!result.is_colliding);
}

TEST_CASE("SAT3D - Two overlapping boxes", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
    auto box_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
    
    SATCollisionResult result = SATSolver::test_convex_hull_3d(
        box_a, box_b, Vec3f(0.0f), Vec3f(0.5f)
    );
    
    REQUIRE(result.is_colliding);
    REQUIRE(result.penetration_depth > 0.0f);
}

TEST_CASE("SAT3D - Box vs tetrahedron overlapping", "[collision][sat]") {
    auto box = ShapeFactory::create_box_3d(2.0f, 2.0f, 2.0f);
    auto tet = ShapeFactory::create_tetrahedron_3d(1.0f);
    
    SATCollisionResult result = SATSolver::test_convex_hull_3d(
        box, tet, Vec3f(0.0f), Vec3f(0.0f)
    );
    
    REQUIRE(result.is_colliding);
}

TEST_CASE("SAT3D - Box vs cylinder", "[collision][sat]") {
    auto box = ShapeFactory::create_box_3d(2.0f, 0.5f, 2.0f);
    auto cyl = ShapeFactory::create_cylinder_3d(0.8f, 2.0f, 8);
    
    SATCollisionResult result = SATSolver::test_convex_hull_3d(
        box, cyl, Vec3f(0.0f), Vec3f(0.0f)
    );
    
    REQUIRE(result.is_colliding);
}

// ============================================================================
// SAT Tests - Determinism
// ============================================================================

TEST_CASE("SAT2D - Deterministic results", "[collision][sat][determinism]") {
    auto box_a = ShapeFactory::create_box_2d(1.5f, 0.7f);
    auto box_b = ShapeFactory::create_box_2d(0.9f, 1.2f);
    
    Vec2f pos_a(0.0f);
    Vec2f pos_b(0.4f, 0.3f);
    
    // Run twice and compare results
    SATCollisionResult result1 = SATSolver::test_convex_hull_2d(box_a, box_b, pos_a, pos_b);
    SATCollisionResult result2 = SATSolver::test_convex_hull_2d(box_a, box_b, pos_a, pos_b);
    
    REQUIRE(result1.is_colliding == result2.is_colliding);
    if (result1.is_colliding) {
        REQUIRE_THAT(result1.penetration_depth, 
                    WithinAbs(result2.penetration_depth, tolerance::STRICT));
        REQUIRE_THAT(result1.collision_normal.x, 
                    WithinAbs(result2.collision_normal.x, tolerance::STRICT));
        REQUIRE_THAT(result1.collision_normal.y, 
                    WithinAbs(result2.collision_normal.y, tolerance::STRICT));
    }
}

TEST_CASE("SAT3D - Deterministic results", "[collision][sat][determinism]") {
    auto box_a = ShapeFactory::create_box_3d(1.5f, 0.7f, 1.1f);
    auto box_b = ShapeFactory::create_box_3d(0.9f, 1.2f, 0.8f);
    
    Vec3f pos_a(0.0f);
    Vec3f pos_b(0.4f, 0.3f, 0.2f);
    
    SATCollisionResult result1 = SATSolver::test_convex_hull_3d(box_a, box_b, pos_a, pos_b);
    SATCollisionResult result2 = SATSolver::test_convex_hull_3d(box_a, box_b, pos_a, pos_b);
    
    REQUIRE(result1.is_colliding == result2.is_colliding);
    if (result1.is_colliding) {
        REQUIRE_THAT(result1.penetration_depth,
                    WithinAbs(result2.penetration_depth, tolerance::STRICT));
    }
}

// ============================================================================
// SAT Tests - Edge Cases
// ============================================================================

TEST_CASE("SAT2D - Identical boxes at same position", "[collision][sat]") {
    auto box = ShapeFactory::create_box_2d(1.0f, 1.0f);
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box, box, Vec2f(0.0f), Vec2f(0.0f)
    );
    
    REQUIRE(result.is_colliding);
}

TEST_CASE("SAT3D - Deeply overlapping boxes", "[collision][sat]") {
    auto box_a = ShapeFactory::create_box_3d(2.0f, 2.0f, 2.0f);
    auto box_b = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f);
    
    // box_b is completely inside box_a
    SATCollisionResult result = SATSolver::test_convex_hull_3d(
        box_a, box_b, Vec3f(0.0f), Vec3f(0.0f)
    );
    
    REQUIRE(result.is_colliding);
}

// ============================================================================
// SAT Tests - Golden Scenarios
// ============================================================================

TEST_CASE("SAT2D - Golden scenario: axis alignment", "[collision][sat][golden]") {
    // Two axis-aligned boxes, slightly overlapping in X
    auto box_a = ShapeFactory::create_box_2d(1.0f, 2.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 2.0f, Vec2f(0.0f));
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_a, box_b, Vec2f(-1.0f, 0.0f), Vec2f(0.5f, 0.0f)
    );
    
    REQUIRE(result.is_colliding);
    // Penetration should be 0.5 (gap = 0 at x=0, penetration = min(1, 1) - abs(offset) = 2 - 1.5 = 0.5)
    REQUIRE_THAT(result.penetration_depth, WithinAbs(0.5f, tolerance::POSITION));
}

TEST_CASE("SAT2D - Golden scenario: 45-degree rotation", "[collision][sat][golden]") {
    auto box_axis = ShapeFactory::create_box_2d(1.0f, 1.0f);
    auto box_rot = ShapeFactory::create_regular_polygon_2d(4, 1.0f, 45.0f);
    
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_axis, box_rot, Vec2f(0.0f), Vec2f(0.0f)
    );
    
    REQUIRE(result.is_colliding);
    REQUIRE(result.penetration_depth > 0.0f);
}

TEST_CASE("SAT2D - Golden scenario: edge touching", "[collision][sat][golden]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    // Boxes touch at x = 0, should NOT collide (merely touching)
    SATCollisionResult result = SATSolver::test_convex_hull_2d(
        box_a, box_b, Vec2f(-2.0f, 0.0f), Vec2f(2.0f, 0.0f)
    );
    
    REQUIRE(!result.is_colliding);
}

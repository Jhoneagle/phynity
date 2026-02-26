#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/aabb_narrowphase.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Basic AABB-AABB Collision Detection
// ============================================================================

TEST_CASE("AABBNarrowphase - No collision when separated", "[collision][aabb][narrowphase]") {
    AABB aabb_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB aabb_b(Vec3f(3.0f, 0.0f, 0.0f), Vec3f(4.0f, 1.0f, 1.0f));
    
    Vec3f center_a(0.5f, 0.5f, 0.5f);
    Vec3f center_b(3.5f, 0.5f, 0.5f);
    Vec3f velocity_a(0.0f, 0.0f, 0.0f);
    Vec3f velocity_b(0.0f, 0.0f, 0.0f);
    
    ContactManifold manifold = AABBNarrowphase::detect(
        aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
    );
    
    REQUIRE_FALSE(manifold.is_valid());
}

TEST_CASE("AABBNarrowphase - Collision with overlap on x-axis", "[collision][aabb][narrowphase]") {
    AABB aabb_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB aabb_b(Vec3f(1.5f, 0.0f, 0.0f), Vec3f(3.5f, 2.0f, 2.0f));
    
    Vec3f center_a(1.0f, 1.0f, 1.0f);
    Vec3f center_b(2.5f, 1.0f, 1.0f);
    Vec3f velocity_a(1.0f, 0.0f, 0.0f);   // Moving toward B (right)
    Vec3f velocity_b(0.0f, 0.0f, 0.0f);
    
    ContactManifold manifold = AABBNarrowphase::detect(
        aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
    );
    
    REQUIRE(manifold.is_valid());
    REQUIRE(manifold.object_a_id == 0);
    REQUIRE(manifold.object_b_id == 1);
    
    // Normal should point from A to B (positive x direction)
    REQUIRE_THAT(manifold.contact.normal.x, WithinAbs(1.0f, tolerance::STRICT));
    REQUIRE_THAT(manifold.contact.normal.y, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(manifold.contact.normal.z, WithinAbs(0.0f, tolerance::STRICT));
    
    // Penetration should be 0.5 (overlap on x-axis)
    REQUIRE_THAT(manifold.contact.penetration, WithinAbs(0.5f, tolerance::STRICT));
}

TEST_CASE("AABBNarrowphase - Collision with overlap on y-axis", "[collision][aabb][narrowphase]") {
    AABB aabb_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB aabb_b(Vec3f(0.0f, 1.5f, 0.0f), Vec3f(2.0f, 3.5f, 2.0f));
    
    Vec3f center_a(1.0f, 1.0f, 1.0f);
    Vec3f center_b(1.0f, 2.5f, 1.0f);
    Vec3f velocity_a(0.0f, 1.0f, 0.0f);   // Moving toward B (up)
    Vec3f velocity_b(0.0f, 0.0f, 0.0f);
    
    ContactManifold manifold = AABBNarrowphase::detect(
        aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
    );
    
    REQUIRE(manifold.is_valid());
    
    // Normal should point from A to B (positive y direction)
    REQUIRE_THAT(manifold.contact.normal.x, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(manifold.contact.normal.y, WithinAbs(1.0f, tolerance::STRICT));
    REQUIRE_THAT(manifold.contact.normal.z, WithinAbs(0.0f, tolerance::STRICT));
    
    // Penetration should be 0.5 (overlap on y-axis)
    REQUIRE_THAT(manifold.contact.penetration, WithinAbs(0.5f, tolerance::STRICT));
}

TEST_CASE("AABBNarrowphase - Collision with overlap on z-axis", "[collision][aabb][narrowphase]") {
    AABB aabb_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB aabb_b(Vec3f(0.0f, 0.0f, 1.5f), Vec3f(2.0f, 2.0f, 3.5f));
    
    Vec3f center_a(1.0f, 1.0f, 1.0f);
    Vec3f center_b(1.0f, 1.0f, 2.5f);
    Vec3f velocity_a(0.0f, 0.0f, 1.0f);   // Moving toward B (forward)
    Vec3f velocity_b(0.0f, 0.0f, 0.0f);
    
    ContactManifold manifold = AABBNarrowphase::detect(
        aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
    );
    
    REQUIRE(manifold.is_valid());
    
    // Normal should point from A to B (positive z direction)
    REQUIRE_THAT(manifold.contact.normal.x, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(manifold.contact.normal.y, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(manifold.contact.normal.z, WithinAbs(1.0f, tolerance::STRICT));
    
    // Penetration should be 0.5 (overlap on z-axis)
    REQUIRE_THAT(manifold.contact.penetration, WithinAbs(0.5f, tolerance::STRICT));
}

TEST_CASE("AABBNarrowphase - Least penetration axis selected", "[collision][aabb][narrowphase]") {
    // Overlap more on y and z, less on x
    AABB aabb_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB aabb_b(Vec3f(1.8f, 0.5f, 0.5f), Vec3f(3.8f, 2.5f, 2.5f));
    
    Vec3f center_a(1.0f, 1.0f, 1.0f);
    Vec3f center_b(2.8f, 1.5f, 1.5f);
    Vec3f velocity_a(1.0f, 0.0f, 0.0f);   // Moving toward B (right)
    Vec3f velocity_b(0.0f, 0.0f, 0.0f);
    
    ContactManifold manifold = AABBNarrowphase::detect(
        aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
    );
    
    REQUIRE(manifold.is_valid());
    
    // Should select x-axis (least overlap = 0.2)
    // y overlap = min(2.0 - 0.5, 2.5 - 0.0) = 1.5
    // z overlap = min(2.0 - 0.5, 2.5 - 0.0) = 1.5
    // x overlap = min(2.0 - 1.8, 3.8 - 0.0) = 0.2
    REQUIRE_THAT(manifold.contact.normal.x, WithinAbs(1.0f, tolerance::STRICT));
    REQUIRE_THAT(manifold.contact.normal.y, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(manifold.contact.normal.z, WithinAbs(0.0f, tolerance::STRICT));
    
    REQUIRE_THAT(manifold.contact.penetration, WithinAbs(0.2f, tolerance::STRICT));
}

TEST_CASE("AABBNarrowphase - Objects separating (rejected)", "[collision][aabb][narrowphase]") {
    AABB aabb_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB aabb_b(Vec3f(1.5f, 0.0f, 0.0f), Vec3f(3.5f, 2.0f, 2.0f));
    
    Vec3f center_a(1.0f, 1.0f, 1.0f);
    Vec3f center_b(2.5f, 1.0f, 1.0f);
    Vec3f velocity_a(-1.0f, 0.0f, 0.0f);  // Moving away from B
    Vec3f velocity_b(-1.0f, 0.0f, 0.0f);  // Also moving away
    
    ContactManifold manifold = AABBNarrowphase::detect(
        aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
    );
    
    // Should be rejected because objects are separating
    REQUIRE_FALSE(manifold.is_valid());
}

TEST_CASE("AABBNarrowphase - Relative velocity calculation", "[collision][aabb][narrowphase]") {
    AABB aabb_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB aabb_b(Vec3f(1.5f, 0.0f, 0.0f), Vec3f(3.5f, 2.0f, 2.0f));
    
    Vec3f center_a(1.0f, 1.0f, 1.0f);
    Vec3f center_b(2.5f, 1.0f, 1.0f);
    Vec3f velocity_a(2.0f, 0.0f, 0.0f);
    Vec3f velocity_b(-1.0f, 0.0f, 0.0f);
    
    ContactManifold manifold = AABBNarrowphase::detect(
        aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
    );
    
    REQUIRE(manifold.is_valid());
    
    // Relative velocity along normal (x-axis)
    // rel_vel = velocity_b - velocity_a = -1 - 2 = -3
    REQUIRE_THAT(manifold.contact.relative_velocity_along_normal, WithinAbs(-3.0f, tolerance::STRICT));
}

TEST_CASE("AABBNarrowphase - Complete containment", "[collision][aabb][narrowphase]") {
    AABB aabb_a(Vec3f(-2.0f, -2.0f, -2.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB aabb_b(Vec3f(-0.5f, -0.5f, -0.5f), Vec3f(0.5f, 0.5f, 0.5f));
    
    // Offset centers slightly to give meaningful collision normal
    Vec3f center_a(-0.1f, 0.0f, 0.0f);
    Vec3f center_b(0.1f, 0.0f, 0.0f);
    Vec3f velocity_a(1.0f, 0.0f, 0.0f);   // A moving toward B
    Vec3f velocity_b(0.0f, 0.0f, 0.0f);
    
    ContactManifold manifold = AABBNarrowphase::detect(
        aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
    );
    
    REQUIRE(manifold.is_valid());
    
    // Penetration should be minimum of all overlaps
    // x: min(2.0 - (-0.5), 0.5 - (-2.0)) = min(2.5, 2.5) = 2.5
    // y: min(2.0 - (-0.5), 0.5 - (-2.0)) = min(2.5, 2.5) = 2.5
    // z: min(2.0 - (-0.5), 0.5 - (-2.0)) = min(2.5, 2.5) = 2.5
    // All equal, so x-axis is chosen with penetration 2.5
    REQUIRE_THAT(manifold.contact.penetration, WithinAbs(2.5f, tolerance::STRICT));
}

TEST_CASE("AABBNarrowphase - Normal direction from A to B", "[collision][aabb][narrowphase]") {
    SECTION("B is to the right of A") {
        AABB aabb_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 2.0f, 2.0f));
        AABB aabb_b(Vec3f(1.5f, 0.0f, 0.0f), Vec3f(3.5f, 2.0f, 2.0f));
        
        Vec3f center_a(1.0f, 1.0f, 1.0f);
        Vec3f center_b(2.5f, 1.0f, 1.0f);
        Vec3f velocity_a(1.0f, 0.0f, 0.0f);   // Moving toward B (right)
        Vec3f velocity_b(0.0f, 0.0f, 0.0f);
        
        ContactManifold manifold = AABBNarrowphase::detect(
            aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
        );
        
        REQUIRE(manifold.is_valid());
        REQUIRE_THAT(manifold.contact.normal.x, WithinAbs(1.0f, tolerance::STRICT));
    }
    
    SECTION("B is to the left of A") {
        AABB aabb_a(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(4.0f, 2.0f, 2.0f));
        AABB aabb_b(Vec3f(0.5f, 0.0f, 0.0f), Vec3f(2.5f, 2.0f, 2.0f));
        
        Vec3f center_a(3.0f, 1.0f, 1.0f);
        Vec3f center_b(1.5f, 1.0f, 1.0f);
        Vec3f velocity_a(-1.0f, 0.0f, 0.0f);  // Moving toward B (left)
        Vec3f velocity_b(0.0f, 0.0f, 0.0f);
        
        ContactManifold manifold = AABBNarrowphase::detect(
            aabb_a, aabb_b, center_a, center_b, velocity_a, velocity_b, 0, 1
        );
        
        REQUIRE(manifold.is_valid());
        REQUIRE_THAT(manifold.contact.normal.x, WithinAbs(-1.0f, tolerance::STRICT));
    }
}

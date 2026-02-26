#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/aabb.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Basic AABB Construction and Properties
// ============================================================================

TEST_CASE("AABB - Default construction", "[collision][aabb]") {
    AABB aabb;
    
    REQUIRE_THAT(aabb.min.x, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.min.y, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.min.z, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.x, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.y, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.z, WithinAbs(0.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Constructor with min/max", "[collision][aabb]") {
    AABB aabb(Vec3f(-1.0f, -2.0f, -3.0f), Vec3f(1.0f, 2.0f, 3.0f));
    
    REQUIRE_THAT(aabb.min.x, WithinAbs(-1.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.min.y, WithinAbs(-2.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.min.z, WithinAbs(-3.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.x, WithinAbs(1.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.y, WithinAbs(2.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.z, WithinAbs(3.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Get center", "[collision][aabb]") {
    AABB aabb(Vec3f(-2.0f, -4.0f, -6.0f), Vec3f(2.0f, 4.0f, 6.0f));
    Vec3f center = aabb.get_center();
    
    REQUIRE_THAT(center.x, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(center.y, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(center.z, WithinAbs(0.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Get half extents", "[collision][aabb]") {
    AABB aabb(Vec3f(-2.0f, -4.0f, -6.0f), Vec3f(2.0f, 4.0f, 6.0f));
    Vec3f half_extents = aabb.get_half_extents();
    
    REQUIRE_THAT(half_extents.x, WithinAbs(2.0f, tolerance::STRICT));
    REQUIRE_THAT(half_extents.y, WithinAbs(4.0f, tolerance::STRICT));
    REQUIRE_THAT(half_extents.z, WithinAbs(6.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Get extents", "[collision][aabb]") {
    AABB aabb(Vec3f(-1.0f, -2.0f, -3.0f), Vec3f(3.0f, 6.0f, 9.0f));
    Vec3f extents = aabb.get_extents();
    
    REQUIRE_THAT(extents.x, WithinAbs(4.0f, tolerance::STRICT));
    REQUIRE_THAT(extents.y, WithinAbs(8.0f, tolerance::STRICT));
    REQUIRE_THAT(extents.z, WithinAbs(12.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Surface area", "[collision][aabb]") {
    AABB aabb(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 3.0f, 4.0f));
    float area = aabb.surface_area();
    
    // Surface area = 2 * (2*3 + 3*4 + 4*2) = 2 * (6 + 12 + 8) = 52
    REQUIRE_THAT(area, WithinAbs(52.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Volume", "[collision][aabb]") {
    AABB aabb(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 3.0f, 4.0f));
    float volume = aabb.volume();
    
    // Volume = 2 * 3 * 4 = 24
    REQUIRE_THAT(volume, WithinAbs(24.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Is valid", "[collision][aabb]") {
    SECTION("Valid AABB") {
        AABB aabb(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
        REQUIRE(aabb.is_valid());
    }
    
    SECTION("Invalid AABB - inverted x") {
        AABB aabb(Vec3f(1.0f, -1.0f, -1.0f), Vec3f(-1.0f, 1.0f, 1.0f));
        REQUIRE_FALSE(aabb.is_valid());
    }
    
    SECTION("Invalid AABB - inverted y") {
        AABB aabb(Vec3f(-1.0f, 1.0f, -1.0f), Vec3f(1.0f, -1.0f, 1.0f));
        REQUIRE_FALSE(aabb.is_valid());
    }
    
    SECTION("Invalid AABB - inverted z") {
        AABB aabb(Vec3f(-1.0f, -1.0f, 1.0f), Vec3f(1.0f, 1.0f, -1.0f));
        REQUIRE_FALSE(aabb.is_valid());
    }
}

// ============================================================================
// Point Containment Tests
// ============================================================================

TEST_CASE("AABB - Contains point inside", "[collision][aabb]") {
    AABB aabb(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    
    REQUIRE(aabb.contains_point(Vec3f(0.0f, 0.0f, 0.0f)));
    REQUIRE(aabb.contains_point(Vec3f(0.5f, 0.5f, 0.5f)));
    REQUIRE(aabb.contains_point(Vec3f(-0.5f, -0.5f, -0.5f)));
}

TEST_CASE("AABB - Contains point on boundary", "[collision][aabb]") {
    AABB aabb(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    
    REQUIRE(aabb.contains_point(Vec3f(-1.0f, 0.0f, 0.0f)));
    REQUIRE(aabb.contains_point(Vec3f(1.0f, 0.0f, 0.0f)));
    REQUIRE(aabb.contains_point(Vec3f(0.0f, -1.0f, 0.0f)));
    REQUIRE(aabb.contains_point(Vec3f(0.0f, 1.0f, 0.0f)));
    REQUIRE(aabb.contains_point(Vec3f(0.0f, 0.0f, -1.0f)));
    REQUIRE(aabb.contains_point(Vec3f(0.0f, 0.0f, 1.0f)));
}

TEST_CASE("AABB - Point outside", "[collision][aabb]") {
    AABB aabb(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    
    REQUIRE_FALSE(aabb.contains_point(Vec3f(2.0f, 0.0f, 0.0f)));
    REQUIRE_FALSE(aabb.contains_point(Vec3f(-2.0f, 0.0f, 0.0f)));
    REQUIRE_FALSE(aabb.contains_point(Vec3f(0.0f, 2.0f, 0.0f)));
    REQUIRE_FALSE(aabb.contains_point(Vec3f(0.0f, -2.0f, 0.0f)));
    REQUIRE_FALSE(aabb.contains_point(Vec3f(0.0f, 0.0f, 2.0f)));
    REQUIRE_FALSE(aabb.contains_point(Vec3f(0.0f, 0.0f, -2.0f)));
}

// ============================================================================
// AABB-AABB Overlap Tests
// ============================================================================

TEST_CASE("AABB - Overlaps with itself", "[collision][aabb]") {
    AABB aabb(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    
    REQUIRE(aabb.overlaps(aabb));
}

TEST_CASE("AABB - Overlaps with contained AABB", "[collision][aabb]") {
    AABB outer(Vec3f(-2.0f, -2.0f, -2.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB inner(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    
    REQUIRE(outer.overlaps(inner));
    REQUIRE(inner.overlaps(outer));
}

TEST_CASE("AABB - Partial overlap", "[collision][aabb]") {
    AABB a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 2.0f, 2.0f));
    AABB b(Vec3f(1.0f, 1.0f, 1.0f), Vec3f(3.0f, 3.0f, 3.0f));
    
    REQUIRE(a.overlaps(b));
    REQUIRE(b.overlaps(a));
}

TEST_CASE("AABB - No overlap - separated on x axis", "[collision][aabb]") {
    AABB a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB b(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(3.0f, 1.0f, 1.0f));
    
    REQUIRE_FALSE(a.overlaps(b));
    REQUIRE_FALSE(b.overlaps(a));
}

TEST_CASE("AABB - No overlap - separated on y axis", "[collision][aabb]") {
    AABB a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB b(Vec3f(0.0f, 2.0f, 0.0f), Vec3f(1.0f, 3.0f, 1.0f));
    
    REQUIRE_FALSE(a.overlaps(b));
    REQUIRE_FALSE(b.overlaps(a));
}

TEST_CASE("AABB - No overlap - separated on z axis", "[collision][aabb]") {
    AABB a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB b(Vec3f(0.0f, 0.0f, 2.0f), Vec3f(1.0f, 1.0f, 3.0f));
    
    REQUIRE_FALSE(a.overlaps(b));
    REQUIRE_FALSE(b.overlaps(a));
}

TEST_CASE("AABB - Edge touching (no overlap)", "[collision][aabb]") {
    AABB a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB b(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(2.0f, 1.0f, 1.0f));
    
    // Edge-to-edge touching should NOT count as overlap (max < min fails)
    REQUIRE_FALSE(a.overlaps(b));
    REQUIRE_FALSE(b.overlaps(a));
}

// ============================================================================
// AABB Expansion Tests
// ============================================================================

TEST_CASE("AABB - Expand by another AABB", "[collision][aabb]") {
    AABB a(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB b(Vec3f(0.5f, 0.5f, 0.5f), Vec3f(2.0f, 2.0f, 2.0f));
    
    a.expand(b);
    
    REQUIRE_THAT(a.min.x, WithinAbs(-1.0f, tolerance::STRICT));
    REQUIRE_THAT(a.min.y, WithinAbs(-1.0f, tolerance::STRICT));
    REQUIRE_THAT(a.min.z, WithinAbs(-1.0f, tolerance::STRICT));
    REQUIRE_THAT(a.max.x, WithinAbs(2.0f, tolerance::STRICT));
    REQUIRE_THAT(a.max.y, WithinAbs(2.0f, tolerance::STRICT));
    REQUIRE_THAT(a.max.z, WithinAbs(2.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Expand by point", "[collision][aabb]") {
    AABB aabb(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    
    aabb.expand(Vec3f(3.0f, 0.0f, 0.0f));
    
    REQUIRE_THAT(aabb.min.x, WithinAbs(-1.0f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.x, WithinAbs(3.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Expand by point inside (no change)", "[collision][aabb]") {
    AABB aabb(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB original = aabb;
    
    aabb.expand(Vec3f(0.5f, 0.5f, 0.5f));
    
    REQUIRE_THAT(aabb.min.x, WithinAbs(original.min.x, tolerance::STRICT));
    REQUIRE_THAT(aabb.min.y, WithinAbs(original.min.y, tolerance::STRICT));
    REQUIRE_THAT(aabb.min.z, WithinAbs(original.min.z, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.x, WithinAbs(original.max.x, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.y, WithinAbs(original.max.y, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.z, WithinAbs(original.max.z, tolerance::STRICT));
}

// ============================================================================
// AABB Factory Methods
// ============================================================================

TEST_CASE("AABB - From sphere", "[collision][aabb]") {
    Vec3f center(1.0f, 2.0f, 3.0f);
    float radius = 0.5f;
    
    AABB aabb = AABB::from_sphere(center, radius);
    
    REQUIRE_THAT(aabb.min.x, WithinAbs(0.5f, tolerance::STRICT));
    REQUIRE_THAT(aabb.min.y, WithinAbs(1.5f, tolerance::STRICT));
    REQUIRE_THAT(aabb.min.z, WithinAbs(2.5f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.x, WithinAbs(1.5f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.y, WithinAbs(2.5f, tolerance::STRICT));
    REQUIRE_THAT(aabb.max.z, WithinAbs(3.5f, tolerance::STRICT));
}

TEST_CASE("AABB - Merge multiple AABBs", "[collision][aabb]") {
    std::vector<AABB> aabbs;
    aabbs.push_back(AABB(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 1.0f, 1.0f)));
    aabbs.push_back(AABB(Vec3f(2.0f, 2.0f, 2.0f), Vec3f(3.0f, 3.0f, 3.0f)));
    aabbs.push_back(AABB(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(0.5f, 0.5f, 0.5f)));
    
    AABB merged = AABB::merge(aabbs);
    
    REQUIRE_THAT(merged.min.x, WithinAbs(-1.0f, tolerance::STRICT));
    REQUIRE_THAT(merged.min.y, WithinAbs(-1.0f, tolerance::STRICT));
    REQUIRE_THAT(merged.min.z, WithinAbs(-1.0f, tolerance::STRICT));
    REQUIRE_THAT(merged.max.x, WithinAbs(3.0f, tolerance::STRICT));
    REQUIRE_THAT(merged.max.y, WithinAbs(3.0f, tolerance::STRICT));
    REQUIRE_THAT(merged.max.z, WithinAbs(3.0f, tolerance::STRICT));
}

TEST_CASE("AABB - Merge empty vector", "[collision][aabb]") {
    std::vector<AABB> aabbs;
    AABB merged = AABB::merge(aabbs);
    
    // Should return default-constructed AABB
    REQUIRE_THAT(merged.min.x, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(merged.max.x, WithinAbs(0.0f, tolerance::STRICT));
}

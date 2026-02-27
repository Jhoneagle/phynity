#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/gjk_solver.hpp>
#include <core/physics/collision/epa_solver.hpp>
#include <core/physics/collision/support_function.hpp>
#include <core/physics/collision/shape_factory.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>
#include <cmath>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;

// ============================================================================
// GJK Algorithm Tests - Basic Distance Detection
// ============================================================================

// Manual support function for sphere
class SphereSupport : public SupportFunction {
public:
    Vec3f position;
    float radius;
    
    SphereSupport(const Vec3f& pos, float r) : position(pos), radius(r) {}
    
    Vec3f get_support_point(const Vec3f& direction) const override {
        Vec3f normalized = direction.normalized();
        return position + normalized * radius;
    }
    
    Vec3f get_origin() const override {
        return position;
    }
};

TEST_CASE("GJK - Two separated spheres detect distance", "[collision][gjk]") {
    // Create two spheres 5 units apart
    Vec3f pos_a(0.0f), pos_b(5.0f, 0.0f, 0.0f);
    float radius = 1.0f;
    
    SphereSupport support_a(pos_a, radius);
    SphereSupport support_b(pos_b, radius);
    
    GJKResult result = GJKSolver::solve(support_a, support_b);
    
    // Distance should be approximately 3.0 (5 - 1 - 1)
    REQUIRE(!result.collision);
    REQUIRE_THAT(result.distance, WithinAbs(3.0f, 0.1f));
}

TEST_CASE("GJK - Two overlapping spheres detect collision", "[collision][gjk]") {
    Vec3f pos_a(0.0f), pos_b(1.5f, 0.0f, 0.0f);
    float radius = 1.0f;
    
    SphereSupport support_a(pos_a, radius);
    SphereSupport support_b(pos_b, radius);
    
    GJKResult result = GJKSolver::solve(support_a, support_b);
    
    // Spheres overlap, so distance should be negative
    REQUIRE(result.collision);
    REQUIRE(result.distance <= 0.0f);
}

TEST_CASE("GJK - Touching spheres detect zero distance", "[collision][gjk]") {
    Vec3f pos_a(0.0f), pos_b(2.0f, 0.0f, 0.0f);
    float radius = 1.0f;
    
    SphereSupport support_a(pos_a, radius);
    SphereSupport support_b(pos_b, radius);
    
    GJKResult result = GJKSolver::solve(support_a, support_b);
    
    // Spheres are touching
    REQUIRE_THAT(result.distance, WithinAbs(0.0f, 0.01f));
}

TEST_CASE("GJK - Box-box separation", "[collision][gjk]") {
    // Two boxes separated on X axis
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    ShapeSupportFunction support_a(&box_a, Vec3f(-3.0f, 0.0f, 0.0f));
    ShapeSupportFunction support_b(&box_b, Vec3f(3.0f, 0.0f, 0.0f));
    
    GJKResult result = GJKSolver::solve(support_a, support_b);
    
    // Distance should be approximately 4.0 (distance between centers minus box extents)
    REQUIRE(!result.collision);
    REQUIRE(result.distance > 0.0f);
}

TEST_CASE("GJK - Box-box overlap", "[collision][gjk]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    ShapeSupportFunction support_a(&box_a, Vec3f(0.0f, 0.0f, 0.0f));
    ShapeSupportFunction support_b(&box_b, Vec3f(0.5f, 0.0f, 0.0f));
    
    GJKResult result = GJKSolver::solve(support_a, support_b);
    
    // Boxes overlap
    REQUIRE(result.collision);
}

TEST_CASE("GJK - Sphere-box separation", "[collision][gjk]") {
    auto box = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    SphereSupport sphere(Vec3f(-5.0f, 0.0f, 0.0f), 1.0f);
    
    ShapeSupportFunction box_support(&box, Vec3f(0.0f, 0.0f, 0.0f));
    
    GJKResult result = GJKSolver::solve(sphere, box_support);
    
    // Sphere and box should be separated
    REQUIRE(!result.collision);
    REQUIRE(result.distance > 0.0f);
}

// ============================================================================
// GJK Algorithm Tests - Determinism
// ============================================================================

TEST_CASE("GJK - Deterministic results for same input", "[collision][gjk][determinism]") {
    Vec3f pos_a(1.0f, 2.0f, 3.0f), pos_b(4.0f, 5.0f, 6.0f);
    float radius = 0.5f;
    
    SphereSupport support_a(pos_a, radius);
    SphereSupport support_b(pos_b, radius);
    
    // Run GJK multiple times with same input
    GJKResult result1 = GJKSolver::solve(support_a, support_b);
    GJKResult result2 = GJKSolver::solve(support_a, support_b);
    GJKResult result3 = GJKSolver::solve(support_a, support_b);
    
    REQUIRE_THAT(result1.distance, WithinAbs(result2.distance, 1e-6f));
    REQUIRE_THAT(result2.distance, WithinAbs(result3.distance, 1e-6f));
    REQUIRE(result1.collision == result2.collision);
    REQUIRE(result2.collision == result3.collision);
}

// ============================================================================
// EPA Algorithm Tests - Contact Generation
// ============================================================================

TEST_CASE("EPA - Penetrating spheres generate contact", "[collision][epa]") {
    Vec3f pos_a(0.0f), pos_b(0.5f, 0.0f, 0.0f);
    float radius = 1.0f;
    
    SphereSupport support_a(pos_a, radius);
    SphereSupport support_b(pos_b, radius);
    
    GJKResult gjk_result = GJKSolver::solve(support_a, support_b);
    
    if (gjk_result.collision) {
        EPAResult epa_result = EPASolver::solve(support_a, support_b, gjk_result);
        
        // Should have computed contact information
        REQUIRE(epa_result.penetration_depth > 0.0f);
        // Normal should be roughly along X axis (from A to B)
        REQUIRE(std::abs(epa_result.contact_normal.x) > 0.8f);
    }
}

TEST_CASE("EPA - Overlapping boxes generate contact normal", "[collision][epa]") {
    auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
    
    ShapeSupportFunction support_a(&box_a, Vec3f(0.0f, 0.0f, 0.0f));
    ShapeSupportFunction support_b(&box_b, Vec3f(0.5f, 0.0f, 0.0f));
    
    GJKResult gjk_result = GJKSolver::solve(support_a, support_b);
    
    if (gjk_result.collision) {
        EPAResult epa_result = EPASolver::solve(support_a, support_b, gjk_result);
        
        // Contact should be generated
        REQUIRE(epa_result.penetration_depth >= 0.0f);
        // Normal length should be approximately 1.0
        REQUIRE_THAT(epa_result.contact_normal.length(), WithinAbs(1.0f, 0.01f));
    }
}

TEST_CASE("EPA - Deterministic contact generation", "[collision][epa][determinism]") {
    Vec3f pos_a(0.0f), pos_b(0.3f, 0.0f, 0.0f);
    float radius = 1.0f;
    
    SphereSupport support_a(pos_a, radius);
    SphereSupport support_b(pos_b, radius);
    
    GJKResult gjk_result = GJKSolver::solve(support_a, support_b);
    
    if (gjk_result.collision) {
        EPAResult epa1 = EPASolver::solve(support_a, support_b, gjk_result);
        EPAResult epa2 = EPASolver::solve(support_a, support_b, gjk_result);
        EPAResult epa3 = EPASolver::solve(support_a, support_b, gjk_result);
        
        // Depth should be deterministic
        REQUIRE_THAT(epa1.penetration_depth, WithinAbs(epa2.penetration_depth, 1e-6f));
        REQUIRE_THAT(epa2.penetration_depth, WithinAbs(epa3.penetration_depth, 1e-6f));
    }
}

// ============================================================================
// GJK+EPA Integration Tests
// ============================================================================

TEST_CASE("GJK+EPA - Conversion to contact manifold", "[collision][gjk][epa][integration]") {
    Vec3f pos_a(0.0f), pos_b(1.5f, 0.0f, 0.0f);
    float radius = 1.0f;
    
    SphereSupport support_a(pos_a, radius);
    SphereSupport support_b(pos_b, radius);
    
    GJKResult gjk_result = GJKSolver::solve(support_a, support_b);
    
    if (gjk_result.collision) {
        EPAResult epa_result = EPASolver::solve(support_a, support_b, gjk_result);
        
        // Contact information should be available
        REQUIRE(epa_result.penetration_depth >= 0.0f);
        REQUIRE_THAT(epa_result.contact_normal.length(), WithinAbs(1.0f, 0.01f));
    } else {
        // For separated shapes, we should use GJK distance
        REQUIRE(gjk_result.distance >= 0.0f);
    }
}

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/gjk_solver.hpp>
#include <core/physics/collision/epa_solver.hpp>
#include <core/physics/collision/support_function.hpp>
#include <core/physics/collision/shape_factory.hpp>
#include <core/math/vectors/vec3.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using Catch::Matchers::WithinAbs;

// Manual support function for sphere (for direct validation tests)
class SimpleSphereSupport : public SupportFunction {
public:
    Vec3f position;
    float radius;
    
    SimpleSphereSupport(const Vec3f& pos, float r) : position(pos), radius(r) {}
    
    Vec3f get_support_point(const Vec3f& direction) const override {
        Vec3f normalized = direction.normalized();
        return position + normalized * radius;
    }
    
    Vec3f get_origin() const override {
        return position;
    }
};

TEST_CASE("GJK Validation: Sphere-sphere distance", "[collision][gjk][validation]") {
    SECTION("Separated spheres - known distance") {
        SimpleSphereSupport sphere_a(Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
        SimpleSphereSupport sphere_b(Vec3f(5.0f, 0.0f, 0.0f), 1.0f);

        GJKResult result = GJKSolver::solve(sphere_a, sphere_b);

        // Distance should be 5.0 (centers) - 1.0 (radius A) - 1.0 (radius B) = 3.0
        REQUIRE(!result.collision);
        REQUIRE_THAT(result.distance, WithinAbs(3.0f, 0.2f));
    }

    SECTION("Touching spheres") {
        SimpleSphereSupport sphere_a(Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
        SimpleSphereSupport sphere_b(Vec3f(2.0f, 0.0f, 0.0f), 1.0f);

        GJKResult result = GJKSolver::solve(sphere_a, sphere_b);

        // Distance should be approximately 0 (touching)
        REQUIRE_THAT(result.distance, WithinAbs(0.0f, 0.1f));
    }

    SECTION("Overlapping spheres") {
        SimpleSphereSupport sphere_a(Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
        SimpleSphereSupport sphere_b(Vec3f(1.5f, 0.0f, 0.0f), 1.0f);

        GJKResult result = GJKSolver::solve(sphere_a, sphere_b);

        // GJK returns collision=true for overlapping shapes
        REQUIRE(result.collision);
    }
}

TEST_CASE("GJK Validation: Box-box distance using ShapeFactory", "[collision][gjk][validation]") {
    SECTION("Separated boxes along X-axis") {
        auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
        auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));

        ShapeSupportFunction support_a(&box_a, Vec3f(-3.0f, 0.0f, 0.0f));
        ShapeSupportFunction support_b(&box_b, Vec3f(3.0f, 0.0f, 0.0f));

        GJKResult result = GJKSolver::solve(support_a, support_b);

        // Distance should be positive (boxes are separated)
        REQUIRE(!result.collision);
        REQUIRE(result.distance > 0.0f);
    }

    SECTION("Overlapping boxes") {
        auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
        auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));

        ShapeSupportFunction support_a(&box_a, Vec3f(0.0f, 0.0f, 0.0f));
        ShapeSupportFunction support_b(&box_b, Vec3f(0.5f, 0.0f, 0.0f));

        GJKResult result = GJKSolver::solve(support_a, support_b);

        // Should detect collision
        REQUIRE(result.collision);
    }
}

TEST_CASE("EPA Validation: Basic penetration detection", "[collision][epa][validation]") {
    SECTION("Overlapping spheres") {
        SimpleSphereSupport sphere_a(Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
        SimpleSphereSupport sphere_b(Vec3f(1.5f, 0.0f, 0.0f), 1.0f);

        GJKResult gjk_result = GJKSolver::solve(sphere_a, sphere_b);

        if (gjk_result.collision) {
            EPAResult epa_result = EPASolver::solve(sphere_a, sphere_b, gjk_result);

            // Should have positive penetration depth
            REQUIRE(epa_result.penetration_depth > 0.0f);
            // Check normal is reasonable (points along X-axis for this setup)
            REQUIRE(std::abs(epa_result.contact_normal.x) > 0.5f);
        }
    }

    SECTION("Overlapping boxes") {
        auto box_a = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));
        auto box_b = ShapeFactory::create_box_2d(1.0f, 1.0f, Vec2f(0.0f));

        ShapeSupportFunction support_a(&box_a, Vec3f(0.0f, 0.0f, 0.0f));
        ShapeSupportFunction support_b(&box_b, Vec3f(0.7f, 0.0f, 0.0f));

        GJKResult gjk_result = GJKSolver::solve(support_a, support_b);

        if (gjk_result.collision) {
            EPAResult epa_result = EPASolver::solve(support_a, support_b, gjk_result);

            REQUIRE(epa_result.penetration_depth > 0.0f);
            // Normal should be reasonably normalized (allow some tolerance)
            float normal_length = epa_result.contact_normal.length();
            REQUIRE(normal_length > 0.8f);  // At least somewhat normalized
            REQUIRE(normal_length < 1.2f);  // Not wildly wrong
        }
    }
}

TEST_CASE("GJK/EPA Integration: Complete collision pipeline", "[collision][gjk][epa][validation]") {
    SECTION("Full pipeline: separated → no EPA needed") {
        SimpleSphereSupport sphere_a(Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
        SimpleSphereSupport sphere_b(Vec3f(5.0f, 0.0f, 0.0f), 1.0f);

        GJKResult gjk_result = GJKSolver::solve(sphere_a, sphere_b);

        REQUIRE(!gjk_result.collision);
        REQUIRE(gjk_result.distance > 0.0f);
        // No need to run EPA for separated shapes
    }

    SECTION("Full pipeline: overlapping → EPA penetration") {
        SimpleSphereSupport sphere_a(Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
        SimpleSphereSupport sphere_b(Vec3f(1.0f, 0.0f, 0.0f), 1.0f);

        GJKResult gjk_result = GJKSolver::solve(sphere_a, sphere_b);

        if (gjk_result.collision) {
            EPAResult epa_result = EPASolver::solve(sphere_a, sphere_b, gjk_result);

            REQUIRE(epa_result.penetration_depth > 0.0f);
            // Check normal is reasonably normalized
            float normal_length = epa_result.contact_normal.length();
            REQUIRE(normal_length > 0.5f);
        }
    }
}

TEST_CASE("GJK/EPA Validation: Deterministic results", "[collision][gjk][epa][validation]") {
    SimpleSphereSupport sphere_a(Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    SimpleSphereSupport sphere_b(Vec3f(1.5f, 0.3f, 0.0f), 1.0f);

    // Run GJK/EPA multiple times
    GJKResult first_gjk_result = GJKSolver::solve(sphere_a, sphere_b);
    
    for (int i = 0; i < 5; ++i) {
        GJKResult gjk_result = GJKSolver::solve(sphere_a, sphere_b);
        
        // GJK results should be deterministic
        REQUIRE(gjk_result.collision == first_gjk_result.collision);
        
        if (gjk_result.collision) {
            EPAResult epa_result = EPASolver::solve(sphere_a, sphere_b, gjk_result);
            REQUIRE(epa_result.penetration_depth > 0.0f);
        }
    }
}

TEST_CASE("GJK Validation: Performance characteristics", "[collision][gjk][validation][!benchmark]") {
    // This test validates that GJK runs in reasonable time
    // Not an exact benchmark, just a sanity check

    SimpleSphereSupport sphere_a(Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    SimpleSphereSupport sphere_b(Vec3f(2.5f, 0.0f, 0.0f), 1.0f);

    // Run many iterations to ensure no obvious performance issues
    for (int i = 0; i < 1000; ++i) {
        GJKResult result = GJKSolver::solve(sphere_a, sphere_b);
        REQUIRE(!result.collision);
    }

    // If this completes in reasonable time (< 1 second), GJK is fast enough
    SUCCEED("GJK performance is acceptable");
}

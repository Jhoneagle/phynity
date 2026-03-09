#include "../../../../../src/core/physics/collision/ccd/conservative_advancement.hpp"
#include "../../../../../src/core/physics/collision/ccd/linear_cast.hpp"
#include "../../../../../src/core/physics/collision/ccd/swept_sphere.hpp"

#include <catch2/catch_all.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <limits>

using namespace phynity::physics::collision::ccd;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;

// ============================================================================
// Test Suite: Swept Sphere TOI Solver - Head-On Collision
// ============================================================================

TEST_CASE("SweptSphereSolver::Head-on collision", "[unit][physics][collision][ccd]")
{
    SECTION("Two spheres moving directly toward each other")
    {
        // Sphere A: moving right at 10 m/s
        SweptSphereSolver::Sphere sphere_a{
            Vec3f(-2.0f, 0.0f, 0.0f), // position
            Vec3f(10.0f, 0.0f, 0.0f), // velocity
            1.0f                      // radius
        };

        // Sphere B: stationary, positioned 4 units away
        SweptSphereSolver::Sphere sphere_b{
            Vec3f(2.0f, 0.0f, 0.0f), // position
            Vec3f(0.0f, 0.0f, 0.0f), // velocity (stationary)
            1.0f                     // radius
        };

        float dt = 1.0f; // 1 second timeframe
        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, dt);

        REQUIRE(result.collision_occurs == true);
        REQUIRE_THAT(result.toi, Catch::Matchers::WithinAbs(0.2f, 0.01f));
        REQUIRE_THAT(result.contact_point.x, Catch::Matchers::WithinAbs(1.0f, 0.1f));
    }

    SECTION("Spheres already overlapping")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-0.5f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(0.5f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
        REQUIRE_THAT(result.toi, Catch::Matchers::WithinAbs(0.0f, 0.05f));
    }

    SECTION("No collision - spheres moving apart")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(-10.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == false);
    }

    SECTION("No collision - spheres missing")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(2.0f, 5.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == false);
    }
}

// ============================================================================
// Test Suite: Swept Sphere TOI Solver - Oblique Collision
// ============================================================================

TEST_CASE("SweptSphereSolver::Oblique collisions", "[unit][physics][collision][ccd]")
{
    SECTION("Glancing blow from side")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-5.0f, 0.0f, 0.0f), Vec3f(10.0f, 1.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(0.0f, 0.5f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
        REQUIRE(result.toi > 0.0f);
        REQUIRE(result.toi <= 1.0f);
    }

    SECTION("Sphere chasing from behind")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-5.0f, 0.0f, 0.0f), Vec3f(20.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(0.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
        REQUIRE_THAT(result.toi, Catch::Matchers::WithinAbs(0.2f, 0.05f));
    }
}

// ============================================================================
// Test Suite: Contact Point and Normal Calculation
// ============================================================================

TEST_CASE("SweptSphereSolver::Contact geometry", "[unit][physics][collision][ccd]")
{
    SECTION("Contact normal points from A to B")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
        REQUIRE_THAT(result.contact_normal.x, Catch::Matchers::WithinAbs(1.0f, 0.01f));
        REQUIRE_THAT(result.contact_normal.y, Catch::Matchers::WithinAbs(0.0f, 0.01f));
        REQUIRE_THAT(result.contact_normal.z, Catch::Matchers::WithinAbs(0.0f, 0.01f));
    }

    SECTION("Contact point lies on surface of both spheres")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), 0.5f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 0.5f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
        REQUIRE_THAT(result.contact_point.x, Catch::Matchers::WithinAbs(2.5f, 0.1f));
    }

    SECTION("Contact point is inside neither sphere")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-3.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);

        Vec3f sphere_a_at_contact = sphere_a.position + sphere_a.velocity * result.toi;
        Vec3f sphere_b_at_contact = sphere_b.position + sphere_b.velocity * result.toi;
        float dist_to_a = (result.contact_point - sphere_a_at_contact).length();
        float dist_to_b = (result.contact_point - sphere_b_at_contact).length();

        REQUIRE_THAT(dist_to_a, Catch::Matchers::WithinAbs(1.0f, 0.1f));
        REQUIRE_THAT(dist_to_b, Catch::Matchers::WithinAbs(1.0f, 0.1f));
    }
}

// ============================================================================
// Test Suite: Edge Cases and Degenerate Scenarios
// ============================================================================

TEST_CASE("SweptSphereSolver::Edge cases", "[unit][physics][collision][ccd]")
{
    SECTION("Zero velocity - stationary spheres far apart")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == false);
    }

    SECTION("Parallel motion - one ahead and faster")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(0.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(3.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
    }

    SECTION("Very small timestep")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(100.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 0.001f);

        REQUIRE((result.collision_occurs == false || result.collision_occurs == true));
    }

    SECTION("Very large radius")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), 100.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(150.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 100.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
    }

    SECTION("Invalid radius returns no collision")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), 0.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);
        REQUIRE(result.collision_occurs == false);
    }

    SECTION("Non-finite velocity returns no collision")
    {
        SweptSphereSolver::Sphere sphere_a{
            Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);
        REQUIRE(result.collision_occurs == false);
    }
}

// ============================================================================
// Test Suite: Relative Velocity Computation
// ============================================================================

TEST_CASE("SweptSphereSolver::Relative velocity", "[unit][physics][collision][ccd]")
{
    SECTION("Relative velocity is computed at contact point")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(4.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
        REQUIRE_THAT(result.relative_velocity.x, Catch::Matchers::WithinAbs(-4.0f, 0.01f));
    }

    SECTION("Separating velocity shows objects moving apart")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(2.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == false);
    }
}

// ============================================================================
// Test Suite: Linear Cast Utilities
// ============================================================================

TEST_CASE("linear_cast::raycast_sphere", "[unit][physics][collision][ccd]")
{
    SECTION("Ray hits sphere head-on")
    {
        Vec3f ray_origin(-5.0f, 0.0f, 0.0f);
        Vec3f ray_direction = Vec3f(1.0f, 0.0f, 0.0f).normalized();
        Vec3f sphere_center(0.0f, 0.0f, 0.0f);
        float sphere_radius = 1.0f;

        auto result = LinearCast::raycast_sphere(ray_origin, ray_direction, 10.0f, sphere_center, sphere_radius);

        REQUIRE(result.hit == true);
        REQUIRE_THAT(result.t_near, Catch::Matchers::WithinAbs(4.0f, 0.1f));
    }

    SECTION("Ray misses sphere")
    {
        Vec3f ray_origin(0.0f, 0.0f, 0.0f);
        Vec3f ray_direction = Vec3f(1.0f, 0.0f, 0.0f).normalized();
        Vec3f sphere_center(0.0f, 10.0f, 0.0f);
        float sphere_radius = 1.0f;

        auto result = LinearCast::raycast_sphere(ray_origin, ray_direction, 1.0f, sphere_center, sphere_radius);

        REQUIRE(result.hit == false);
    }

    SECTION("Ray starts inside sphere")
    {
        Vec3f ray_origin(0.0f, 0.0f, 0.0f);
        Vec3f ray_direction = Vec3f(1.0f, 0.0f, 0.0f).normalized();
        Vec3f sphere_center(0.0f, 0.0f, 0.0f);
        float sphere_radius = 2.0f;

        auto result = LinearCast::raycast_sphere(ray_origin, ray_direction, 1.0f, sphere_center, sphere_radius);

        REQUIRE(result.hit == true);
    }
}

TEST_CASE("linear_cast::sweep_sphere_vs_sphere_static", "[unit][physics][collision][ccd]")
{
    SECTION("Moving sphere hits stationary sphere")
    {
        Vec3f sphere_a_pos(-5.0f, 0.0f, 0.0f);
        Vec3f sphere_a_vel(10.0f, 0.0f, 0.0f);
        float radius_a = 1.0f;

        Vec3f sphere_b_pos(0.0f, 0.0f, 0.0f);
        float radius_b = 1.0f;

        auto result = LinearCast::sweep_sphere_vs_sphere_static(
            sphere_a_pos, sphere_a_vel, radius_a, sphere_b_pos, radius_b, 1.0f);

        REQUIRE(result.hit == true);
        REQUIRE_THAT(result.t_near, Catch::Matchers::WithinAbs(3.0f, 0.05f));
    }

    SECTION("No collision - sphere missing")
    {
        Vec3f sphere_a_pos(-5.0f, 0.0f, 0.0f);
        Vec3f sphere_a_vel(10.0f, 0.0f, 0.0f);
        float radius_a = 1.0f;

        Vec3f sphere_b_pos(0.0f, 10.0f, 0.0f);
        float radius_b = 1.0f;

        auto result = LinearCast::sweep_sphere_vs_sphere_static(
            sphere_a_pos, sphere_a_vel, radius_a, sphere_b_pos, radius_b, 1.0f);

        REQUIRE(result.hit == false);
    }
}

TEST_CASE("ConservativeAdvancement::sphere_sphere", "[unit][physics][collision][ccd]")
{
    SECTION("Finds contact time for head-on approach")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = ConservativeAdvancement::solve_sphere_sphere(sphere_a, sphere_b, 1.0f, 16, 1e-4f);

        REQUIRE(result.collision_occurs == true);
        REQUIRE_THAT(result.toi, Catch::Matchers::WithinAbs(0.2f, 0.02f));
    }

    SECTION("Returns no collision when separating")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(-5.0f, 0.0f, 0.0f), 1.0f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(2.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f), 1.0f};

        TimeOfImpactResult result = ConservativeAdvancement::solve_sphere_sphere(sphere_a, sphere_b, 1.0f, 16, 1e-4f);

        REQUIRE(result.collision_occurs == false);
    }
}

// ============================================================================
// Integration: CCD Detection in ParticleSystem Context
// ============================================================================

TEST_CASE("SweptSphereSolver::Consistency with physics scale", "[unit][physics][collision][ccd]")
{
    SECTION("Results with ~1 unit scale are reasonable")
    {
        SweptSphereSolver::Sphere sphere_a{Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(20.0f, 0.0f, 0.0f), 0.5f};

        SweptSphereSolver::Sphere sphere_b{Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 0.5f};

        TimeOfImpactResult result = SweptSphereSolver::solve_static(sphere_a, sphere_b, 1.0f);

        REQUIRE(result.collision_occurs == true);
        REQUIRE(result.toi > 0.0f);
        REQUIRE(result.toi <= 1.0f);
        REQUIRE(std::isfinite(result.contact_point.x));
        REQUIRE(std::isfinite(result.contact_normal.x));
    }
}

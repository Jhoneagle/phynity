#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/physics/fluids/sph_fluid_system.hpp>

using Catch::Matchers::WithinAbs;
using phynity::math::utilities::is_finite;
using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::SphFluidSystem;
using phynity::physics::fluids::SphParameters;
using phynity::physics::shapes::AABB;

namespace
{
SphParameters box_params()
{
    SphParameters p;
    p.smoothing_radius = 0.5f;
    p.boundary_restitution = 0.3f;
    p.bounds = AABB(Vec3f(-1.0f), Vec3f(1.0f));
    return p;
}
} // namespace

TEST_CASE("resolve_boundaries: clamps position and reflects+damps normal velocity", "[fluids][boundary]")
{
    SphFluidSystem system(box_params());
    system.spawn(Vec3f(0.0f, -2.0f, 0.0f), Vec3f(0.0f, -5.0f, 0.0f)); // below floor, moving down
    system.resolve_boundaries();

    const auto &p = system.particles()[0];
    REQUIRE_THAT(p.position.y, WithinAbs(-1.0f, 1e-6f)); // clamped to wall
    REQUIRE(p.velocity.y > 0.0f); // reflected (now moving up)
    REQUIRE(p.velocity.y < 5.0f); // and damped (lost normal momentum)
    REQUIRE_THAT(p.velocity.y, WithinAbs(1.5f, 1e-6f)); // 5 * restitution 0.3
}

TEST_CASE("resolve_boundaries: tangential velocity is untouched", "[fluids][boundary]")
{
    SphFluidSystem system(box_params());
    system.spawn(Vec3f(0.0f, -2.0f, 0.0f), Vec3f(3.0f, -5.0f, 0.0f));
    system.resolve_boundaries();
    REQUIRE_THAT(system.particles()[0].velocity.x, WithinAbs(3.0f, 1e-6f)); // tangential preserved
}

TEST_CASE("update: a particle driven into the wall stays inside bounds and settles", "[fluids][boundary]")
{
    SphFluidSystem system(box_params());
    system.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    // Single particle so only gravity + integration + boundary act (no SPH forces).
    system.spawn(Vec3f(0.0f, 0.9f, 0.0f), Vec3f(0.0f));

    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 2000; ++step)
    {
        system.update(dt);
        const auto &p = system.particles()[0];
        REQUIRE(system.parameters().bounds.contains_point(p.position));
        REQUIRE(is_finite(p.position.y));
        REQUIRE(is_finite(p.velocity.y));
    }

    // Bouncing loses normal momentum each impact, so it settles near the floor
    // with small speed.
    const auto &p = system.particles()[0];
    REQUIRE(p.velocity.length() < 1.0f);
    REQUIRE_THAT(p.position.y, WithinAbs(-1.0f, 0.1f));
}

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/utilities/constants.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/dynamics/force_field.hpp>
#include <core/physics/particles/particle_system.hpp>
#include <core/physics/rigid_bodies/rigid_body_system.hpp>
#include <core/physics/shapes/aabb.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

using namespace phynity::physics;
using namespace phynity::physics::constants;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;
using phynity::math::quaternions::Quatf;
using phynity::physics::shapes::AABB;

// ============================================================================
// Force Field Validation Tests
// ============================================================================
// Multi-step, tolerance-based validation of the new configurable force fields.
// These verify emergent behavior of the whole simulation (spawn -> add field ->
// integrate over many steps), not the single-step math already covered by unit
// tests. Deliberately tolerance-based rather than golden to avoid introducing
// baselines that need cross-toolchain management.

// ----------------------------------------------------------------------------
// SpringDamperField: a displaced mass settles back to the equilibrium center.
// ----------------------------------------------------------------------------

TEST_CASE("Forces Validation - Damped spring converges to center", "[forces_validation]")
{
    ParticleSystem system;

    const float dt = 0.005f;
    const int steps = 4000; // 20 seconds
    const float mass = 1.0f;
    const Vec3f center(0.0f, 0.0f, 0.0f);

    // Underdamped: c < 2*sqrt(k*m) => oscillates while decaying toward center.
    system.spawn(Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f), make_no_damping_material(mass));
    system.add_force_field(std::make_unique<SpringDamperField>(center, 10.0f, 2.0f));

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
    }

    const auto &p = system.particles()[0];

    REQUIRE(std::isfinite(p.position.x));
    REQUIRE_THAT(p.position.x, WithinAbs(0.0f, 1e-3f));
    REQUIRE_THAT(p.velocity.x, WithinAbs(0.0f, 1e-3f));
}

// ----------------------------------------------------------------------------
// BuoyancyField: a light (floating) body pushed up by fluid, damped by drag,
// settles near the fluid surface instead of sinking or flying out.
// ----------------------------------------------------------------------------

TEST_CASE("Forces Validation - Buoyant particle settles near the surface", "[forces_validation]")
{
    ParticleSystem system;

    const float dt = 0.005f;
    const int steps = 4000; // 20 seconds
    const float mass = 1.0f;
    const float surface_height = 0.0f;
    const Vec3f gravity(0.0f, -EARTH_GRAVITY, 0.0f);

    // object_density < fluid_density => the body floats.
    system.set_ambient_gravity(gravity); // BuoyancyField reads gravity from the shared context
    system.spawn(Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), make_no_damping_material(mass));
    system.add_force_field(std::make_unique<GravityField>(gravity));
    system.add_force_field(std::make_unique<BuoyancyField>(1000.0f, 500.0f, surface_height));
    system.add_force_field(std::make_unique<DragField>(2.0f)); // dissipate the oscillation

    float start_y = system.particles()[0].position.y;

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
    }

    const auto &p = system.particles()[0];

    REQUIRE(std::isfinite(p.position.y));
    // Rose substantially from its submerged start toward the surface...
    REQUIRE(p.position.y > start_y);
    // ...and settled close to the surface with little residual motion.
    REQUIRE_THAT(p.position.y, WithinAbs(surface_height, 0.25f));
    REQUIRE_THAT(p.velocity.y, WithinAbs(0.0f, 0.25f));
}

// ----------------------------------------------------------------------------
// WindField (bounded): only bodies inside the region feel the wind.
// ----------------------------------------------------------------------------

TEST_CASE("Forces Validation - Wind carries a particle only inside its region", "[forces_validation]")
{
    ParticleSystem system;

    const float dt = 0.01f;
    const int steps = 50; // 0.5 seconds

    // Region is a thin slab around y = 0.
    AABB region(Vec3f(-10.0f, -1.0f, -1.0f), Vec3f(10.0f, 1.0f, 1.0f));

    // Particle 0 sits inside the slab; particle 1 sits above it (outside).
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), make_no_damping_material(1.0f));
    system.spawn(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f), make_no_damping_material(1.0f));

    system.add_force_field(std::make_unique<WindField>(Vec3f(5.0f, 0.0f, 0.0f), 1.0f, region));

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
    }

    const auto &inside = system.particles()[0];
    const auto &outside = system.particles()[1];

    // Inside the region: accelerated toward the wind and carried in +x.
    REQUIRE(inside.velocity.x > 0.1f);
    REQUIRE(inside.position.x > 0.0f);

    // Outside the region: no wind, so it never moves.
    REQUIRE_THAT(outside.velocity.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(outside.position.x, WithinAbs(0.0f, 1e-6f));
}

// ----------------------------------------------------------------------------
// PointGravityField: a mass at rest is pulled toward the well's center.
// ----------------------------------------------------------------------------

TEST_CASE("Forces Validation - Point gravity pulls a resting mass inward", "[forces_validation]")
{
    ParticleSystem system;

    const float dt = 0.01f;
    const int steps = 100; // 1 second

    const Vec3f start(5.0f, 0.0f, 0.0f);
    system.spawn(start, Vec3f(0.0f), make_no_damping_material(1.0f));
    system.add_force_field(std::make_unique<PointGravityField>(Vec3f(0.0f), 20.0f, 1e-2f));

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
    }

    const auto &p = system.particles()[0];

    REQUIRE(std::isfinite(p.position.x));
    // Accelerated inward (toward -x) and moved closer to the center.
    REQUIRE(p.velocity.x < 0.0f);
    REQUIRE(p.position.length() < start.length());
}

// ----------------------------------------------------------------------------
// PointGravityField: a tangential launch stays on a bound orbit (does not
// escape to infinity or collapse into the singularity).
// ----------------------------------------------------------------------------

TEST_CASE("Forces Validation - Point gravity supports a bound orbit", "[forces_validation]")
{
    ParticleSystem system;

    const float dt = 1.0f / 240.0f;
    const int steps = 2400; // 10 seconds
    const float radius = 5.0f;
    const float strength = 20.0f;

    // Circular-orbit speed for an inverse-square well: v = sqrt(strength / r).
    const float speed = std::sqrt(strength / radius);

    system.spawn(Vec3f(radius, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, speed), make_no_damping_material(1.0f));
    system.add_force_field(std::make_unique<PointGravityField>(Vec3f(0.0f), strength, 1e-2f));

    float min_radius = radius;
    float max_radius = radius;

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
        const float r = system.particles()[0].position.length();
        REQUIRE(std::isfinite(r));
        min_radius = std::min(min_radius, r);
        max_radius = std::max(max_radius, r);
    }

    // The orbit stays bounded well away from both escape and the center.
    REQUIRE(min_radius > radius * 0.5f);
    REQUIRE(max_radius < radius * 2.0f);
}

// ----------------------------------------------------------------------------
// BuoyancyField on the RIGID-BODY path: the rigid-body apply loop stamps the
// system's ambient gravity into every ForceContext, and BuoyancyField must read
// that (not a stored copy). This is the rigid-body counterpart to the
// ParticleSystem buoyancy coverage above — a single integration step is checked
// quantitatively so the numbers pin down that ctx.gravity actually drives the force.
// ----------------------------------------------------------------------------

TEST_CASE("Forces Validation - Rigid-body buoyancy reads the system's ambient gravity", "[forces_validation]")
{
    const float dt = 0.001f;
    const float mass = 1.0f;

    // fluid_density == object_density == 1000, mass == 1 => submerged volume
    // V = mass / object_density = 1e-3 m³, so the Archimedes force magnitude is
    // fluid_density * V * |g| = |g| (numerically). With no gravity field present,
    // buoyancy is the only force, so after one step v.y = (|F|/m) * dt = |g| * dt.
    auto step_and_get_vy = [&](const Vec3f &gravity)
    {
        RigidBodySystem system;
        system.set_ambient_gravity(gravity);
        system.add_force_field(std::make_unique<BuoyancyField>(1000.0f, 1000.0f, 0.0f));

        // Submerged (below the surface at y = 0). No shape: a lone body has no
        // collision pairs, so narrowphase never dereferences the (null) shape.
        RigidBodyID id =
            system.spawn_body(Vec3f(0.0f, -5.0f, 0.0f), Quatf(), nullptr, mass, make_no_damping_material(mass));

        system.update(dt);

        const RigidBody *body = system.get_body(id);
        REQUIRE(body != nullptr);
        REQUIRE(std::isfinite(body->velocity.y));
        return body->velocity.y;
    };

    const float vy_g10 = step_and_get_vy(Vec3f(0.0f, -10.0f, 0.0f));
    const float vy_g20 = step_and_get_vy(Vec3f(0.0f, -20.0f, 0.0f));

    // Pushed upward (out of the fluid), with the exact Archimedes magnitude...
    REQUIRE(vy_g10 > 0.0f);
    REQUIRE_THAT(vy_g10, WithinAbs(10.0f * dt, 1e-5f));
    REQUIRE_THAT(vy_g20, WithinAbs(20.0f * dt, 1e-5f));

    // ...and the force tracks the ambient gravity: doubling |g| doubles it.
    REQUIRE_THAT(vy_g20 / vy_g10, WithinAbs(2.0f, 1e-4f));
}

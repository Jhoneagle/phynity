#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/dynamics/force_field.hpp>
#include <physics_context.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <algorithm>
#include <cmath>

using namespace phynity::app;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using phynity::math::quaternions::Quatf;
using phynity::physics::BuoyancyField;
using phynity::physics::RigidBodyID;
using Catch::Matchers::WithinAbs;

// ============================================================================
// PhysicsContext force-field integration tests
// ============================================================================
// These cover the "single source of truth" gravity plumbing that the sandbox
// PhysicsContext owns: set_gravity() must publish the ambient gravity to BOTH
// the particle and rigid-body systems, and medium-dependent fields (buoyancy)
// on the rigid-body path must read that published value via the ForceContext.
// Unit / core-level tests can't reach this wiring — it only exists at the app
// layer, so it needs a sandbox_context-linked test to guard against regression.

namespace
{

PhysicsContext::Config make_serial_config()
{
    PhysicsContext::Config config;
    config.target_fps = 120.0f;
    config.use_determinism = true;
    config.enable_jobs = false; // pure serial path — single body, no threads needed
    return config;
}

} // namespace

// ----------------------------------------------------------------------------
// set_gravity() is the single publish point: it must reach both systems.
// ----------------------------------------------------------------------------

TEST_CASE("Forces Integration - set_gravity publishes ambient gravity to both systems",
          "[forces_validation][context]")
{
    PhysicsContext::Config config = make_serial_config();
    config.gravity = Vec3f(0.0f, -9.81f, 0.0f);
    PhysicsContext ctx(config);

    // The constructor already publishes the configured gravity to both systems.
    const Vec3f &initial_p = ctx.particle_system().ambient_gravity();
    const Vec3f &initial_r = ctx.rigid_body_system().ambient_gravity();
    REQUIRE_THAT(initial_p.y, WithinAbs(-9.81f, 1e-6f));
    REQUIRE_THAT(initial_r.y, WithinAbs(-9.81f, 1e-6f));

    // Changing gravity mid-simulation must update BOTH systems in lock-step, so
    // no field can be left reading a stale "down".
    const Vec3f moon(0.0f, -1.62f, 0.0f);
    ctx.set_gravity(moon);

    const Vec3f &after_p = ctx.particle_system().ambient_gravity();
    const Vec3f &after_r = ctx.rigid_body_system().ambient_gravity();

    REQUIRE(after_p.x == moon.x);
    REQUIRE(after_p.y == moon.y);
    REQUIRE(after_p.z == moon.z);
    REQUIRE(after_r.x == moon.x);
    REQUIRE(after_r.y == moon.y);
    REQUIRE(after_r.z == moon.z);
}

// ----------------------------------------------------------------------------
// End-to-end: a submerged rigid body's buoyancy is driven by the gravity that
// set_gravity() published. Contrasting a normal-gravity run against a
// zero-gravity run proves buoyancy reads the published value rather than a
// stored/default copy — with gravity off, buoyancy produces no lift at all.
// ----------------------------------------------------------------------------

TEST_CASE("Forces Integration - Rigid-body buoyancy tracks gravity set via set_gravity",
          "[forces_validation][context]")
{
    const int steps = 240; // 2 seconds at 120 fps
    const float mass = 1.0f;
    const float start_y = -3.0f;   // start well below the fluid surface
    const float surface_y = 0.0f;

    // A submerged, less-dense-than-water body: object 500 < fluid 1000 => floats.
    auto run_with_gravity = [&](const Vec3f &gravity) {
        PhysicsContext ctx(make_serial_config());
        ctx.set_gravity(gravity); // publishes to both systems; also (re)adds a GravityField

        // Added AFTER set_gravity so it survives the field reset set_gravity performs.
        ctx.rigid_body_system().add_force_field(std::make_unique<BuoyancyField>(1000.0f, 500.0f, surface_y));

        RigidBodyID id = ctx.spawn_body(
            Vec3f(0.0f, start_y, 0.0f), Quatf(), nullptr, mass, make_no_damping_material(mass));

        float max_y = start_y;
        for (int i = 0; i < steps; ++i)
        {
            ctx.step_deterministic();
            const auto *body = ctx.rigid_body_system().get_body(id);
            REQUIRE(body != nullptr);
            REQUIRE(std::isfinite(body->position.y));
            max_y = std::max(max_y, body->position.y);
        }
        return max_y;
    };

    // With real gravity, net upward force lifts the body out of the depths and
    // up to the surface region.
    const float max_y_earth = run_with_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    REQUIRE(max_y_earth > start_y);          // it rose...
    REQUIRE(max_y_earth > surface_y - 0.5f); // ...essentially reaching the surface

    // With gravity published as zero, buoyancy has nothing to scale against: the
    // body neither sinks nor floats — it must not move at all.
    const float max_y_zero_g = run_with_gravity(Vec3f(0.0f, 0.0f, 0.0f));
    REQUIRE_THAT(max_y_zero_g, WithinAbs(start_y, 1e-6f));
}

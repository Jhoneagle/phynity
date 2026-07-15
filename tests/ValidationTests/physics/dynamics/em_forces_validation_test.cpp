#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/utilities/constants.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/dynamics/force_field.hpp>
#include <core/physics/particles/particle_system.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

using namespace phynity::physics;
using namespace phynity::physics::constants;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Electromagnetism Field Validation Tests
// ============================================================================
// Multi-step, tolerance-based validation of the electromagnetic force fields:
// emergent trajectory behavior (spawn -> add field -> integrate), not the
// single-step math already covered by the unit tests. Deliberately tolerance-
// based (expected ranges, not golden baselines).
//
// Integrator note: the engine uses semi-implicit Euler, which is NOT energy-
// conserving for the velocity-dependent magnetic force — kinetic energy drifts
// upward and a cyclotron orbit spirals slowly outward. These tests therefore use
// small dt over 1-2 orbits and assert BOUNDED, approximately-circular behavior
// within a loose tolerance rather than exact conservation. All scenes use a
// no-damping material so material damping does not corrupt the measurements.

// ----------------------------------------------------------------------------
// Uniform E acceleration (velocity-independent — the cleanest analytic check).
// A charge released from rest in a uniform field follows x = 1/2 a t², a = qE/m.
// ----------------------------------------------------------------------------

TEST_CASE("EM Validation - Uniform E accelerates a charge from rest", "[em_validation]")
{
    ParticleSystem system;
    system.set_ambient_gravity(Vec3f(0.0f)); // isolate the electric force

    const float dt = 0.001f;
    const int steps = 1000; // t = 1 s
    const float mass = 2.0f;
    const float charge = 3.0f;
    const Vec3f e_field(4.0f, 0.0f, 0.0f);

    Material mat = make_no_damping_material(mass);
    mat.charge = charge;
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), mat);
    system.add_force_field(std::make_unique<UniformElectricField>(e_field));

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
    }

    const auto &p = system.particles()[0];
    const float t = dt * static_cast<float>(steps);
    const float a = charge * e_field.x / mass; // = 6
    const float expected_x = 0.5f * a * t * t; // = 3

    REQUIRE(std::isfinite(p.position.x));
    REQUIRE_THAT(p.position.x, WithinAbs(expected_x, 0.02f * expected_x)); // within 2%
    REQUIRE_THAT(p.position.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(p.position.z, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(p.velocity.x, WithinAbs(a * t, 0.01f * a * t)); // v = a t
}

TEST_CASE("EM Validation - Negative charge accelerates opposite to E", "[em_validation]")
{
    ParticleSystem system;
    system.set_ambient_gravity(Vec3f(0.0f));

    const float dt = 0.001f;
    const int steps = 500;

    Material mat = make_no_damping_material(1.0f);
    mat.charge = -1.0f;
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), mat);
    system.add_force_field(std::make_unique<UniformElectricField>(Vec3f(5.0f, 0.0f, 0.0f)));

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
    }

    // Force = qE = -1 * (+5) -> particle moves in -x.
    REQUIRE(system.particles()[0].position.x < 0.0f);
}

// ----------------------------------------------------------------------------
// Cyclotron motion: a charge moving perpendicular to a uniform B traces a
// circle of radius r = m|v| / (|q||B|). Integrator-limited (see header note):
// asserted approximately-circular and bounded, not exactly closed/conserved.
// ----------------------------------------------------------------------------

TEST_CASE("EM Validation - Cyclotron orbit is approximately circular", "[em_validation]")
{
    ParticleSystem system;
    system.set_ambient_gravity(Vec3f(0.0f));

    const float mass = 1.0f;
    const float charge = 1.0f;
    const float b_mag = 1.0f;
    const float speed = 1.0f;

    const float radius = mass * speed / (charge * b_mag); // = 1
    const float omega = charge * b_mag / mass; // = 1
    const float period = TWO_PI / omega; // = 2π

    const float dt = 0.001f; // omega*dt = 1e-3 -> ~0.6% energy growth/orbit
    const int steps = static_cast<int>(period / dt); // ~one orbit

    // v = (speed,0,0), B = (0,0,b): F = q v×B initially -y, so the circle's
    // center is at (0, -radius, 0) when starting from the origin.
    Material mat = make_no_damping_material(mass);
    mat.charge = charge;
    system.spawn(Vec3f(0.0f), Vec3f(speed, 0.0f, 0.0f), mat);
    system.add_force_field(std::make_unique<MagneticField>(Vec3f(0.0f, 0.0f, b_mag)));

    const Vec3f center(0.0f, -radius, 0.0f);
    float min_r = radius;
    float max_r = radius;
    float max_speed = speed;

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
        const auto &p = system.particles()[0];
        REQUIRE(std::isfinite(p.position.x));

        const float r = (p.position - center).length();
        min_r = std::min(min_r, r);
        max_r = std::max(max_r, r);
        max_speed = std::max(max_speed, p.velocity.length());

        // Motion stays in the xy-plane (B along z does no z-work).
        REQUIRE_THAT(p.position.z, WithinAbs(0.0f, 1e-6f));
    }

    // Approximately circular: the orbital radius stays within a loose band of the
    // analytic radius throughout (upper bound admits the Euler energy growth).
    REQUIRE(min_r > 0.9f * radius);
    REQUIRE(max_r < 1.1f * radius);

    // Energy grows but stays bounded over one orbit (NOT exact conservation).
    REQUIRE(max_speed < 1.1f * speed);

    // After ~one period the charge returns near its start (integrator-limited).
    const auto &p = system.particles()[0];
    REQUIRE_THAT(p.position.x, WithinAbs(0.0f, 0.15f * radius));
    REQUIRE_THAT(p.position.y, WithinAbs(0.0f, 0.15f * radius));
}

// ----------------------------------------------------------------------------
// E×B drift: in crossed uniform E and B fields the guiding center drifts at
// v_d = E×B / |B|² (independent of charge and mass). The instantaneous motion
// is a cycloid, so the drift is measured as the time-averaged velocity.
// ----------------------------------------------------------------------------

TEST_CASE("EM Validation - Crossed E and B produce E-cross-B drift", "[em_validation]")
{
    ParticleSystem system;
    system.set_ambient_gravity(Vec3f(0.0f));

    const float mass = 1.0f;
    const float charge = 1.0f;
    const float b_mag = 1.0f; // B = (0,0,1)
    const float e_mag = 1.0f; // E = (0,1,0)

    const float omega = charge * b_mag / mass;
    const float period = TWO_PI / omega;
    const float dt = 0.001f;
    const int periods = 2;
    const int steps = static_cast<int>(periods * period / dt);

    // v_d = E×B / |B|² = (0,e,0)×(0,0,b) / b² = (e/b, 0, 0).
    const float expected_drift_x = e_mag / b_mag; // = 1

    Material mat = make_no_damping_material(mass);
    mat.charge = charge;
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), mat);
    system.add_force_field(std::make_unique<UniformElectricField>(Vec3f(0.0f, e_mag, 0.0f)));
    system.add_force_field(std::make_unique<MagneticField>(Vec3f(0.0f, 0.0f, b_mag)));

    for (int i = 0; i < steps; ++i)
    {
        system.update(dt);
        REQUIRE(std::isfinite(system.particles()[0].position.x));
    }

    // Time-averaged velocity = net displacement / elapsed time.
    const auto &p = system.particles()[0];
    const float elapsed = dt * static_cast<float>(steps);
    const float avg_vx = p.position.x / elapsed;
    const float avg_vy = p.position.y / elapsed;

    // Loose tolerance under Euler: the drift is the load-bearing claim, not the
    // (integrator-limited) gyration on top of it.
    REQUIRE_THAT(avg_vx, WithinAbs(expected_drift_x, 0.1f * expected_drift_x));
    REQUIRE_THAT(avg_vy, WithinAbs(0.0f, 0.1f * expected_drift_x));
}

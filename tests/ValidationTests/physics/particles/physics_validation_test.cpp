#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>

#include <core/physics/micro/particle_system.hpp>
#include <core/physics/common/timestep_controller.hpp>
#include <core/physics/common/material.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/math/utilities/numeric.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <core/math/utilities/constants.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

using namespace phynity::physics;
using namespace phynity::physics::constants;
using namespace phynity::math::vectors;
using namespace phynity::math::utilities;
using namespace phynity::test::helpers;
using phynity::math::utilities::mathf;
using Catch::Approx;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ============================================================================
// Physics Validation Tests - Phase 3
// ============================================================================
// These tests validate the particle system against known analytical solutions
// and conservation laws to ensure correctness of the physics simulation.

// ============================================================================
// Test 1: Free Fall Validation
// ============================================================================

TEST_CASE("Physics Validation - Free fall from rest", "[physics_validation]") {
    // Problem: Single particle dropped from y=100m with g=9.80665 m/s²
    // Analytical solution: y(t) = 100 - 0.5*g*t², v(t) = -g*t
    
    ParticleSystem system;
    TimestepController controller(1.0f / 60.0f);  // 60 FPS
    
    // Setup: particle at rest at y=100
    system.spawn(
        Vec3f(0.0f, 100.0f, 0.0f),
        Vec3f(0.0f, 0.0f, 0.0f),
        make_no_damping_material(1.0f)
    );
    
    // Add gravity field
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f)));
    
    // Simulate for 2 seconds
    float simulation_time = 2.0f;
    float dt_frame = 0.016f;  // 60 FPS frame time
    int frames = static_cast<int>(simulation_time / dt_frame);
    
    for (int i = 0; i < frames; ++i) {
        controller.accumulate(dt_frame);
        float dt = 0.0f;
        while ((dt = controller.step()) > 0.0f) {
            system.update(dt);
        }
    }
    
    // Verify final state
    float actual_time = static_cast<float>(frames) * dt_frame;
    const auto& particles = system.particles();
    REQUIRE(particles.size() == 1);
    
    const auto& p = particles[0];
    
    // Analytical solution at t ≈ 2.0s
    float expected_y = 100.0f - 0.5f * EARTH_GRAVITY * actual_time * actual_time;
    float expected_v = -EARTH_GRAVITY * actual_time;
    
    // Tolerance: 1% for position, 2% for velocity (numerical integration)
    REQUIRE_THAT(p.position.y, WithinRel(expected_y, tolerance::ANALYTICAL_REL_1PCT));
    REQUIRE_THAT(p.velocity.y, WithinRel(expected_v, tolerance::ANALYTICAL_REL_2PCT));
}

// ============================================================================
// Test 1.1: Reference-Based Determinism (Fixed-Step Free Fall)
// ============================================================================

TEST_CASE("Physics Validation - Reference-based determinism (free fall)", "[physics_validation]") {
    // Reference values derived from semi-implicit Euler with constant acceleration:
    // v_n = v0 + n * a * dt
    // p_n = p0 + dt * (n * v0 + a * dt * n * (n + 1) / 2)

    ParticleSystem system;

    const float dt = 0.01f;
    const int steps = 100;  // 1 second
    const float a = -EARTH_GRAVITY;

    system.spawn(
        Vec3f(0.0f, 100.0f, 0.0f),
        Vec3f(0.0f, 0.0f, 0.0f),
        make_no_damping_material(1.0f)
    );
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, a, 0.0f)));

    for (int i = 0; i < steps; ++i) {
        system.update(dt);
    }

    const auto& p = system.particles()[0];

    const float n = static_cast<float>(steps);
    const float expected_v = n * a * dt;
    const float expected_y = 100.0f + dt * (a * dt * n * (n + 1.0f) * 0.5f);

    REQUIRE_THAT(p.velocity.y, WithinAbs(expected_v, tolerance::VELOCITY));
    REQUIRE_THAT(p.position.y, WithinAbs(expected_y, tolerance::POSITION));
}

// ============================================================================
// Test 1.2: Reference-Based Determinism (Constant Velocity)
// ============================================================================

TEST_CASE("Physics Validation - Reference-based determinism (constant velocity)", "[physics_validation]") {
    // No forces, no damping. Velocity remains constant.
    // p_n = p0 + v0 * dt * n

    ParticleSystem system;

    const float dt = 0.02f;
    const int steps = 200;  // 4 seconds
    const Vec3f v0(1.25f, -2.5f, 0.75f);
    const Vec3f p0(10.0f, -5.0f, 2.0f);

    system.spawn(p0, v0, make_no_damping_material(1.0f));

    for (int i = 0; i < steps; ++i) {
        system.update(dt);
    }

    const auto& p = system.particles()[0];
    const float n = static_cast<float>(steps);
    Vec3f expected = p0 + v0 * (dt * n);

    REQUIRE(approx_equal(p.position, expected, tolerance::POSITION));
    REQUIRE(approx_equal(p.velocity, v0, tolerance::VELOCITY));
}

// ============================================================================
// Test 1.3: Reference-Based Determinism (Constant Acceleration with v0)
// ============================================================================

TEST_CASE("Physics Validation - Reference-based determinism (constant accel)", "[physics_validation]") {
    // Semi-implicit Euler with constant acceleration and nonzero v0:
    // v_n = v0 + n * a * dt
    // p_n = p0 + dt * (n * v0 + a * dt * n * (n + 1) / 2)

    ParticleSystem system;

    const float dt = 0.01f;
    const int steps = 150;  // 1.5 seconds
    const float a = -4.0f;
    const float v0 = 3.0f;
    const float p0 = 2.0f;

    system.spawn(
        Vec3f(0.0f, p0, 0.0f),
        Vec3f(0.0f, v0, 0.0f),
        make_no_damping_material(1.0f)
    );
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, a, 0.0f)));

    for (int i = 0; i < steps; ++i) {
        system.update(dt);
    }

    const auto& p = system.particles()[0];
    const float n = static_cast<float>(steps);
    const float expected_v = v0 + n * a * dt;
    const float expected_y = p0 + dt * (n * v0 + a * dt * n * (n + 1.0f) * 0.5f);

    REQUIRE_THAT(p.velocity.y, WithinAbs(expected_v, tolerance::VELOCITY));
    REQUIRE_THAT(p.position.y, WithinAbs(expected_y, tolerance::VELOCITY));
}

// ============================================================================
// Test 1.4: Reference-Based Determinism (Linear Drag, Discrete)
// ============================================================================

TEST_CASE("Physics Validation - Reference-based determinism (linear drag)", "[physics_validation]") {
    // Discrete semi-implicit Euler with drag: v_{n+1} = v_n * (1 - k*dt)
    // p_n = p0 + dt * v0 * sum_{i=1..n} (1 - k*dt)^i

    ParticleSystem system;

    const float dt = 0.01f;
    const int steps = 200;  // 2 seconds
    const float k = 0.2f;
    const float v0 = 5.0f;
    const float p0 = -1.0f;

    system.spawn(
        Vec3f(p0, 0.0f, 0.0f),
        Vec3f(v0, 0.0f, 0.0f),
        make_no_damping_material(1.0f)
    );
    system.add_force_field(std::make_unique<DragField>(k));

    for (int i = 0; i < steps; ++i) {
        system.update(dt);
    }

    const auto& p = system.particles()[0];
    const float ratio = 1.0f - k * dt;
    const float expected_v = v0 * std::pow(ratio, static_cast<float>(steps));
    const float expected_x = p0 + dt * v0 * geometric_series_sum(ratio, steps);

    REQUIRE_THAT(p.velocity.x, WithinAbs(expected_v, tolerance::POSITION));
    REQUIRE_THAT(p.position.x, WithinAbs(expected_x, tolerance::POSITION));
}

// ============================================================================
// Test 1.5: Reference-Based Determinism (2D Coupled: Gravity + Lateral)
// ============================================================================

TEST_CASE("Physics Validation - Reference-based determinism (2D coupled)", "[physics_validation]") {
    // 2D motion: constant lateral velocity (x) + constant gravity (y)
    // x_n = x0 + vx * dt * n
    // y_n = y0 + dt * (n * vy0 + ay * dt * n * (n + 1) / 2)
    // vy_n = vy0 + n * ay * dt

    ParticleSystem system;

    const float dt = 0.01f;
    const int steps = 250;  // 2.5 seconds
    const Vec3f p0(0.0f, 10.0f, 0.0f);
    const Vec3f v0(3.0f, 2.0f, 0.0f);
    const float ay = -EARTH_GRAVITY;

    system.spawn(p0, v0, make_no_damping_material(1.0f));
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, ay, 0.0f)));

    for (int i = 0; i < steps; ++i) {
        system.update(dt);
    }

    const auto& p = system.particles()[0];
    const float n = static_cast<float>(steps);

    // X-axis: pure constant velocity
    const float expected_x = p0.x + v0.x * dt * n;
    const float expected_vx = v0.x;

    // Y-axis: constant acceleration
    const float expected_y = p0.y + dt * (n * v0.y + ay * dt * n * (n + 1.0f) * 0.5f);
    const float expected_vy = v0.y + n * ay * dt;

    REQUIRE_THAT(p.position.x, WithinAbs(expected_x, tolerance::POSITION));
    REQUIRE_THAT(p.position.y, WithinAbs(expected_y, tolerance::POSITION));
    REQUIRE_THAT(p.velocity.x, WithinAbs(expected_vx, tolerance::VELOCITY));
    REQUIRE_THAT(p.velocity.y, WithinAbs(expected_vy, tolerance::POSITION));
}

// ============================================================================
// Test 1.6: Reference-Based Determinism (Spring Force)
// ============================================================================

TEST_CASE("Physics Validation - Reference-based determinism (spring force)", "[physics_validation]") {
    // Spring force: F = -k * (x - center), a = -k/m * x (center = 0)
    // Semi-implicit Euler with spring:
    // v_{n+1} = v_n - (k/m) * x_n * dt
    // x_{n+1} = x_n + v_{n+1} * dt
    //
    // Discrete recurrence (1D):
    // v_{n+1} = v_n - omega^2 * x_n * dt
    // x_{n+1} = x_n + v_{n+1} * dt
    // where omega^2 = k/m

    ParticleSystem system;

    const float dt = 0.005f;  // Smaller timestep for spring stability
    const int steps = 400;    // 2 seconds
    const float mass = 1.0f;
    const float k = 10.0f;
    const float omega_sq = k / mass;
    
    const float x0 = 2.0f;
    const float v0 = 0.0f;

    system.spawn(
        Vec3f(x0, 0.0f, 0.0f),
        Vec3f(v0, 0.0f, 0.0f),
        make_no_damping_material(mass)
    );
    system.add_force_field(std::make_unique<SpringField>(Vec3f(0.0f, 0.0f, 0.0f), k));

    // Compute reference using discrete semi-implicit Euler recurrence
    float x_ref = x0;
    float v_ref = v0;
    for (int i = 0; i < steps; ++i) {
        v_ref = v_ref - omega_sq * x_ref * dt;
        x_ref = x_ref + v_ref * dt;
    }

    for (int i = 0; i < steps; ++i) {
        system.update(dt);
    }

    const auto& p = system.particles()[0];

    // Spring oscillations accumulate error more than constant motion
    REQUIRE_THAT(p.position.x, WithinAbs(x_ref, tolerance::REFERENCE_ABS));
    REQUIRE_THAT(p.velocity.x, WithinAbs(v_ref, tolerance::REFERENCE_ABS));
}

// ============================================================================
// Test 2: Momentum Conservation (No External Forces)
// ============================================================================

TEST_CASE("Physics Validation - Momentum conservation in zero gravity", "[physics_validation]") {
    // Problem: Two particles with different velocities, no gravity
    // Expected: Total momentum remains constant
    
    ParticleSystem system;
    TimestepController controller(1.0f / 60.0f);
    
    // Spawn particles with different velocities (no damping)
    system.spawn(
        Vec3f(-5.0f, 0.0f, 0.0f),
        Vec3f(10.0f, 0.0f, 0.0f),
        make_no_damping_material(2.0f)
    );  // mass=2kg
    system.spawn(
        Vec3f(5.0f, 0.0f, 0.0f),
        Vec3f(-5.0f, 0.0f, 0.0f),
        make_no_damping_material(1.0f)
    );   // mass=1kg
    
    // Record initial momentum
    auto diag_initial = system.compute_diagnostics();
    Vec3f initial_momentum = diag_initial.total_momentum;
    
    // Simulate for 3 seconds
    float dt_frame = 0.016f;
    int frames = static_cast<int>(3.0f / dt_frame);
    
    for (int i = 0; i < frames; ++i) {
        controller.accumulate(dt_frame);
        float dt = 0.0f;
        while ((dt = controller.step()) > 0.0f) {
            system.update(dt);
        }
    }
    
    // Check final momentum
    auto diag_final = system.compute_diagnostics();
    Vec3f final_momentum = diag_final.total_momentum;
    
    // Momentum should be conserved (no external forces, no damping)
    REQUIRE(approx_equal(final_momentum, initial_momentum, tolerance::MOMENTUM));
}

// ============================================================================
// Test 3: Energy Conservation (With Gravity)
// ============================================================================

TEST_CASE("Physics Validation - Energy conservation with gravity", "[physics_validation]") {
    // Problem: Single particle in gravity field (potential + kinetic energy constant)
    // Expected: E_total = KE + PE = constant
    
    ParticleSystem system;
    TimestepController controller(1.0f / 60.0f);
    
    // Setup: particle at y=50 with upward velocity
    float mass = 1.0f;
    system.spawn(
        Vec3f(0.0f, 50.0f, 0.0f),
        Vec3f(0.0f, 20.0f, 0.0f),
        make_no_damping_material(mass)
    );
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f)));
    
    // Calculate initial total energy
    float KE_initial = 0.5f * mass * (20.0f * 20.0f);  // v_y = 20 m/s
    float PE_initial = mass * EARTH_GRAVITY * 50.0f;            // y = 50m
    float E_total_initial = KE_initial + PE_initial;
    
    // Simulate for 3 seconds
    float dt_frame = 0.016f;
    int frames = static_cast<int>(3.0f / dt_frame);
    float max_energy_error = 0.0f;
    
    for (int i = 0; i < frames; ++i) {
        controller.accumulate(dt_frame);
        float dt = 0.0f;
        while ((dt = controller.step()) > 0.0f) {
            system.update(dt);
        }
        
        // Check energy at each step
        const auto& p = system.particles()[0];
        float KE = 0.5f * mass * (p.velocity.x * p.velocity.x + 
                                   p.velocity.y * p.velocity.y + 
                                   p.velocity.z * p.velocity.z);
        float PE = mass * EARTH_GRAVITY * p.position.y;
        float E_total = KE + PE;
        
        float error = std::abs(E_total - E_total_initial);
        max_energy_error = std::max(max_energy_error, error);
    }
    
    // Energy should be conserved (tolerance: 2% - allows for numerical integration)
    float tolerance = E_total_initial * 0.02f;
    REQUIRE(max_energy_error < tolerance);
}

// ============================================================================
// Test 4: Projectile Motion
// ============================================================================

TEST_CASE("Physics Validation - Projectile motion range", "[physics_validation]") {
    // Problem: Projectile launched at 45° with v=20 m/s from ground
    // Expected: Range ≈ v²sin(2θ)/g = 400*sin(90°)/9.80665 ≈ 40.77m
    
    ParticleSystem system;
    TimestepController controller(1.0f / 60.0f);
    
    // Launch angle: 45 degrees
    float v0 = 20.0f;
    float angle = mathf::pi / 4.0f;  // 45 degrees
    float vx = v0 * std::cos(angle);
    float vy = v0 * std::sin(angle);
    
    system.spawn(
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(vx, vy, 0.0f),
        make_no_damping_material(1.0f)
    );
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f)));
    
    // Simulate until particle returns to y ≈ 0
    float dt_frame = 0.016f;
    float max_time = 5.0f;  // Sufficient for projectile to land
    int max_frames = static_cast<int>(max_time / dt_frame);
    
    float landing_x = 0.0f;
    bool landed = false;
    
    for (int i = 0; i < max_frames; ++i) {
        controller.accumulate(dt_frame);
        float dt = 0.0f;
        while ((dt = controller.step()) > 0.0f) {
            system.update(dt);
        }
        
        const auto& p = system.particles()[0];
        if (p.position.y < 0.1f && i > 100) {  // Skip initial frames, check y ≈ 0
            landing_x = p.position.x;
            landed = true;
            break;
        }
    }
    
    REQUIRE(landed);
    
    // Expected range: v²sin(2θ)/g = 400*sin(90°)/9.80665 ≈ 40.77m
    float expected_range = (v0 * v0) / EARTH_GRAVITY;  // sin(2*45°) = 1
    
    // Tolerance: 5% (allows for numerical integration and timestep effects)
    REQUIRE_THAT(landing_x, WithinRel(expected_range, tolerance::ANALYTICAL_REL_5PCT));
}

// ============================================================================
// Test 5: Drag Deceleration
// ============================================================================

TEST_CASE("Physics Validation - Linear drag force", "[physics_validation]") {
    // Problem: Particle with initial velocity in drag field
    // F = -k*v, so dv/dt = -k*v => v(t) = v0*exp(-k*t)
    // With k=0.1: v(t) = v0*exp(-0.1*t)
    
    ParticleSystem system;
    TimestepController controller(1.0f / 60.0f);
    
    float v0 = 10.0f;
    float k = 0.1f;  // drag coefficient
    system.spawn(
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(v0, 0.0f, 0.0f),
        make_no_damping_material(1.0f)
    );
    system.add_force_field(std::make_unique<DragField>(k));
    
    // No gravity for this test
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, 0.0f, 0.0f)));
    
    // Simulate for varying times and check velocity decay
    float dt_frame = 0.016f;
    float max_velocity_error = 0.0f;
    
    for (int step = 0; step < 250; ++step) {
        controller.accumulate(dt_frame);
        float dt = 0.0f;
        while ((dt = controller.step()) > 0.0f) {
            system.update(dt);
        }
        
        // Check velocity at specific times
        if (step == 60 || step == 150) {  // t ≈ 1s and 2.4s
            const auto& p = system.particles()[0];
            float elapsed_time = static_cast<float>(step + 1) * dt_frame;
            float expected_v = v0 * std::exp(-k * elapsed_time);
            float error = std::abs(p.velocity.x - expected_v) / v0;
            max_velocity_error = std::max(max_velocity_error, error);
        }
    }
    
    // Tolerance: 5% for exponential decay
    REQUIRE(max_velocity_error < 0.05f);
}

// ============================================================================
// Test 6: Determinism
// ============================================================================

TEST_CASE("Physics Validation - Deterministic simulation reproducibility", "[physics_validation]") {
    // Problem: Identical simulation should produce bit-identical results
    // Run same scenario twice and compare final state
    
    auto run_simulation = []() -> ParticleSystem {
        ParticleSystem system;
        TimestepController controller(1.0f / 60.0f, 1.0f / 30.0f, TimestepController::OverflowMode::SUBDIVIDE);
        
        // Setup: 3 particles with various initial conditions
        system.spawn(Vec3f(-2.0f, 10.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), 1.0f);
        system.spawn(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 2.0f);
        system.spawn(Vec3f(2.0f, 10.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), 1.5f);
        
        system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f)));
        system.add_force_field(std::make_unique<DragField>(0.05f));
        
        // Simulate for 2 seconds
        float dt_frame = 0.016f;
        int frames = static_cast<int>(2.0f / dt_frame);
        
        for (int i = 0; i < frames; ++i) {
            controller.accumulate(dt_frame);
            float dt = 0.0f;
            while ((dt = controller.step()) > 0.0f) {
                system.update(dt);
            }
        }
        
        return system;
    };
    
    // Run simulation twice
    ParticleSystem sim1 = run_simulation();
    ParticleSystem sim2 = run_simulation();
    
    // Compare particle states
    const auto& particles1 = sim1.particles();
    const auto& particles2 = sim2.particles();
    
    REQUIRE(particles1.size() == particles2.size());
    
    for (size_t i = 0; i < particles1.size(); ++i) {
        const auto& p1 = particles1[i];
        const auto& p2 = particles2[i];
        
        // Positions should match exactly (deterministic)
        REQUIRE(approx_equal(p1.position, p2.position, 1e-10f));
        
        // Velocities should match exactly
        REQUIRE(approx_equal(p1.velocity, p2.velocity, 1e-10f));
    }
}

// ============================================================================
// Test 7: Multi-Particle Energy Conservation
// ============================================================================

TEST_CASE("Physics Validation - System energy conservation (multi-particle)", "[physics_validation]") {
    // Problem: Multiple particles in gravity field
    // Expected: Total system energy (KE + PE) remains constant
    
    ParticleSystem system;
    TimestepController controller(1.0f / 60.0f);
    
    // Spawn multiple particles with different masses and initial conditions
    system.spawn(
        Vec3f(-5.0f, 20.0f, 0.0f),
        Vec3f(5.0f, 10.0f, 0.0f),
        make_no_damping_material(1.0f)
    );
    system.spawn(
        Vec3f(0.0f, 30.0f, 0.0f),
        Vec3f(0.0f, 0.0f, 0.0f),
        make_no_damping_material(2.0f)
    );
    system.spawn(
        Vec3f(5.0f, 15.0f, 0.0f),
        Vec3f(-3.0f, 5.0f, 0.0f),
        make_no_damping_material(1.5f)
    );
    
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f)));
    
    // Calculate initial total energy
    float E_total_initial = 0.0f;
    for (const auto& p : system.particles()) {
        float KE = 0.5f * p.material.mass * (p.velocity.x * p.velocity.x + 
                                              p.velocity.y * p.velocity.y + 
                                              p.velocity.z * p.velocity.z);
        float PE = p.material.mass * EARTH_GRAVITY * p.position.y;
        E_total_initial += KE + PE;
    }
    
    // Simulate and track energy
    float dt_frame = 0.016f;
    int frames = static_cast<int>(3.0f / dt_frame);
    float max_energy_error = 0.0f;
    
    for (int i = 0; i < frames; ++i) {
        controller.accumulate(dt_frame);
        float dt = 0.0f;
        while ((dt = controller.step()) > 0.0f) {
            system.update(dt);
        }
        
        // Check energy conservation
        float E_total_current = 0.0f;
        for (const auto& p : system.particles()) {
            float KE = 0.5f * p.material.mass * (p.velocity.x * p.velocity.x + 
                                                  p.velocity.y * p.velocity.y + 
                                                  p.velocity.z * p.velocity.z);
            float PE = p.material.mass * EARTH_GRAVITY * p.position.y;
            E_total_current += KE + PE;
        }
        
        float error = std::abs(E_total_current - E_total_initial);
        max_energy_error = std::max(max_energy_error, error);
    }
    
    // Tolerance: 2% of initial total energy
    float tolerance = E_total_initial * 0.02f;
    REQUIRE(max_energy_error < tolerance);
}

// ============================================================================
// Test 8: Orbit Stability (Spring Field)
// ============================================================================

TEST_CASE("Physics Validation - Orbit stability in spring field", "[physics_validation]") {
    // Problem: Central spring force F = -k * r
    // For circular orbit: v = sqrt(k / m) * r

    ParticleSystem system;
    TimestepController controller(1.0f / 120.0f);

    const float mass = 1.0f;
    const float radius = 5.0f;
    const float spring_constant = 4.0f;
    const float omega = std::sqrt(spring_constant / mass);
    const float tangential_speed = omega * radius;

    system.spawn(
        Vec3f(radius, 0.0f, 0.0f),
        Vec3f(0.0f, 0.0f, tangential_speed),
        make_no_damping_material(mass)
    );
    system.add_force_field(std::make_unique<SpringField>(Vec3f(0.0f, 0.0f, 0.0f), spring_constant));

    float dt_frame = 1.0f / 120.0f;
    int frames = static_cast<int>(5.0f / dt_frame);
    float max_radius_error = 0.0f;

    for (int i = 0; i < frames; ++i) {
        controller.accumulate(dt_frame);
        float dt = 0.0f;
        while ((dt = controller.step()) > 0.0f) {
            system.update(dt);
        }

        const auto& p = system.particles()[0];
        float current_radius = p.position.length();
        float error = std::abs(current_radius - radius);
        max_radius_error = std::max(max_radius_error, error);
    }

    // Tolerance: 5% radius drift for numerical integration
    REQUIRE(max_radius_error < radius * 0.05f);
}

// ============================================================================
// Test 9: Simple Sphere Collision Response
// ============================================================================

TEST_CASE("Physics Validation - Simple elastic collision", "[physics_validation]") {
    // Problem: Two equal-mass spheres collide head-on
    // Expected: Velocities swap for restitution = 1

    ParticleSystem system;
    TimestepController controller(1.0f / 240.0f);

    const float radius = 0.5f;
    const float restitution = 1.0f;

    system.enable_collisions(true);
    system.set_default_collision_radius(radius);

    system.spawn(
        Vec3f(-1.0f, 0.0f, 0.0f),
        Vec3f(2.0f, 0.0f, 0.0f),
        make_no_damping_material(1.0f, restitution),
        -1.0f,
        radius
    );
    system.spawn(
        Vec3f(1.0f, 0.0f, 0.0f),
        Vec3f(-2.0f, 0.0f, 0.0f),
        make_no_damping_material(1.0f, restitution),
        -1.0f,
        radius
    );

    float dt_frame = 1.0f / 240.0f;
    int frames = static_cast<int>(2.0f / dt_frame);
    bool had_contact = false;
    float min_distance = std::numeric_limits<float>::max();

    for (int i = 0; i < frames; ++i) {
        controller.accumulate(dt_frame);
        float dt = 0.0f;
        while ((dt = controller.step()) > 0.0f) {
            system.update(dt);
        }

        const auto& particles = system.particles();
        const Vec3f delta = particles[1].position - particles[0].position;
        const float distance = delta.length();
        min_distance = std::min(min_distance, distance);

        if (distance <= 2.0f * radius + tolerance::POSITION) {
            had_contact = true;
        }
    }

    const auto& particles = system.particles();
    const Vec3f delta = particles[1].position - particles[0].position;
    const float final_distance = delta.length();
    const Vec3f normal = (final_distance > 1e-6f) ? (delta / final_distance) : Vec3f(1.0f, 0.0f, 0.0f);
    const float relative_velocity = (particles[1].velocity - particles[0].velocity).dot(normal);

    REQUIRE(had_contact);
    REQUIRE(min_distance <= 2.0f * radius + tolerance::POSITION);
    REQUIRE(final_distance >= 2.0f * radius - 0.05f);
    REQUIRE(relative_velocity >= -0.05f);
}


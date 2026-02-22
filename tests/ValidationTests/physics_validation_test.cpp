#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/particle_system.hpp>
#include <core/physics/timestep_controller.hpp>
#include <core/physics/material.hpp>
#include <core/physics/force_field.hpp>
#include <core/math/vectors/vec3.hpp>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using Catch::Approx;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
Material make_no_damping_material(float mass) {
    return Material(mass, 0.8f, 0.3f, 0.0f, 0.0f, 0.0f);
}
}  // namespace

// ============================================================================
// Physics Validation Tests - Phase 3
// ============================================================================
// These tests validate the particle system against known analytical solutions
// and conservation laws to ensure correctness of the physics simulation.

// ============================================================================
// Test 1: Free Fall Validation
// ============================================================================

TEST_CASE("Physics Validation - Free fall from rest", "[physics_validation]") {
    // Problem: Single particle dropped from y=100m with g=9.81 m/s²
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
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
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
    float expected_y = 100.0f - 0.5f * 9.81f * actual_time * actual_time;
    float expected_v = -9.81f * actual_time;
    
    // Tolerance: 1% for position, 2% for velocity (numerical integration)
    REQUIRE_THAT(p.position.y, WithinRel(expected_y, 0.01f));
    REQUIRE_THAT(p.velocity.y, WithinRel(expected_v, 0.02f));
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
    REQUIRE_THAT(final_momentum.x, WithinAbs(initial_momentum.x, 1e-5f));
    REQUIRE_THAT(final_momentum.y, WithinAbs(initial_momentum.y, 1e-5f));
    REQUIRE_THAT(final_momentum.z, WithinAbs(initial_momentum.z, 1e-5f));
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
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
    // Calculate initial total energy
    float KE_initial = 0.5f * mass * (20.0f * 20.0f);  // v_y = 20 m/s
    float PE_initial = mass * 9.81f * 50.0f;            // y = 50m
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
        float PE = mass * 9.81f * p.position.y;
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
    // Expected: Range ≈ v²sin(2θ)/g = 400*sin(90°)/9.81 ≈ 40.77m
    
    ParticleSystem system;
    TimestepController controller(1.0f / 60.0f);
    
    // Launch angle: 45 degrees
    float v0 = 20.0f;
    float angle = 3.14159265f / 4.0f;  // 45 degrees
    float vx = v0 * std::cos(angle);
    float vy = v0 * std::sin(angle);
    
    system.spawn(
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(vx, vy, 0.0f),
        make_no_damping_material(1.0f)
    );
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
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
    
    // Expected range: v²sin(2θ)/g = 400*sin(90°)/9.81 ≈ 40.77m
    float expected_range = (v0 * v0) / 9.81f;  // sin(2*45°) = 1
    
    // Tolerance: 5% (allows for numerical integration and timestep effects)
    REQUIRE_THAT(landing_x, WithinRel(expected_range, 0.05f));
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
        
        system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
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
        REQUIRE_THAT(p1.position.x, WithinAbs(p2.position.x, 1e-10f));
        REQUIRE_THAT(p1.position.y, WithinAbs(p2.position.y, 1e-10f));
        REQUIRE_THAT(p1.position.z, WithinAbs(p2.position.z, 1e-10f));
        
        // Velocities should match exactly
        REQUIRE_THAT(p1.velocity.x, WithinAbs(p2.velocity.x, 1e-10f));
        REQUIRE_THAT(p1.velocity.y, WithinAbs(p2.velocity.y, 1e-10f));
        REQUIRE_THAT(p1.velocity.z, WithinAbs(p2.velocity.z, 1e-10f));
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
    
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
    // Calculate initial total energy
    float E_total_initial = 0.0f;
    for (const auto& p : system.particles()) {
        float KE = 0.5f * p.material.mass * (p.velocity.x * p.velocity.x + 
                                              p.velocity.y * p.velocity.y + 
                                              p.velocity.z * p.velocity.z);
        float PE = p.material.mass * 9.81f * p.position.y;
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
            float PE = p.material.mass * 9.81f * p.position.y;
            E_total_current += KE + PE;
        }
        
        float error = std::abs(E_total_current - E_total_initial);
        max_energy_error = std::max(max_energy_error, error);
    }
    
    // Tolerance: 2% of initial total energy
    float tolerance = E_total_initial * 0.02f;
    REQUIRE(max_energy_error < tolerance);
}


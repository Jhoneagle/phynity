#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/particle.hpp>
#include <core/physics/material.hpp>
#include <cmath>

using phynity::physics::Particle;
using phynity::physics::Material;
using phynity::math::vectors::Vec3f;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_CASE("Particle: Default constructor", "[Particle][constructor]") {
    Particle p;
    
    REQUIRE_THAT(p.position.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p.position.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p.position.z, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p.velocity.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p.velocity.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p.velocity.z, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p.material.mass, WithinAbs(1.0f, 1e-6f));
    REQUIRE(p.active == true);
    REQUIRE(p.is_alive() == true);
}

TEST_CASE("Particle: Constructor with position", "[Particle][constructor]") {
    Vec3f pos(5.0f, 10.0f, -3.0f);
    Particle p(pos);
    
    REQUIRE_THAT(p.position.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(p.position.y, WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(p.position.z, WithinAbs(-3.0f, 1e-6f));
}

TEST_CASE("Particle: Constructor with position and velocity", "[Particle][constructor]") {
    Vec3f pos(1.0f, 2.0f, 3.0f);
    Vec3f vel(10.0f, 20.0f, 30.0f);
    Particle p(pos, vel);
    
    REQUIRE_THAT(p.position.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(p.velocity.x, WithinAbs(10.0f, 1e-6f));
}

TEST_CASE("Particle: Constructor with material", "[Particle][constructor]") {
    Material steel = phynity::physics::steel();
    Particle p(Vec3f(0.0f), Vec3f(0.0f), steel);
    
    REQUIRE_THAT(p.material.mass, WithinAbs(7850.0f, 1e-3f));
    REQUIRE_THAT(p.material.restitution, WithinAbs(0.3f, 1e-6f));
}

// ============================================================================
// Force Application Tests
// ============================================================================

TEST_CASE("Particle: Apply force accumulates", "[Particle][forces]") {
    Particle p;
    
    Vec3f f1(5.0f, 0.0f, 0.0f);
    Vec3f f2(3.0f, 0.0f, 0.0f);
    
    p.apply_force(f1);
    REQUIRE_THAT(p.force_accumulator.x, WithinAbs(5.0f, 1e-6f));
    
    p.apply_force(f2);
    REQUIRE_THAT(p.force_accumulator.x, WithinAbs(8.0f, 1e-6f));
}

TEST_CASE("Particle: Clear forces", "[Particle][forces]") {
    Particle p;
    
    p.apply_force(Vec3f(10.0f, 20.0f, 30.0f));
    REQUIRE(p.force_accumulator.length() > 0.0f);
    
    p.clear_forces();
    REQUIRE_THAT(p.force_accumulator.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Particle: Update acceleration from forces", "[Particle][forces]") {
    Particle p;
    p.material.mass = 2.0f;
    
    p.apply_force(Vec3f(10.0f, 0.0f, 0.0f));
    p.update_acceleration();
    
    // a = F / m = 10 / 2 = 5
    REQUIRE_THAT(p.acceleration.x, WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("Particle: Zero mass particle (kinematic)", "[Particle][forces]") {
    Particle p;
    p.material.mass = 0.0f;
    
    p.apply_force(Vec3f(100.0f, 0.0f, 0.0f));
    p.update_acceleration();
    
    // With zero mass, acceleration should be zero
    REQUIRE_THAT(p.acceleration.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Particle: Inverse mass calculation", "[Particle][forces]") {
    Particle p;
    p.material.mass = 4.0f;
    
    REQUIRE_THAT(p.inverse_mass(), WithinAbs(0.25f, 1e-6f));
}

TEST_CASE("Particle: Inverse mass with zero mass", "[Particle][forces]") {
    Particle p;
    p.material.mass = 0.0f;
    
    REQUIRE_THAT(p.inverse_mass(), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_CASE("Particle: Gravity integration", "[Particle][integration]") {
    Particle p;
    p.position = Vec3f(0.0f, 100.0f, 0.0f);
    p.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    p.material.mass = 1.0f;
    p.material.linear_damping = 0.0f;  // No damping for this test
    
    const float g = -9.81f;
    const float dt = 1.0f / 60.0f;
    
    p.apply_force(Vec3f(0.0f, g, 0.0f));
    p.update_acceleration();
    p.integrate(dt);
    
    // After one frame: v = a*dt = -9.81 * (1/60)
    float expected_vy = g * dt;
    REQUIRE_THAT(p.velocity.y, WithinAbs(expected_vy, 1e-6f));
    
    // Position updated: y = y0 + v_avg * dt ≈ y0 + v_final/2 * dt
    float expected_y = 100.0f + expected_vy * dt;
    REQUIRE_THAT(p.position.y, WithinAbs(expected_y, 1e-6f));
}

TEST_CASE("Particle: Velocity damping during integration", "[Particle][integration]") {
    Particle p;
    p.velocity = Vec3f(100.0f, 0.0f, 0.0f);
    p.material.linear_damping = 0.1f;
    
    const float dt = 1.0f / 60.0f;
    p.integrate(dt);
    
    // velocity *= (1 - damping * dt) = 100 * (1 - 0.1 * dt)
    float damping_factor = 1.0f - 0.1f * dt;
    float expected_vx = 100.0f * damping_factor;
    
    REQUIRE_THAT(p.velocity.x, WithinAbs(expected_vx, 1e-6f));
}

TEST_CASE("Particle: Position update from velocity", "[Particle][integration]") {
    Particle p;
    p.position = Vec3f(0.0f);
    p.velocity = Vec3f(10.0f, 0.0f, 0.0f);  // 10 m/s in x direction
    p.material.linear_damping = 0.0f;
    
    const float dt = 1.0f;  // 1 second
    p.integrate(dt);
    
    REQUIRE_THAT(p.position.x, WithinAbs(10.0f, 1e-6f));
}

TEST_CASE("Particle: Forces cleared after integration", "[Particle][integration]") {
    Particle p;
    p.apply_force(Vec3f(10.0f, 0.0f, 0.0f));
    
    p.integrate(0.016f);
    
    // Forces should be cleared after integration
    REQUIRE_THAT(p.force_accumulator.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Particle: Acceleration zeroed after integration", "[Particle][integration]") {
    Particle p;
    p.acceleration = Vec3f(10.0f, 0.0f, 0.0f);
    
    p.integrate(0.016f);
    
    // Acceleration should be cleared
    REQUIRE_THAT(p.acceleration.length(), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Lifetime Management Tests
// ============================================================================

TEST_CASE("Particle: Finite lifetime countdown", "[Particle][lifetime]") {
    Particle p;
    p.lifetime = 5.0f;
    
    REQUIRE(p.is_alive() == true);
    
    p.integrate(1.0f);
    REQUIRE_THAT(p.lifetime, WithinAbs(4.0f, 1e-6f));
    REQUIRE(p.is_alive() == true);
    
    p.integrate(4.0f);
    REQUIRE_THAT(p.lifetime, WithinAbs(0.0f, 1e-6f));
    REQUIRE(p.is_alive() == true);  // Still alive at lifetime=0
}

TEST_CASE("Particle: Infinite lifetime never expires", "[Particle][lifetime]") {
    Particle p;
    p.set_infinite_lifetime();
    
    REQUIRE(p.is_alive() == true);
    
    p.integrate(1.0f);
    p.integrate(1.0f);
    
    REQUIRE(p.is_alive() == true);
}

TEST_CASE("Particle: Set finite lifetime", "[Particle][lifetime]") {
    Particle p;
    p.set_lifetime(10.0f);
    
    REQUIRE_THAT(p.lifetime, WithinAbs(10.0f, 1e-6f));
    REQUIRE(p.is_alive() == true);
}

// ============================================================================
// Reset and Pooling Tests
// ============================================================================

TEST_CASE("Particle: Reset to initial state", "[Particle][pooling]") {
    Particle p;
    p.position = Vec3f(100.0f);
    p.velocity = Vec3f(50.0f);
    p.force_accumulator = Vec3f(10.0f);
    p.active = false;
    p.lifetime = 5.0f;
    
    Vec3f new_pos(1.0f, 2.0f, 3.0f);
    Material mat = phynity::physics::rubber();
    p.reset(new_pos, Vec3f(0.0f), mat);
    
    REQUIRE_THAT(p.position.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(p.velocity.length(), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(p.force_accumulator.length(), WithinAbs(0.0f, 1e-6f));
    REQUIRE(p.active == true);
    REQUIRE_THAT(p.material.mass, WithinAbs(1200.0f, 1e-3f));
}

// ============================================================================
// Accessor Tests
// ============================================================================

TEST_CASE("Particle: Speed calculation", "[Particle][accessors]") {
    Particle p;
    p.velocity = Vec3f(3.0f, 4.0f, 0.0f);  // Pythagorean triple: 3-4-5
    
    REQUIRE_THAT(p.speed(), WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("Particle: Kinetic energy calculation", "[Particle][accessors]") {
    Particle p;
    p.material.mass = 2.0f;
    p.velocity = Vec3f(10.0f, 0.0f, 0.0f);
    
    // KE = 0.5 * m * v^2 = 0.5 * 2 * 100 = 100
    REQUIRE_THAT(p.kinetic_energy(), WithinAbs(100.0f, 1e-6f));
}

TEST_CASE("Particle: Zero velocity kinetic energy", "[Particle][accessors]") {
    Particle p;
    p.material.mass = 5.0f;
    p.velocity = Vec3f(0.0f);
    
    REQUIRE_THAT(p.kinetic_energy(), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Backwards Compatibility Tests
// ============================================================================

TEST_CASE("Particle: Legacy applyForce method", "[Particle][legacy]") {
    Particle p;
    Vec3f force(5.0f, 10.0f, 15.0f);
    
    p.applyForce(force);
    REQUIRE_THAT(p.force_accumulator.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(p.force_accumulator.y, WithinAbs(10.0f, 1e-6f));
}

TEST_CASE("Particle: Legacy isAlive method", "[Particle][legacy]") {
    Particle p;
    p.lifetime = 5.0f;
    
    REQUIRE(p.isAlive() == true);
}

// ============================================================================
// Material Integration Tests
// ============================================================================

TEST_CASE("Particle: Different materials affect behavior", "[Particle][materials]") {
    Particle p1;
    p1.material = phynity::physics::steel();
    
    Particle p2;
    p2.material = phynity::physics::dust();
    
    REQUIRE(p1.material.mass > p2.material.mass);
    REQUIRE(p1.material.linear_damping < p2.material.linear_damping);
}

TEST_CASE("Particle: Material restitution accessible", "[Particle][materials]") {
    Particle p;
    p.material = phynity::physics::rubber();
    
    REQUIRE_THAT(p.material.restitution, WithinAbs(0.8f, 1e-6f));
}

// ============================================================================
// Combined Physics Tests
// ============================================================================

TEST_CASE("Particle: Apply gravity and integrate", "[Particle][integration][combined]") {
    Particle p;
    p.position = Vec3f(0.0f, 0.0f, 0.0f);
    p.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    p.material.mass = 1.0f;
    p.material.linear_damping = 0.0f;
    
    const float g = -9.81f;
    const float dt = 0.016f;  // 60 FPS
    
    // Step 1: Apply gravity
    p.apply_force(Vec3f(0.0f, g, 0.0f));
    p.update_acceleration();
    p.integrate(dt);
    
    REQUIRE(p.velocity.y < 0.0f);  // Moving downward
    REQUIRE(p.position.y < 0.0f);  // Below origin
}

TEST_CASE("Particle: Damping reduces velocity over time", "[Particle][integration][combined]") {
    Particle p;
    p.velocity = Vec3f(100.0f, 0.0f, 0.0f);
    p.material.linear_damping = 0.1f;  // 10% damping per second
    
    float v0 = p.velocity.x;
    
    // Integrate multiple steps
    for (int i = 0; i < 10; i++) {
        p.integrate(0.016f);
    }
    
    REQUIRE(p.velocity.x < v0);  // Velocity reduced
    REQUIRE(p.velocity.x > 0.0f);  // Still moving (not completely stopped)
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_CASE("Particle: Very small timestep", "[Particle][edge-cases]") {
    Particle p;
    p.apply_force(Vec3f(10.0f, 0.0f, 0.0f));
    p.update_acceleration();
    
    p.integrate(1e-9f);  // Nanosecond scale
    
    REQUIRE(p.position.x >= 0.0f);
    REQUIRE(p.is_alive() == true);
}

TEST_CASE("Particle: Large timestep", "[Particle][edge-cases]") {
    Particle p;
    p.velocity = Vec3f(1.0f, 0.0f, 0.0f);
    
    p.integrate(1000.0f);  // 1000 seconds
    
    REQUIRE_THAT(p.position.x, WithinAbs(1000.0f, 1e-3f));
}

TEST_CASE("Particle: Negative velocity", "[Particle][edge-cases]") {
    Particle p;
    p.velocity = Vec3f(-10.0f, 0.0f, 0.0f);
    
    p.integrate(1.0f);
    
    REQUIRE_THAT(p.position.x, WithinAbs(-10.0f, 1e-6f));
}

TEST_CASE("Particle: Very high mass", "[Particle][edge-cases]") {
    Particle p;
    p.material.mass = 1e6f;
    p.apply_force(Vec3f(1000.0f, 0.0f, 0.0f));
    p.update_acceleration();
    
    // Large mass -> small acceleration
    REQUIRE_THAT(p.acceleration.x, WithinAbs(0.001f, 1e-6f));
}


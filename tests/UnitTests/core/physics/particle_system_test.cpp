#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/particle_system.hpp>
#include <core/physics/force_field.hpp>
#include <core/physics/material.hpp>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Basic Particle Management Tests
// ============================================================================

TEST_CASE("ParticleSystem - Default construction", "[particle_system]") {
    ParticleSystem ps;
    
    REQUIRE(ps.particleCount() == 0);
    REQUIRE(ps.particles().empty());
    REQUIRE(ps.force_field_count() == 0);
}

TEST_CASE("ParticleSystem - Spawn particles", "[particle_system]") {
    ParticleSystem ps;
    
    SECTION("Spawn single particle") {
        ps.spawn(Vec3f(1.0f, 2.0f, 3.0f), Vec3f(0.5f, 0.0f, 0.0f));
        
        REQUIRE(ps.particleCount() == 1);
        REQUIRE_THAT(ps.particles()[0].position.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(ps.particles()[0].position.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(ps.particles()[0].position.z, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(ps.particles()[0].velocity.x, WithinAbs(0.5f, 1e-6f));
    }
    
    SECTION("Spawn multiple particles") {
        ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f));
        ps.spawn(Vec3f(1.0f, 1.0f, 1.0f), Vec3f(0.0f, 1.0f, 0.0f));
        ps.spawn(Vec3f(2.0f, 2.0f, 2.0f), Vec3f(0.0f, 0.0f, 1.0f));
        
        REQUIRE(ps.particleCount() == 3);
    }
    
    SECTION("Spawn with custom mass") {
        ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 5.0f);
        
        REQUIRE_THAT(ps.particles()[0].material.mass, WithinAbs(5.0f, 1e-6f));
    }
    
    SECTION("Spawn with finite lifetime") {
        ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f, 2.0f);
        
        REQUIRE_THAT(ps.particles()[0].lifetime, WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("ParticleSystem - Spawn with Material", "[particle_system]") {
    ParticleSystem ps;
    Material rubber_mat = rubber();
    
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), rubber_mat);
    
    REQUIRE(ps.particleCount() == 1);
    REQUIRE_THAT(ps.particles()[0].material.mass, WithinAbs(rubber_mat.mass, 1e-6f));
    REQUIRE_THAT(ps.particles()[0].material.restitution, WithinAbs(rubber_mat.restitution, 1e-6f));
    REQUIRE_THAT(ps.particles()[0].material.linear_damping, WithinAbs(rubber_mat.linear_damping, 1e-6f));
}

TEST_CASE("ParticleSystem - Clear particles", "[particle_system]") {
    ParticleSystem ps;
    
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f));
    ps.spawn(Vec3f(1.0f, 1.0f, 1.0f), Vec3f(0.0f, 0.0f, 0.0f));
    ps.spawn(Vec3f(2.0f, 2.0f, 2.0f), Vec3f(0.0f, 0.0f, 0.0f));
    
    REQUIRE(ps.particleCount() == 3);
    
    ps.clear();
    
    REQUIRE(ps.particleCount() == 0);
    REQUIRE(ps.particles().empty());
}

TEST_CASE("ParticleSystem - Remove dead particles", "[particle_system]") {
    ParticleSystem ps;
    
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f, -1.0f);  // Infinite
    ps.spawn(Vec3f(1.0f, 1.0f, 1.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f, 1.0f);   // 1 second
    ps.spawn(Vec3f(2.0f, 2.0f, 2.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f, 0.5f);   // 0.5 second
    
    REQUIRE(ps.particleCount() == 3);
    
    // Step by 0.6 seconds - should kill the 0.5s particle
    ps.update(0.6f);
    
    REQUIRE(ps.particleCount() == 2);
}

// ============================================================================
// Force Field Management Tests
// ============================================================================

TEST_CASE("ParticleSystem - Add force fields", "[particle_system]") {
    ParticleSystem ps;
    
    SECTION("Add single force field") {
        ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
        
        REQUIRE(ps.force_field_count() == 1);
    }
    
    SECTION("Add multiple force fields") {
        ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
        ps.add_force_field(std::make_unique<DragField>(0.1f));
        
        REQUIRE(ps.force_field_count() == 2);
    }
}

TEST_CASE("ParticleSystem - Clear force fields", "[particle_system]") {
    ParticleSystem ps;
    
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    ps.add_force_field(std::make_unique<DragField>(0.1f));
    
    REQUIRE(ps.force_field_count() == 2);
    
    ps.clear_force_fields();
    
    REQUIRE(ps.force_field_count() == 0);
}

// ============================================================================
// Simulation Update Tests
// ============================================================================

TEST_CASE("ParticleSystem - Update with no forces", "[particle_system]") {
    ParticleSystem ps;
    ps.spawn(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f));
    
    ps.update(0.1f);
    
    // No forces, no motion
    REQUIRE_THAT(ps.particles()[0].position.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(ps.particles()[0].position.y, WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(ps.particles()[0].position.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("ParticleSystem - Update with gravity", "[particle_system]") {
    ParticleSystem ps;
    Material zero_damp;
    zero_damp.mass = 1.0f;
    zero_damp.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), zero_damp);
    
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -10.0f, 0.0f)));
    
    ps.update(0.1f);
    
    // v = a*t = -10 * 0.1 = -1 m/s
    // y = y0 + v*t = 10 + (-1)*0.1 = 9.9 m
    REQUIRE_THAT(ps.particles()[0].velocity.y, WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(ps.particles()[0].position.y, WithinAbs(9.9f, 1e-5f));
}

TEST_CASE("ParticleSystem - Update with multiple force fields", "[particle_system]") {
    ParticleSystem ps;
    Material zero_damp;
    zero_damp.mass = 2.0f;
    zero_damp.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), zero_damp);
    
    // Two gravity fields should combine
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -5.0f, 0.0f)));
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -5.0f, 0.0f)));
    
    ps.update(0.1f);
    
    // Combined gravity: -10 m/s^2
    // v = a*t = -10 * 0.1 = -1 m/s
    REQUIRE_THAT(ps.particles()[0].velocity.y, WithinAbs(-1.0f, 1e-6f));
}

TEST_CASE("ParticleSystem - Update with drag", "[particle_system]") {
    ParticleSystem ps;
    Material zero_damp;
    zero_damp.mass = 1.0f;
    zero_damp.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), zero_damp);
    
    ps.add_force_field(std::make_unique<DragField>(1.0f));
    
    // Drag applies F = -drag_coefficient * velocity
    // F = -1.0 * 10.0 = -10 N
    // a = F/m = -10 / 1.0 = -10 m/s^2
    ps.update(0.1f);
    
    // Initial velocity was 10 m/s
    // Acceleration is -10 m/s^2
    // v_new = v_old + a*dt = 10 + (-10)*0.1 = 9 m/s
    REQUIRE_THAT(ps.particles()[0].velocity.x, WithinAbs(9.0f, 1e-5f));
}

TEST_CASE("ParticleSystem - Multiple particles with forces", "[particle_system]") {
    ParticleSystem ps;
    Material zero_damp1, zero_damp2;
    zero_damp1.mass = 1.0f;
    zero_damp1.linear_damping = 0.0f;
    zero_damp2.mass = 2.0f;
    zero_damp2.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), zero_damp1);
    ps.spawn(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), zero_damp2);
    
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -10.0f, 0.0f)));
    
    ps.update(0.1f);
    
    // Both particles should fall, same acceleration (gravity is mass-independent)
    REQUIRE_THAT(ps.particles()[0].velocity.y, WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(ps.particles()[1].velocity.y, WithinAbs(-1.0f, 1e-6f));
}

TEST_CASE("ParticleSystem - Legacy step method", "[particle_system]") {
    ParticleSystem ps;
    Material zero_damp;
    zero_damp.mass = 1.0f;
    zero_damp.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), zero_damp);
    
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -10.0f, 0.0f)));
    
    ps.step(0.1f);  // Should behave same as update()
    
    REQUIRE_THAT(ps.particles()[0].velocity.y, WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(ps.particles()[0].position.y, WithinAbs(9.9f, 1e-5f));
}

TEST_CASE("ParticleSystem - Legacy applyGravity method", "[particle_system]") {
    ParticleSystem ps;
    Material zero_damp;
    zero_damp.mass = 1.0f;
    zero_damp.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), zero_damp);
    
    ps.applyGravity(Vec3f(0.0f, -10.0f, 0.0f));
    
    // Forces applied, now update acceleration and integrate
    ps.particles()[0].update_acceleration();
    ps.particles()[0].integrate(0.1f);
    
    REQUIRE_THAT(ps.particles()[0].velocity.y, WithinAbs(-1.0f, 1e-6f));
}

// ============================================================================
// Diagnostics Tests
// ============================================================================

TEST_CASE("ParticleSystem - Diagnostics with no particles", "[particle_system]") {
    ParticleSystem ps;
    
    auto diag = ps.compute_diagnostics();
    
    REQUIRE(diag.particle_count == 0);
    REQUIRE_THAT(diag.total_kinetic_energy, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(diag.total_momentum.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(diag.total_momentum.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(diag.total_momentum.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("ParticleSystem - Diagnostics kinetic energy", "[particle_system]") {
    ParticleSystem ps;
    
    // KE = 0.5 * m * v^2
    // KE = 0.5 * 2.0 * (3^2) = 0.5 * 2.0 * 9 = 9.0 J
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(3.0f, 0.0f, 0.0f), 2.0f);
    
    auto diag = ps.compute_diagnostics();
    
    REQUIRE(diag.particle_count == 1);
    REQUIRE_THAT(diag.total_kinetic_energy, WithinAbs(9.0f, 1e-5f));
}

TEST_CASE("ParticleSystem - Diagnostics momentum", "[particle_system]") {
    ParticleSystem ps;
    
    // p = m * v = 2.0 * 5.0 = 10.0 kg*m/s
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f), 2.0f);
    
    auto diag = ps.compute_diagnostics();
    
    REQUIRE(diag.particle_count == 1);
    REQUIRE_THAT(diag.total_momentum.x, WithinAbs(10.0f, 1e-5f));
    REQUIRE_THAT(diag.total_momentum.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(diag.total_momentum.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("ParticleSystem - Diagnostics with multiple particles", "[particle_system]") {
    ParticleSystem ps;
    
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), 1.0f);  // KE = 0.5*1*4 = 2, p = 2
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 3.0f, 0.0f), 2.0f);  // KE = 0.5*2*9 = 9, p = 6
    
    auto diag = ps.compute_diagnostics();
    
    REQUIRE(diag.particle_count == 2);
    REQUIRE_THAT(diag.total_kinetic_energy, WithinAbs(11.0f, 1e-5f));
    REQUIRE_THAT(diag.total_momentum.x, WithinAbs(2.0f, 1e-5f));
    REQUIRE_THAT(diag.total_momentum.y, WithinAbs(6.0f, 1e-5f));
}

// ============================================================================
// Energy/Momentum Conservation Tests
// ============================================================================

TEST_CASE("ParticleSystem - Energy conservation in free space", "[particle_system]") {
    ParticleSystem ps;
    Material zero_damp;
    zero_damp.mass = 1.0f;
    zero_damp.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(5.0f, 3.0f, 0.0f), zero_damp);
    
    auto initial_diag = ps.compute_diagnostics();
    float initial_energy = initial_diag.total_kinetic_energy;
    
    // Update without forces - energy should be conserved (within numerical error)
    for (int i = 0; i < 10; ++i) {
        ps.update(0.01f);
    }
    
    auto final_diag = ps.compute_diagnostics();
    float final_energy = final_diag.total_kinetic_energy;
    
    // Energy should be conserved (within small numerical tolerance)
    REQUIRE_THAT(final_energy, WithinAbs(initial_energy, 1e-4f));
}

TEST_CASE("ParticleSystem - Momentum conservation in free space", "[particle_system]") {
    ParticleSystem ps;
    Material zero_damp;
    zero_damp.mass = 1.0f;
    zero_damp.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(5.0f, 3.0f, 2.0f), zero_damp);
    
    auto initial_diag = ps.compute_diagnostics();
    Vec3f initial_momentum = initial_diag.total_momentum;
    
    // Update without forces - momentum should be conserved
    for (int i = 0; i < 10; ++i) {
        ps.update(0.01f);
    }
    
    auto final_diag = ps.compute_diagnostics();
    Vec3f final_momentum = final_diag.total_momentum;
    
    REQUIRE_THAT(final_momentum.x, WithinAbs(initial_momentum.x, 1e-4f));
    REQUIRE_THAT(final_momentum.y, WithinAbs(initial_momentum.y, 1e-4f));
    REQUIRE_THAT(final_momentum.z, WithinAbs(initial_momentum.z, 1e-4f));
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_CASE("ParticleSystem - Free fall simulation", "[particle_system][integration]") {
    ParticleSystem ps;
    ps.spawn(Vec3f(0.0f, 100.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
    // Simulate for 1 second with small timesteps
    const float dt = 0.01f;
    for (int i = 0; i < 100; ++i) {
        ps.update(dt);
    }
    
    // After 1 second:
    // v = g*t = -9.81 m/s
    // y = y0 + 0.5*g*t^2 = 100 - 0.5*9.81*1 = 100 - 4.905 ≈ 95.095 m
    REQUIRE_THAT(ps.particles()[0].velocity.y, WithinAbs(-9.81f, 0.1f));
    REQUIRE_THAT(ps.particles()[0].position.y, WithinAbs(95.095f, 0.1f));
}

TEST_CASE("ParticleSystem - Projectile motion", "[particle_system][integration]") {
    ParticleSystem ps;
    Material zero_damp;
    zero_damp.mass = 1.0f;
    zero_damp.linear_damping = 0.0f;
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(10.0f, 10.0f, 0.0f), zero_damp);
    
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -10.0f, 0.0f)));
    
    // Simulate for 1 second
    const float dt = 0.01f;
    for (int i = 0; i < 100; ++i) {
        ps.update(dt);
    }
    
    // After 1 second:
    // vx = 10 m/s (constant)
    // vy = 10 - 10*1 = 0 m/s
    // x = 10*1 = 10 m
    // y = 10*1 - 0.5*10*1 = 10 - 5 = 5 m
    REQUIRE_THAT(ps.particles()[0].velocity.x, WithinAbs(10.0f, 0.05f));
    REQUIRE_THAT(ps.particles()[0].velocity.y, WithinAbs(0.0f, 0.1f));
    REQUIRE_THAT(ps.particles()[0].position.x, WithinAbs(10.0f, 0.1f));
    REQUIRE_THAT(ps.particles()[0].position.y, WithinAbs(5.0f, 0.1f));
}

TEST_CASE("ParticleSystem - Terminal velocity with drag", "[particle_system][integration]") {
    ParticleSystem ps;
    ps.spawn(Vec3f(0.0f, 100.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    
    // Gravity -10 m/s^2, drag coefficient 1.0
    // Terminal velocity when -drag*v = -m*g => v = m*g/drag = 1*10/1 = 10 m/s
    ps.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -10.0f, 0.0f)));
    ps.add_force_field(std::make_unique<DragField>(1.0f));
    
    // Simulate for several seconds
    const float dt = 0.01f;
    for (int i = 0; i < 500; ++i) {
        ps.update(dt);
    }
    
    // Should approach terminal velocity of 10 m/s downward
    // Note: sign is negative because falling down
    REQUIRE_THAT(ps.particles()[0].velocity.y, WithinAbs(-10.0f, 0.5f));
}

// ============================================================================
// Material Integration Tests
// ============================================================================

TEST_CASE("ParticleSystem - Damping from material", "[particle_system][material]") {
    ParticleSystem ps;
    
    Material damped;
    damped.mass = 1.0f;
    damped.linear_damping = 1.0f;  // High damping
    
    ps.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), damped);
    
    // Simulate with no forces - velocity should decay due to damping
    for (int i = 0; i < 10; ++i) {
        ps.update(0.1f);
    }
    
    // After 1 second with damping factor = 1.0 * dt = 0.1 per step
    // Velocity should be significantly reduced
    REQUIRE(ps.particles()[0].velocity.x < 5.0f);
}

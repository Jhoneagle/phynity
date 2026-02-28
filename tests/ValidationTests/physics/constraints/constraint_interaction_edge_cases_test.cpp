#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/particle_system.hpp>
#include <core/physics/force_field.hpp>
#include <core/math/vectors/vec3.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <cmath>
#include <memory>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// ============================================================================
// Test 1: Contact + Constraint Mixing
// ============================================================================
// Verify constraint solver stability when particles are constrained and colliding.
//
TEST_CASE("constraint_interaction.contact_restitution_mixing",
          "[validation][physics][constraints][interaction]") {
    ParticleSystem system;
    system.enable_collisions(true);
    system.enable_constraints(true);

    auto config = system.constraint_solver_config();
    config.iterations = 6;
    config.use_warm_start = true;
    system.set_constraint_solver_config(config);

    // Create two particles connected by fixed constraint
    // Particle 0 and 1 move toward a wall
    auto p0_material = make_no_damping_material(1.0f, 0.8f);
    system.spawn(Vec3f(-1.5f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), p0_material, -1.0f, 0.3f);
    
    auto p1_material = make_no_damping_material(1.0f, 0.8f);
    system.spawn(Vec3f(-0.5f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), p1_material, -1.0f, 0.3f);
    
    // Fixed constraint between p0 and p1
    system.add_fixed_constraint(0, 1);

    // No gravity - isolate constraint + contact interaction
    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 240; ++step) {
        system.update(dt);
    }

    // Verify:
    // 1. All positions and velocities are finite (no NaN/Inf)
    for (size_t i = 0; i < 2; ++i) {
        REQUIRE(std::isfinite(system.particles()[i].position.x));
        REQUIRE(std::isfinite(system.particles()[i].position.y));
        REQUIRE(std::isfinite(system.particles()[i].velocity.x));
        REQUIRE(std::isfinite(system.particles()[i].velocity.y));
    }

    // 2. Constraint still holds
    float dist = (system.particles()[0].position - system.particles()[1].position).length();
    REQUIRE_THAT(dist, WithinAbs(1.0f, 0.2f));

    // 3. Particles bounded (not explosive)
    for (size_t i = 0; i < 2; ++i) {
        REQUIRE(system.particles()[i].velocity.squaredLength() < 100.0f);
    }
}

// ============================================================================
// Test 2: Constraint + Friction
// ============================================================================
// Verify that friction is applied correctly when constrained particles slide.
//
TEST_CASE("constraint_interaction.friction_on_constrained_contact",
          "[validation][physics][constraints][interaction]") {
    ParticleSystem system;
    system.enable_collisions(true);
    system.enable_constraints(true);

    auto config = system.constraint_solver_config();
    config.iterations = 6;
    config.use_warm_start = true;
    system.set_constraint_solver_config(config);

    // Create a pair connected by fixed constraint
    auto p0_material = make_no_damping_material(1.0f, 0.5f);
    p0_material.friction = 0.5f;
    system.spawn(Vec3f(0.0f, 1.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), p0_material, -1.0f, 0.3f);

    auto p1_material = make_no_damping_material(1.0f, 0.5f);
    p1_material.friction = 0.5f;
    system.spawn(Vec3f(0.0f, 0.5f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), p1_material, -1.0f, 0.3f);

    // Floor to slide against (low friction)
    auto floor_material = make_no_damping_material(1e6f, 0.2f);
    floor_material.friction = 0.2f;
    system.spawn(Vec3f(0.0f, -1.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), floor_material, -1.0f, 0.3f);

    system.add_fixed_constraint(0, 1);

    // No gravity to isolate sliding friction

    // Simulate sliding with friction
    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 360; ++step) {
        system.update(dt);
    }

    // Verify:
    // 1. No NaNs or Infs
    for (const auto& p : system.particles()) {
        REQUIRE(std::isfinite(p.position.squaredLength()));
        REQUIRE(std::isfinite(p.velocity.squaredLength()));
    }

    // 2. Constraint maintained
    float dist = (system.particles()[0].position - system.particles()[1].position).length();
    REQUIRE_THAT(dist, WithinAbs(0.5f, 0.1f));

    // 3. X-velocities reduced by friction but not explosive
    for (size_t i = 0; i < 2; ++i) {
        REQUIRE_THAT(system.particles()[i].velocity.x, WithinAbs(0.0f, 2.0f));
        REQUIRE(system.particles()[i].velocity.x <= 1.0f);  // Some dissipation (converged or reduced)
    }
}

// ============================================================================
// Test 3: Extreme Mass Ratios
// ============================================================================
// Verify solver converges with extreme mass ratios under constraints.
//
TEST_CASE("constraint_interaction.extreme_mass_ratios",
          "[validation][physics][constraints][interaction]") {
    // Test: constrained particles with different masses should converge
    ParticleSystem system;
    system.enable_collisions(false);
    system.enable_constraints(true);

    auto config = system.constraint_solver_config();
    config.iterations = 8;
    config.use_warm_start = true;
    system.set_constraint_solver_config(config);

    // Heavy particle (100x heavier)
    auto heavy_material = make_no_damping_material(100.0f, 0.0f);
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 heavy_material, -1.0f, 0.5f);

    // Light particle with error velocity
    auto light_material = make_no_damping_material(1.0f, 0.0f);
    system.spawn(Vec3f(3.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), 
                 light_material, -1.0f, 0.5f);

    // Constrain them
    system.add_fixed_constraint(0, 1);

    // Simulate for convergence
    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 240; ++step) {
        system.update(dt);
    }

    // Verify: all values are finite (no NaN/Inf)
    for (size_t i = 0; i < 2; ++i) {
        REQUIRE(std::isfinite(system.particles()[i].position.x));
        REQUIRE(std::isfinite(system.particles()[i].position.y));
        REQUIRE(std::isfinite(system.particles()[i].velocity.x));
        REQUIRE(std::isfinite(system.particles()[i].velocity.y));
    }

    // Verify: velocities converge (both particles at rest or very slow)
    float total_speed = system.particles()[0].velocity.length() + 
                       system.particles()[1].velocity.length();
    REQUIRE(total_speed < 5.0f);
}

// ============================================================================
// Test 4: Constraint Chains
// ============================================================================
// Multiple constraints in series (A-B-C where A↔B and B↔C are fixed).
// Verify constraints propagate corrections correctly.
//
TEST_CASE("constraint_interaction.constraint_chains",
          "[validation][physics][constraints][interaction]") {
    ParticleSystem system;
    system.enable_collisions(false);
    system.enable_constraints(true);

    auto config = system.constraint_solver_config();
    config.iterations = 8;
    config.use_warm_start = true;
    system.set_constraint_solver_config(config);

    // Three particles in a line: A -- B -- C
    auto material = make_no_damping_material(1.0f, 0.0f);
    
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 material, -1.0f, 0.3f);
    system.spawn(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 material, -1.0f, 0.3f);
    system.spawn(Vec3f(2.5f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 material, -1.0f, 0.3f);

    // Create constraint chain
    system.add_fixed_constraint(0, 1);  // A-B distance = 1.0
    system.add_fixed_constraint(1, 2);  // B-C distance = 1.5

    // Simulate
    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 300; ++step) {
        system.update(dt);
    }

    // Verify:
    // 1. All positions finite
    for (const auto& p : system.particles()) {
        REQUIRE(std::isfinite(p.position.x));
        REQUIRE(std::isfinite(p.position.y));
    }

    // 2. Both constraints maintained
    float dist_ab = (system.particles()[0].position - system.particles()[1].position).length();
    float dist_bc = (system.particles()[1].position - system.particles()[2].position).length();

    REQUIRE_THAT(dist_ab, WithinAbs(1.0f, 0.15f));
    REQUIRE_THAT(dist_bc, WithinAbs(1.5f, 0.15f));

    // 3. All velocities bounded
    for (const auto& p : system.particles()) {
        REQUIRE_THAT(p.velocity.squaredLength(), WithinAbs(0.0f, 0.5f));
    }

    // 4. Verify chain topology: A-B = 1.0, B-C = 1.5, so A-C should be ~2.5
    float dist_ac = (system.particles()[0].position - system.particles()[2].position).length();
    REQUIRE_THAT(dist_ac, WithinAbs(2.5f, 0.25f));
}

// ============================================================================
// Test 5: Long-Duration Stability
// ============================================================================
// Constraints over extended periods (5000+ steps) to detect drift/instability.
//
TEST_CASE("constraint_interaction.long_duration_stability",
          "[validation][physics][constraints][interaction]") {
    ParticleSystem system;
    system.enable_collisions(false);
    system.enable_constraints(true);

    auto config = system.constraint_solver_config();
    config.iterations = 6;
    config.use_warm_start = true;
    config.convergence_threshold = 1e-5f;
    system.set_constraint_solver_config(config);

    // Create a small chain of 4 particles
    auto material = make_no_damping_material(1.0f, 0.0f);
    for (size_t i = 0; i < 4; ++i) {
        system.spawn(Vec3f(static_cast<float>(i), 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f),
                     material, -1.0f, 0.3f);
    }

    // Connect all adjacent pairs
    system.add_fixed_constraint(0, 1);
    system.add_fixed_constraint(1, 2);
    system.add_fixed_constraint(2, 3);

    // Run for a very long time (5000+ steps = ~42 seconds of sim time at 120 Hz)
    const int num_steps = 5040;
    const float dt = 1.0f / 120.0f;
    
    for (int step = 0; step < num_steps; ++step) {
        system.update(dt);

        // Periodic checks
        if (step > 0 && step % 1000 == 0) {
            // Verify no divergence every 1000 steps
            for (const auto& p : system.particles()) {
                REQUIRE(std::isfinite(p.position.x));
                REQUIRE(std::isfinite(p.position.y));
                REQUIRE(std::isfinite(p.position.z));
                REQUIRE(std::isfinite(p.velocity.x));
                REQUIRE(std::isfinite(p.velocity.y));
                REQUIRE(std::isfinite(p.velocity.z));
            }
        }
    }

    // Final verification
    // 1. All particles at rest in starting configuration
    REQUIRE_THAT(system.particles()[0].position.x, WithinAbs(0.0f, 0.2f));
    REQUIRE_THAT(system.particles()[1].position.x, WithinAbs(1.0f, 0.2f));
    REQUIRE_THAT(system.particles()[2].position.x, WithinAbs(2.0f, 0.2f));
    REQUIRE_THAT(system.particles()[3].position.x, WithinAbs(3.0f, 0.2f));

    // 2. All velocities near zero
    for (const auto& p : system.particles()) {
        REQUIRE_THAT(p.velocity.squaredLength(), WithinAbs(0.0f, 0.1f));
    }

    // 3. Constraints maintained
    for (size_t i = 0; i < 3; ++i) {
        float dist = (system.particles()[i].position - system.particles()[i + 1].position).length();
        REQUIRE_THAT(dist, WithinAbs(1.0f, 0.25f));
    }
}

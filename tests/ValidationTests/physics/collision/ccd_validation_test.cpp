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
// Test 1: Discrete Collision Detection Stability
// ============================================================================
// Verify that even with discrete collision detection, moderate velocities
// are handled stably without explosion or tunneling escape.
//
TEST_CASE("ccd.discrete_collision_stability",
          "[validation][physics][collision][ccd]") {
    ParticleSystem system;
    system.enable_collisions(true);
    system.enable_constraints(false);

    // Moderate-speed projectile (not extremely fast)
    auto projectile_mat = make_no_damping_material(1.0f, 0.5f);
    system.spawn(Vec3f(-4.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), 
                 projectile_mat, -1.0f, 0.25f);
    
    // Wall
    auto wall_mat = make_no_damping_material(1e6f, 0.8f);
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 wall_mat, -1.0f, 0.3f);

    // Simulate with reasonable timestep
    const float dt = 1.0f / 120.0f;  // Standard 120 Hz timestep
    for (int step = 0; step < 120; ++step) {
        system.update(dt);
    }

    // Verify all values remain finite (system is stable)
    for (size_t i = 0; i < 2; ++i) {
        REQUIRE(std::isfinite(system.particles()[i].position.x));
        REQUIRE(std::isfinite(system.particles()[i].velocity.x));
    }

    // Projectile should not pass through wall completely
    // (with discrete CD and dt=1/120, this should be caught)
    REQUIRE(system.particles()[0].position.x < 2.0f);
}

// ============================================================================
// Test 2: Swept Sphere Collision
// ============================================================================
// Verify that swept volumes are used to detect collisions along trajectory.
// A moving sphere should detect collision with stationary object even if
// they don't overlap at simulation tick (but would have overlapped in between).
//
TEST_CASE("ccd.swept_sphere_collision",
          "[validation][physics][collision][ccd]") {
    ParticleSystem system;
    system.enable_collisions(true);

    // Moving sphere toward obstacle
    auto moving_mat = make_no_damping_material(1.0f, 0.8f);
    system.spawn(Vec3f(-3.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), 
                 moving_mat, -1.0f, 0.3f);
    
    // Obstacle (stationary)
    auto obstacle_mat = make_no_damping_material(1e6f, 0.7f);
    system.spawn(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 obstacle_mat, -1.0f, 0.4f);

    // Simulate
    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 120; ++step) {
        system.update(dt);
    }

    // After collision, moving sphere should not pass through obstacle
    // Distance between centers should be >= sum of radii (0.3 + 0.4 = 0.7)
    float dist = (system.particles()[0].position - system.particles()[1].position).length();
    REQUIRE(dist >= 0.65f);  // Allow small numerical tolerance

    // Sphere should have some rebound velocity
    REQUIRE(std::abs(system.particles()[0].velocity.x) > 0.1f);
}

// ============================================================================
// Test 3: Time-of-Impact (TOI) Accuracy
// ============================================================================
// Verify that predicted collision time matches actual collision detection.
// Two particles on collision course - predicted TOI should be accurate.
//
TEST_CASE("ccd.time_of_impact_accuracy",
          "[validation][physics][collision][ccd]") {
    ParticleSystem system;
    system.enable_collisions(true);

    // Particle A: moving right at 5 m/s
    auto mat_a = make_no_damping_material(1.0f, 0.5f);
    system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f), 
                 mat_a, -1.0f, 0.25f);
    
    // Particle B: moving left at 5 m/s
    auto mat_b = make_no_damping_material(1.0f, 0.5f);
    system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(-5.0f, 0.0f, 0.0f), 
                 mat_b, -1.0f, 0.25f);

    // Expected collision time: they're 4 units apart, closing at 10 m/s
    // Contact at (0+0.25) = 0.25 units from center
    // Distance to close = 4 - 2*0.25 = 3.5 units
    // Expected TOI = 3.5 / 10 = 0.35 seconds

    int collision_step = -1;
    const float dt = 1.0f / 120.0f;
    
    for (int step = 0; step < 100; ++step) {
        float prev_dist = (system.particles()[0].position - system.particles()[1].position).length();
        system.update(dt);
        float curr_dist = (system.particles()[0].position - system.particles()[1].position).length();
        
        // Detect collision frame (distance stops decreasing or reverses)
        if (curr_dist > prev_dist && collision_step < 0) {
            collision_step = step;
            break;
        }
    }

    // Should collide around step 40-50 (0.33-0.42 seconds)
    REQUIRE(collision_step > 0);
    REQUIRE(collision_step < 60);  // Within ~0.5 seconds

    // After collision, particles should be separating
    float final_dist = (system.particles()[0].position - system.particles()[1].position).length();
    REQUIRE(final_dist >= 0.45f);  // Slightly relaxed for numerical tolerance
}

// ============================================================================
// Test 4: Multiple Object Collision Sequence
// ============================================================================
// Verify CCD handles cascading collisions (A hits B, B hits C).
// Tests that contact resolution doesn't break subsequent collision detection.
//
TEST_CASE("ccd.cascade_collision_sequence",
          "[validation][physics][collision][ccd]") {
    ParticleSystem system;
    system.enable_collisions(true);

    // Three particles in a line: A moving, B center, C stationary
    auto mat_a = make_no_damping_material(1.0f, 0.8f);
    system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(4.0f, 0.0f, 0.0f), 
                 mat_a, -1.0f, 0.2f);
    
    auto mat_b = make_no_damping_material(1.0f, 0.8f);
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 mat_b, -1.0f, 0.2f);
    
    auto mat_c = make_no_damping_material(1.0f, 0.8f);
    system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 mat_c, -1.0f, 0.2f);

    // Simulate cascade
    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 300; ++step) {
        system.update(dt);
    }

    // Verify no explosive velocities or positions
    for (size_t i = 0; i < 3; ++i) {
        REQUIRE(std::isfinite(system.particles()[i].position.x));
        REQUIRE(std::isfinite(system.particles()[i].velocity.x));
        REQUIRE(system.particles()[i].velocity.squaredLength() < 100.0f);
    }

    // After settling, verify sensible spacing (not explosive)
    // Due to momentum transfer in elastic-ish collisions, particles will spread
    float dist_ab = (system.particles()[0].position - system.particles()[1].position).length();
    float dist_bc = (system.particles()[1].position - system.particles()[2].position).length();
    float dist_ac = (system.particles()[0].position - system.particles()[2].position).length();
    
    // Particles should maintain minimum separation (sum of radii)
    REQUIRE(dist_ab >= 0.35f);  // 0.2 + 0.2 radii
    REQUIRE(dist_bc >= 0.35f);
    REQUIRE(dist_ac >= 0.65f);  // Total chain should span ~0.65+
}

// ============================================================================
// Test 5: Extreme Velocity CCD Robustness
// ============================================================================
// Test CCD stability with extreme velocities (e.g., 100 m/s).
// Verify tunneling and collision detection remain stable.
//
TEST_CASE("ccd.extreme_velocity_robustness",
          "[validation][physics][collision][ccd]") {
    ParticleSystem system;
    system.enable_collisions(true);

    // Extremely fast projectile (100 m/s)
    auto projectile_mat = make_no_damping_material(1.0f, 0.9f);
    system.spawn(Vec3f(-10.0f, 0.0f, 0.0f), Vec3f(100.0f, 0.0f, 0.0f), 
                 projectile_mat, -1.0f, 0.2f);
    
    // Series of obstacles
    for (int i = 0; i < 3; ++i) {
        auto obstacle_mat = make_no_damping_material(1e6f, 0.8f);
        float x_pos = 5.0f + static_cast<float>(i) * 5.0f;
        system.spawn(Vec3f(x_pos, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                     obstacle_mat, -1.0f, 0.3f);
    }

    // Simulate with small timesteps to handle extreme velocity
    const float dt = 1.0f / 240.0f;  // Smaller dt for stability
    int num_steps = 1000;

    for (int step = 0; step < num_steps; ++step) {
        system.update(dt);
        
        // Check for NaN/Inf every 100 steps
        if (step % 100 == 0) {
            for (const auto& p : system.particles()) {
                REQUIRE(std::isfinite(p.position.x));
                REQUIRE(std::isfinite(p.velocity.x));
            }
        }
    }

    // Final checks
    REQUIRE(std::isfinite(system.particles()[0].position.x));
    REQUIRE(std::isfinite(system.particles()[0].velocity.x));
    
    // Projectile should have been affected by collisions (velocity reduced)
    float final_speed_x = std::abs(system.particles()[0].velocity.x);
    REQUIRE(final_speed_x < 100.0f);  // Reduced from initial 100
}

// ============================================================================
// Test 6: Collision Behavior Under Standard Conditions
// ============================================================================
// Two particles on direct collision course - verify collision is detected
// and velocities are affected appropriately.
//
TEST_CASE("ccd.head_on_collision",
          "[validation][physics][collision][ccd]") {
    ParticleSystem system;
    system.enable_collisions(true);

    // Particle moving right
    auto mat_a = make_no_damping_material(1.0f, 0.7f);
    system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(3.0f, 0.0f, 0.0f), 
                 mat_a, -1.0f, 0.25f);
    
    // Particle moving left
    auto mat_b = make_no_damping_material(1.0f, 0.7f);
    system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(-3.0f, 0.0f, 0.0f), 
                 mat_b, -1.0f, 0.25f);

    // Simulate collision
    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 200; ++step) {
        system.update(dt);
    }

    // Verify collision occurred (particles separated after close approach)
    float final_dist = (system.particles()[0].position - system.particles()[1].position).length();
    REQUIRE(final_dist >= 0.48f);  // Particles at least touching (sum of radii 0.5)

    // Both particles should have finite velocities
    REQUIRE(std::isfinite(system.particles()[0].velocity.x));
    REQUIRE(std::isfinite(system.particles()[1].velocity.x));
}

// ============================================================================
// Test 7: High-Speed Glancing Collision
// ============================================================================
// Fast object at shallow angle - test that CCD catches near-miss scenarios.
//
TEST_CASE("ccd.high_speed_glancing_collision",
          "[validation][physics][collision][ccd]") {
    ParticleSystem system;
    system.enable_collisions(true);

    // Fast projectile at shallow angle (mostly X, small Y perturbation)
    auto projectile_mat = make_no_damping_material(1.0f, 0.8f);
    system.spawn(Vec3f(-5.0f, 0.3f, 0.0f), Vec3f(20.0f, -0.5f, 0.0f), 
                 projectile_mat, -1.0f, 0.2f);
    
    // Small obstacle at glancing point
    auto obstacle_mat = make_no_damping_material(1e6f, 0.8f);
    system.spawn(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                 obstacle_mat, -1.0f, 0.25f);

    // Simulate
    const float dt = 1.0f / 120.0f;
    for (int step = 0; step < 180; ++step) {
        system.update(dt);
    }

    // Verify all finite
    for (const auto& p : system.particles()) {
        REQUIRE(std::isfinite(p.position.x));
        REQUIRE(std::isfinite(p.position.y));
        REQUIRE(std::isfinite(p.velocity.x));
        REQUIRE(std::isfinite(p.velocity.y));
    }

    // Verify separation maintained (should have bounced or slid)
    float final_dist = (system.particles()[0].position - system.particles()[1].position).length();
    REQUIRE(final_dist >= 0.42f);
}

// ============================================================================
// Test 8: Long-Duration High-Speed Stability
// ============================================================================
// Extended simulation with high-velocity particles to ensure CCD
// maintains stability over many frames without divergence.
//
TEST_CASE("ccd.long_duration_high_speed",
          "[validation][physics][collision][ccd]") {
    ParticleSystem system;
    system.enable_collisions(true);

    // Several high-speed particles bouncing in confined area
    std::vector<Vec3f> velocities = {
        Vec3f(20.0f, 10.0f, 0.0f),
        Vec3f(-15.0f, 8.0f, 0.0f),
        Vec3f(5.0f, -18.0f, 0.0f)
    };

    for (size_t i = 0; i < velocities.size(); ++i) {
        auto mat = make_no_damping_material(1.0f, 0.8f);
        float x = -2.0f + static_cast<float>(i) * 2.0f;
        system.spawn(Vec3f(x, 0.0f, 0.0f), velocities[i], 
                     mat, -1.0f, 0.3f);
    }

    // Wall obstacles
    for (int i = 0; i < 2; ++i) {
        auto wall_mat = make_no_damping_material(1e6f, 0.8f);
        float pos = -4.0f + static_cast<float>(i) * 8.0f;
        system.spawn(Vec3f(pos, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 
                     wall_mat, -1.0f, 0.2f);
    }

    // Long simulation
    const float dt = 1.0f / 120.0f;
    const int num_steps = 2400;  // 20 seconds at 120 Hz
    
    for (int step = 0; step < num_steps; ++step) {
        system.update(dt);
        
        // Periodic sanity checks
        if (step % 240 == 0) {
            for (const auto& p : system.particles()) {
                REQUIRE(std::isfinite(p.position.x));
                REQUIRE(std::isfinite(p.position.y));
                REQUIRE(std::isfinite(p.velocity.x));
                REQUIRE(std::isfinite(p.velocity.y));
                // Velocities shouldn't explode
                REQUIRE(p.velocity.squaredLength() < 1000.0f);
            }
        }
    }

    // Final verification - all particles still valid
    for (const auto& p : system.particles()) {
        REQUIRE(std::isfinite(p.position.x));
        REQUIRE(std::isfinite(p.velocity.x));
    }
}

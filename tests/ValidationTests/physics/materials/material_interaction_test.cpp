#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/particle_system.hpp>
#include <core/physics/force_field.hpp>
#include <core/math/vectors/vec3.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <cmath>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

/// Measure momentum transfer between two colliding particles
float measure_momentum_transfer(ParticleSystem& system) {
    float dt = 1.0f / 60.0f;
    
    // Run until particles separate (low relative velocity)
    for (int i = 0; i < 300; ++i) {
        system.update(dt);
        
        Vec3f v0 = system.particles()[0].velocity;
        Vec3f v1 = system.particles()[1].velocity;
        float relative_speed = (v0 - v1).length();
        
        if (relative_speed < 0.1f && i > 10) {
            break;
        }
    }
    
    return system.particles()[0].velocity.x;
}

}  // namespace

TEST_CASE("Material Interaction: Equal mass elastic collision", "[material][interaction][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
    
    // Particle A: moving right at 2 m/s with e=1
    auto mat_a = make_no_damping_material(1.0f, 1.0f);
    system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), mat_a, -1.0f, 0.3f);
    
    // Particle B: stationary with e=1
    auto mat_b = make_no_damping_material(1.0f, 1.0f);
    system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), mat_b, -1.0f, 0.3f);
    
    float final_a_velocity = measure_momentum_transfer(system);
    float final_b_velocity = system.particles()[1].velocity.x;
    
    // For equal mass elastic collision, velocities swap
    // A should slow down/stop, B should gain speed
    REQUIRE(final_a_velocity < 1.0f);
    REQUIRE(final_b_velocity > 0.5f);
    
    // Momentum should be conserved
    float initial_momentum = 1.0f * 2.0f + 1.0f * 0.0f;
    float final_momentum = 1.0f * final_a_velocity + 1.0f * final_b_velocity;
    REQUIRE_THAT(final_momentum, WithinRel(initial_momentum, 0.15f));
}

TEST_CASE("Material Interaction: Heavy vs light collision", "[material][interaction][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
    
    // Light particle moving right at 2 m/s
    auto light = make_no_damping_material(0.5f, 0.0f);
    system.spawn(Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), light, -1.0f, 0.3f);
    
    // Heavy stationary particle (2x heavier)
    auto heavy = make_no_damping_material(1.0f, 0.0f);
    system.spawn(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f), heavy, -1.0f, 0.3f);
    
    float final_light_velocity = measure_momentum_transfer(system);
    float final_heavy_velocity = system.particles()[1].velocity.x;
    
    // Light particle should slow down significantly
    REQUIRE(final_light_velocity < 2.0f);
    
    // Heavy particle should move forward
    REQUIRE(final_heavy_velocity > 0.0f);
    
    // Momentum should be conserved
    float initial_momentum = 0.5f * 2.0f + 1.0f * 0.0f;
    float final_momentum = 0.5f * final_light_velocity + 1.0f * final_heavy_velocity;
    REQUIRE_THAT(final_momentum, WithinRel(initial_momentum, 0.15f));
}

TEST_CASE("Material Interaction: Inelastic collision (e=0)", "[material][interaction][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
    
    // Particle A: moving right at 2 m/s (e=0)
    auto mat_a = make_no_damping_material(1.0f, 0.0f);
    system.spawn(Vec3f(-1.5f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), mat_a, -1.0f, 0.3f);
    
    // Particle B: stationary (e=0)
    auto mat_b = make_no_damping_material(1.0f, 0.0f);
    system.spawn(Vec3f(1.5f, 0.0f, 0.0f), Vec3f(0.0f), mat_b, -1.0f, 0.3f);
    
    float final_a_velocity = measure_momentum_transfer(system);
    float final_b_velocity = system.particles()[1].velocity.x;
    
    // Both should move in the same direction (inelastic)
    REQUIRE(final_a_velocity >= final_b_velocity);
    REQUIRE(final_b_velocity > 0.0f);
    
    // Momentum should be conserved
    float initial_momentum = 1.0f * 2.0f + 1.0f * 0.0f;
    float final_momentum = 1.0f * final_a_velocity + 1.0f * final_b_velocity;
    REQUIRE_THAT(final_momentum, WithinRel(initial_momentum, 0.15f));
}

TEST_CASE("Material Interaction: Different restitution values", "[material][interaction][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
    
    // A has high restitution
    auto high_e = make_no_damping_material(1.0f, 0.8f);
    system.spawn(Vec3f(-1.5f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), high_e, -1.0f, 0.3f);
    
    // B has low restitution
    auto low_e = make_no_damping_material(1.0f, 0.1f);
    system.spawn(Vec3f(1.5f, 0.0f, 0.0f), Vec3f(0.0f), low_e, -1.0f, 0.3f);
    
    float final_a_velocity = measure_momentum_transfer(system);
    float final_b_velocity = system.particles()[1].velocity.x;
    
    // Uses min restitution = 0.1 (more inelastic)
    // A slowed, B moving forward
    REQUIRE(final_a_velocity < 2.0f);
    REQUIRE(final_b_velocity > 0.0f);
    
    // Momentum should be conserved
    float initial_momentum = 1.0f * 2.0f + 1.0f * 0.0f;
    float final_momentum = 1.0f * final_a_velocity + 1.0f * final_b_velocity;
    REQUIRE_THAT(final_momentum, WithinRel(initial_momentum, 0.15f));
}

TEST_CASE("Material Interaction: Three particle system stability", "[material][interaction][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
    
    // Three particles in a line
    auto mat = make_no_damping_material(1.0f, 0.1f);
    
    // Left-moving particle
    system.spawn(Vec3f(-3.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), mat, -1.0f, 0.3f);
    
    // Middle (stationary)
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 0.3f);
    
    // Right (stationary)
    system.spawn(Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 0.3f);
    
    float dt = 1.0f / 60.0f;
    
    // Run for a few seconds
    for (int i = 0; i < 300; ++i) {
        system.update(dt);
    }
    
    // All particles should still exist and not have explosive velocities
    for (size_t i = 0; i < system.particles().size(); ++i) {
        float speed = system.particles()[i].velocity.length();
        REQUIRE(speed < 10.0f);  // No explosive behavior
        
        float pos_magnitude = system.particles()[i].position.length();
        REQUIRE(pos_magnitude < 100.0f);  // Particles don't fly away
    }
}


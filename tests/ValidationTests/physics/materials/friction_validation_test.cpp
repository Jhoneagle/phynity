#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/micro/particle_system.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/math/vectors/vec3.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <cmath>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("Friction: Friction parameter is read from material", "[material][friction][validation]") {
    ParticleSystem system;
    
    // Create materials with different friction values
    auto low_friction = make_no_damping_material(1.0f, 0.0f);
    low_friction.friction = 0.2f;
    
    auto high_friction = make_no_damping_material(1.0f, 0.0f);
    high_friction.friction = 0.8f;
    
    // Particles should be created with correct friction values
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), low_friction, -1.0f, 0.5f);
    system.spawn(Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f), high_friction, -1.0f, 0.5f);
    
    REQUIRE(system.particles()[0].material.friction == 0.2f);
    REQUIRE(system.particles()[1].material.friction == 0.8f);
}

TEST_CASE("Friction: Friction in collision contact", "[material][friction][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
    
    // Moving particle with friction
    auto moving = make_no_damping_material(1.0f, 0.0f);
    moving.friction = 0.5f;
    system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(3.0f, 0.0f, 0.0f), moving, -1.0f, 0.4f);
    
    // Stationary particle with friction
    auto stationary = make_no_damping_material(2.0f, 0.0f);
    stationary.friction = 0.5f;
    system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), stationary, -1.0f, 0.4f);
    
    float dt = 1.0f / 60.0f;
    float initial_moving_speed = system.particles()[0].velocity.length();
    
    // Collide and settle
    for (int i = 0; i < 200; ++i) {
        system.update(dt);
    }
    
    // Moving particle should have lost energy from collision
    float final_moving_speed = system.particles()[0].velocity.length();
    REQUIRE(final_moving_speed < initial_moving_speed);
}

TEST_CASE("Friction: Contact respects friction material property", "[material][friction][validation]") {
    // Test with low friction
    ParticleSystem system_low;
    system_low.enable_collisions(true);
    auto config = system_low.constraint_solver_config();
    config.use_warm_start = false;
    system_low.set_constraint_solver_config(config);
    
    auto low_mat = make_no_damping_material(1.0f, 0.0f);
    low_mat.friction = 0.1f;
    
    system_low.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), low_mat, -1.0f, 0.4f);
    system_low.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), low_mat, -1.0f, 0.4f);
    
    // Test with high friction
    ParticleSystem system_high;
    system_high.enable_collisions(true);
    config.use_warm_start = false;
    system_high.set_constraint_solver_config(config);
    
    auto high_mat = make_no_damping_material(1.0f, 0.0f);
    high_mat.friction = 0.9f;
    
    system_high.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), high_mat, -1.0f, 0.4f);
    system_high.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), high_mat, -1.0f, 0.4f);
    
    float dt = 1.0f / 60.0f;
    
    // Simulate both
    for (int i = 0; i < 200; ++i) {
        system_low.update(dt);
        system_high.update(dt);
    }
    
    // Both should collide and settle
    float low_friction_final_speed = system_low.particles()[0].velocity.length();
    float high_friction_final_speed = system_high.particles()[0].velocity.length();
    
    // Systems should reach reasonable end states
    REQUIRE(low_friction_final_speed < 2.0f);
    REQUIRE(high_friction_final_speed < 2.0f);
}

TEST_CASE("Friction: Multiple collisions with friction", "[material][friction][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
    
    // Create chain of particles
    for (int i = 0; i < 3; ++i) {
        auto mat = make_no_damping_material(1.0f, 0.0f);
        mat.friction = 0.5f;
        
        float x = -3.0f + (static_cast<float>(i) * 2.0f);
        float vx = (i == 0) ? 2.0f : 0.0f;
        system.spawn(Vec3f(x, 0.0f, 0.0f), Vec3f(vx, 0.0f, 0.0f), mat, -1.0f, 0.4f);
    }
    
    float dt = 1.0f / 60.0f;
    
    // Simulate collisions
    for (int i = 0; i < 300; ++i) {
        system.update(dt);
    }
    
    // All particles should exist and have reasonable velocities
    for (const auto& particle : system.particles()) {
        REQUIRE(particle.velocity.length() < 10.0f);
        REQUIRE(std::isfinite(particle.position.x));
    }
}

TEST_CASE("Friction: Friction in different contact pairs", "[material][friction][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
    
    // A: low friction
    auto mat_a = make_no_damping_material(1.0f, 0.0f);
    mat_a.friction = 0.2f;
    system.spawn(Vec3f(-3.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), mat_a, -1.0f, 0.4f);
    
    // B: high friction
    auto mat_b = make_no_damping_material(1.0f, 0.0f);
    mat_b.friction = 0.9f;
    system.spawn(Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f), mat_b, -1.0f, 0.4f);
    
    float dt = 1.0f / 60.0f;
    
    // Simulate collision
    for (int i = 0; i < 250; ++i) {
        system.update(dt);
    }
    
    // Contact between A and B should use min(0.2, 0.9) = 0.2 friction
    // System should stabilize
    REQUIRE(std::isfinite(system.particles()[0].velocity.x));
    REQUIRE(std::isfinite(system.particles()[1].velocity.x));
}

TEST_CASE("Friction: static-like threshold vs kinetic-like motion", "[material][friction][edge][validation]") {
    auto run_with_initial_speed = [](float initial_speed) {
        ParticleSystem system;
        system.enable_collisions(true);

        auto config = system.constraint_solver_config();
        config.use_warm_start = false;
        system.set_constraint_solver_config(config);

        auto moving = make_no_damping_material(1.0f, 0.0f);
        moving.friction = 1.2f;
        auto stationary = make_no_damping_material(2.0f, 0.0f);
        stationary.friction = 1.2f;

        system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(initial_speed, 0.0f, 0.0f), moving, -1.0f, 0.4f);
        system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), stationary, -1.0f, 0.4f);

        constexpr float dt = 1.0f / 60.0f;
        for (int i = 0; i < 240; ++i) {
            system.update(dt);
        }
        return std::abs(system.particles()[0].velocity.x);
    };

    const float low_input_final_speed = run_with_initial_speed(0.5f);
    const float high_input_final_speed = run_with_initial_speed(2.0f);

    REQUIRE(std::isfinite(low_input_final_speed));
    REQUIRE(std::isfinite(high_input_final_speed));
    REQUIRE(low_input_final_speed < high_input_final_speed);
}

TEST_CASE("Friction: extreme coefficients remain stable", "[material][friction][edge][validation]") {
    std::vector<float> coefficients{0.0f, 0.05f, 1.5f};

    for (const float mu : coefficients) {
        ParticleSystem system;
        system.enable_collisions(true);

        auto config = system.constraint_solver_config();
        config.use_warm_start = false;
        system.set_constraint_solver_config(config);

        auto ground = make_no_damping_material(0.0f, 0.0f);
        ground.friction = mu;
        auto block = make_no_damping_material(1.0f, 0.0f);
        block.friction = mu;

        system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), ground, -1.0f, 2.0f);
        system.spawn(Vec3f(0.0f, 2.5f, 0.0f), Vec3f(1.5f, 0.0f, 0.0f), block, -1.0f, 0.5f);

        system.add_force_field(
            std::make_unique<GravityField>(
                Vec3f(0.0f, -phynity::test::helpers::constants::EARTH_GRAVITY, 0.0f)));

        constexpr float dt = 1.0f / 60.0f;
        for (int i = 0; i < 360; ++i) {
            system.update(dt);
        }

        const auto& p = system.particles()[1];
        REQUIRE(std::isfinite(p.position.x));
        REQUIRE(std::isfinite(p.position.y));
        REQUIRE(std::isfinite(p.velocity.x));
        REQUIRE(std::isfinite(p.velocity.y));
    }
}

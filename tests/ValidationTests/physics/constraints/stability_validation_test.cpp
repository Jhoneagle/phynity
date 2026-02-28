#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/particle_system.hpp>
#include <core/physics/force_field.hpp>
#include <core/math/vectors/vec3.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <algorithm>
#include <cmath>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;

namespace {

bool system_is_finite(const ParticleSystem& system) {
    for (const auto& p : system.particles()) {
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z) ||
            !std::isfinite(p.velocity.x) || !std::isfinite(p.velocity.y) || !std::isfinite(p.velocity.z)) {
            return false;
        }
    }
    return true;
}

float total_energy(const ParticleSystem& system, float gravity) {
    float total_ke = 0.0f;
    float total_pe = 0.0f;

    for (const auto& p : system.particles()) {
        if (p.material.mass > 0.0f) {
            total_ke += 0.5f * p.material.mass * p.velocity.squaredLength();
            total_pe += p.material.mass * gravity * p.position.y;
        }
    }

    return total_ke + total_pe;
}

}  // namespace

TEST_CASE("Constraint stability: energy stays bounded", "[constraint][stability][validation]") {
    ParticleSystem system;
    system.enable_collisions(false);
    system.enable_constraints(true);

    auto config = system.constraint_solver_config();
    config.iterations = 8;
    config.enable_adaptive_iterations = false;
    config.use_warm_start = true;
    system.set_constraint_solver_config(config);

    auto material = make_no_damping_material(1.0f, 0.0f);
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.2f, 0.0f, 0.0f), material, -1.0f, 0.5f);
    system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(-0.2f, 0.0f, 0.0f), material, -1.0f, 0.5f);

    system.add_fixed_constraint(0, 1);

    constexpr float dt = 1.0f / 60.0f;
    float initial_energy = total_energy(system, 0.0f);
    float max_energy = initial_energy;

    for (int i = 0; i < 600; ++i) {
        system.update(dt);
        float energy = total_energy(system, 0.0f);
        REQUIRE(std::isfinite(energy));
        max_energy = std::max(max_energy, energy);
    }

    REQUIRE(system_is_finite(system));
    REQUIRE(max_energy < initial_energy * 5.0f + 0.5f);
}

TEST_CASE("Constraint stability: long-run fixed constraint drift", "[constraint][stability][validation]") {
    ParticleSystem system;
    system.enable_collisions(false);
    system.enable_constraints(true);

    auto config = system.constraint_solver_config();
    config.iterations = 8;
    config.enable_adaptive_iterations = false;
    config.use_warm_start = true;
    system.set_constraint_solver_config(config);

    auto material = make_no_damping_material(1.0f, 0.0f);
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), material, -1.0f, 0.5f);
    system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), material, -1.0f, 0.5f);

    system.add_fixed_constraint(0, 1);

    constexpr float dt = 1.0f / 60.0f;
    float max_drift = 0.0f;

    for (int i = 0; i < 1200; ++i) {
        system.update(dt);
        float distance = (system.particles()[1].position - system.particles()[0].position).length();
        max_drift = std::max(max_drift, std::abs(distance - 2.0f));
    }

    REQUIRE(system_is_finite(system));
    REQUIRE(max_drift < 1.0f);
}

TEST_CASE("Constraint stability: extreme timestep values stay finite", "[constraint][stability][validation]") {
    auto run_dt = [](float dt, int steps) {
        ParticleSystem system;
        system.enable_collisions(true);
        system.enable_constraints(true);

        auto config = system.constraint_solver_config();
        config.iterations = 8;
        config.enable_adaptive_iterations = true;
        config.use_warm_start = false;
        system.set_constraint_solver_config(config);

        auto ground = make_no_damping_material(0.0f, 0.1f);
        auto body = make_no_damping_material(1.0f, 0.1f);

        system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), ground, -1.0f, 1.5f);
        system.spawn(Vec3f(0.0f, 8.0f, 0.0f), Vec3f(0.0f), body, -1.0f, 0.5f);

        system.add_force_field(std::make_unique<GravityField>(
            Vec3f(0.0f, -phynity::test::helpers::constants::EARTH_GRAVITY, 0.0f)));

        for (int i = 0; i < steps; ++i) {
            system.update(dt);
        }

        return system;
    };

    ParticleSystem small_dt = run_dt(1.0f / 600.0f, 2000);
    ParticleSystem large_dt = run_dt(1.0f / 10.0f, 120);

    REQUIRE(system_is_finite(small_dt));
    REQUIRE(system_is_finite(large_dt));
}

TEST_CASE("Constraint stability: max iteration improves settling", "[constraint][stability][validation]") {
    auto run_iterations = [](int iterations) {
        ParticleSystem system;
        system.enable_collisions(true);
        system.enable_constraints(true);

        auto config = system.constraint_solver_config();
        config.iterations = iterations;
        config.enable_adaptive_iterations = false;
        config.use_warm_start = false;
        system.set_constraint_solver_config(config);

        auto ground = make_no_damping_material(0.0f, 0.1f);
        auto body = make_no_damping_material(1.0f, 0.1f);

        system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), ground, -1.0f, 1.5f);
        system.spawn(Vec3f(0.0f, 4.0f, 0.0f), Vec3f(0.0f), body, -1.0f, 0.5f);

        system.add_force_field(std::make_unique<GravityField>(
            Vec3f(0.0f, -phynity::test::helpers::constants::EARTH_GRAVITY, 0.0f)));

        constexpr float dt = 1.0f / 60.0f;
        for (int i = 0; i < 480; ++i) {
            system.update(dt);
        }

        float speed = system.particles()[1].velocity.length();
        float height = system.particles()[1].position.y;

        return std::pair<float, float>{speed, height};
    };

    const auto low_iter = run_iterations(2);
    const auto high_iter = run_iterations(12);

    REQUIRE(low_iter.first >= 0.0f);
    REQUIRE(high_iter.first >= 0.0f);
    REQUIRE(high_iter.first <= low_iter.first + 0.2f);
    REQUIRE(high_iter.second < 20.0f);
}

TEST_CASE("Constraint stability: extreme velocity stays finite", "[constraint][stability][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);
    system.enable_constraints(true);

    auto config = system.constraint_solver_config();
    config.iterations = 8;
    config.enable_adaptive_iterations = true;
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);

    auto ground = make_no_damping_material(0.0f, 0.2f);
    auto body = make_no_damping_material(1.0f, 0.2f);

    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), ground, -1.0f, 1.5f);
    system.spawn(Vec3f(0.0f, 6.0f, 0.0f), Vec3f(0.0f, -100.0f, 0.0f), body, -1.0f, 0.5f);

    system.add_force_field(std::make_unique<GravityField>(
        Vec3f(0.0f, -phynity::test::helpers::constants::EARTH_GRAVITY, 0.0f)));

    constexpr float dt = 1.0f / 120.0f;
    for (int i = 0; i < 240; ++i) {
        system.update(dt);
    }

    REQUIRE(system_is_finite(system));
    REQUIRE(std::abs(system.particles()[1].velocity.y) < 1000.0f);
}

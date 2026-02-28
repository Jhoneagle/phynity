#include <catch2/catch_test_macros.hpp>

#include <core/physics/particle_system.hpp>
#include <core/physics/constraints/constraint.hpp>
#include <core/physics/constraints/constraint_solver.hpp>
#include <core/physics/constraints/contact_constraint.hpp>
#include <core/physics/constraints/fixed_constraint.hpp>
#include <core/physics/collision/contact_manifold.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

using namespace phynity::physics;
using namespace phynity::physics::constraints;
using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;

namespace {

struct SolveMetrics {
    float closing_speed = 0.0f;
    bool finite = true;
};

SolveMetrics solve_fixed_once(int iterations, bool warm_start) {
    std::vector<Particle> particles;
    particles.emplace_back(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f));
    particles.emplace_back(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f));

    particles[0].material = make_no_damping_material(1.0f, 0.0f);
    particles[1].material = make_no_damping_material(1.0f, 0.0f);

    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.push_back(std::make_unique<FixedConstraint>(particles[0], particles[1]));

    particles[1].position = Vec3f(4.0f, 0.0f, 0.0f);

    ConstraintSolver solver;
    ConstraintSolverConfig config;
    config.iterations = iterations;
    config.enable_adaptive_iterations = false;
    config.use_warm_start = warm_start;
    config.convergence_threshold = 1e-6f;
    solver.set_config(config);

    solver.solve(constraints, particles);

    SolveMetrics metrics;
    metrics.closing_speed = particles[0].velocity.x - particles[1].velocity.x;
    metrics.finite = std::isfinite(particles[0].velocity.x) && std::isfinite(particles[1].velocity.x);
    return metrics;
}

float second_step_closing_speed_with_or_without_warm_start(bool warm_start) {
    std::vector<Particle> particles;
    particles.emplace_back(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f));
    particles.emplace_back(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f));

    particles[0].material = make_no_damping_material(1.0f, 0.0f);
    particles[1].material = make_no_damping_material(1.0f, 0.0f);

    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.push_back(std::make_unique<FixedConstraint>(particles[0], particles[1]));

    ConstraintSolver solver;
    ConstraintSolverConfig config;
    config.iterations = 2;
    config.enable_adaptive_iterations = false;
    config.use_warm_start = warm_start;
    config.convergence_threshold = 1e-6f;
    solver.set_config(config);

    particles[1].position = Vec3f(4.0f, 0.0f, 0.0f);
    solver.solve(constraints, particles);

    particles[0].position = Vec3f(0.0f, 0.0f, 0.0f);
    particles[1].position = Vec3f(4.0f, 0.0f, 0.0f);
    particles[0].velocity = Vec3f(0.0f);
    particles[1].velocity = Vec3f(0.0f);

    solver.solve(constraints, particles);
    return particles[0].velocity.x - particles[1].velocity.x;
}

bool particle_system_finite(const ParticleSystem& system) {
    for (const auto& p : system.particles()) {
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z) ||
            !std::isfinite(p.velocity.x) || !std::isfinite(p.velocity.y) || !std::isfinite(p.velocity.z)) {
            return false;
        }
    }
    return true;
}

} // namespace

TEST_CASE("Constraint convergence: more iterations increase correction", "[constraint][convergence][validation]") {
    const SolveMetrics low_iter = solve_fixed_once(1, false);
    const SolveMetrics high_iter = solve_fixed_once(8, false);

    REQUIRE(low_iter.finite);
    REQUIRE(high_iter.finite);
    REQUIRE(high_iter.closing_speed >= low_iter.closing_speed);
}

TEST_CASE("Constraint convergence: warm-start improves second-step response", "[constraint][convergence][validation]") {
    const float no_warm = second_step_closing_speed_with_or_without_warm_start(false);
    const float warm = second_step_closing_speed_with_or_without_warm_start(true);

    REQUIRE(std::isfinite(no_warm));
    REQUIRE(std::isfinite(warm));
    REQUIRE(warm >= no_warm * 0.95f);
}

TEST_CASE("Constraint convergence: over-constrained pair remains finite", "[constraint][convergence][validation]") {
    std::vector<Particle> particles;
    particles.emplace_back(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f));
    particles.emplace_back(Vec3f(1.2f, 0.0f, 0.0f), Vec3f(0.0f));

    particles[0].material = make_no_damping_material(1.0f, 0.2f);
    particles[1].material = make_no_damping_material(1.0f, 0.2f);

    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.push_back(std::make_unique<FixedConstraint>(particles[0], particles[1]));

    particles[1].position = Vec3f(0.8f, 0.0f, 0.0f);

    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.contact.penetration = 0.4f;
    manifold.contact.relative_velocity_along_normal = -0.5f;
    manifold.update_contact_id();
    constraints.push_back(std::make_unique<ContactConstraint>(manifold, particles[0], particles[1]));

    ConstraintSolver solver;
    ConstraintSolverConfig config;
    config.iterations = 8;
    config.enable_adaptive_iterations = false;
    config.use_warm_start = false;
    solver.set_config(config);

    solver.solve(constraints, particles);

    REQUIRE(std::isfinite(particles[0].velocity.x));
    REQUIRE(std::isfinite(particles[1].velocity.x));
    REQUIRE(std::abs(particles[0].velocity.x) < 1000.0f);
    REQUIRE(std::abs(particles[1].velocity.x) < 1000.0f);
}

TEST_CASE("Constraint convergence: timestep robustness stays finite", "[constraint][convergence][validation]") {
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
        system.spawn(Vec3f(0.0f, 6.0f, 0.0f), Vec3f(0.0f), body, -1.0f, 0.5f);

        system.add_force_field(std::make_unique<GravityField>(
            Vec3f(0.0f, -phynity::test::helpers::constants::EARTH_GRAVITY, 0.0f)));

        for (int i = 0; i < steps; ++i) {
            system.update(dt);
        }
        return system;
    };

    ParticleSystem small_dt = run_dt(1.0f / 240.0f, 1400);
    ParticleSystem large_dt = run_dt(1.0f / 20.0f, 140);

    REQUIRE(particle_system_finite(small_dt));
    REQUIRE(particle_system_finite(large_dt));
}

TEST_CASE("Constraint convergence: larger contact sets remain bounded", "[constraint][convergence][validation]") {
    auto run_contact_set = [](int count) {
        ParticleSystem system;
        system.enable_collisions(true);
        system.enable_constraints(true);

        auto config = system.constraint_solver_config();
        config.iterations = 8;
        config.enable_adaptive_iterations = true;
        config.use_warm_start = false;
        system.set_constraint_solver_config(config);

        auto material = make_no_damping_material(1.0f, 0.1f);
        const float radius = 0.5f;

        for (int i = 0; i < count; ++i) {
            const float x = static_cast<float>(i) * 0.95f;
            system.spawn(Vec3f(x, 0.0f, 0.0f), Vec3f(0.0f), material, -1.0f, radius);
        }

        constexpr float dt = 1.0f / 60.0f;
        for (int i = 0; i < 240; ++i) {
            system.update(dt);
        }

        return system;
    };

    ParticleSystem set_6 = run_contact_set(6);
    ParticleSystem set_16 = run_contact_set(16);

    REQUIRE(particle_system_finite(set_6));
    REQUIRE(particle_system_finite(set_16));

    float max_speed_6 = 0.0f;
    for (const auto& p : set_6.particles()) {
        max_speed_6 = std::max(max_speed_6, p.velocity.length());
    }

    float max_speed_16 = 0.0f;
    for (const auto& p : set_16.particles()) {
        max_speed_16 = std::max(max_speed_16, p.velocity.length());
    }

    REQUIRE(max_speed_6 < 200.0f);
    REQUIRE(max_speed_16 < 300.0f);
}

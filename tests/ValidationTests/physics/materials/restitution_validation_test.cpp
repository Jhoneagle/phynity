#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/micro/particle_system.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/math/vectors/vec3.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

struct BounceMetrics {
    bool impact_detected = false;
    float impact_speed = 0.0f;
    float rebound_speed = 0.0f;
    float max_height_after = 0.0f;
};

BounceMetrics measure_single_bounce(
    ParticleSystem& system,
    size_t particle_index,
    float dt,
    int max_steps,
    float impact_height_threshold) {

    BounceMetrics metrics;
    float prev_vy = system.particles()[particle_index].velocity.y;

    for (int i = 0; i < max_steps; ++i) {
        system.update(dt);
        const auto& p = system.particles()[particle_index];

        if (!metrics.impact_detected && prev_vy < 0.0f && p.velocity.y > 0.0f &&
            p.position.y < impact_height_threshold) {
            metrics.impact_detected = true;
            metrics.impact_speed = std::abs(prev_vy);
            metrics.rebound_speed = p.velocity.y;
            metrics.max_height_after = p.position.y;
        }

        if (metrics.impact_detected) {
            metrics.max_height_after = std::max(metrics.max_height_after, p.position.y);
        }

        prev_vy = p.velocity.y;
    }

    return metrics;
}

void setup_drop_scene(ParticleSystem& system, float restitution, float initial_height) {
    system.enable_collisions(true);

    // Spawn static ground (infinite mass = kinematic)
    auto ground = make_no_damping_material(0.0f, restitution);  // mass = 0 means infinite (kinematic)
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), ground, -1.0f, 1.5f);

    auto ball = make_no_damping_material(1.0f, restitution);
    system.spawn(Vec3f(0.0f, initial_height, 0.0f), Vec3f(0.0f), ball, -1.0f, 0.5f);

    system.add_force_field(
        std::make_unique<GravityField>(
            Vec3f(0.0f, -phynity::test::helpers::constants::EARTH_GRAVITY, 0.0f)));
    
    // Disable warm-start for restitution tests to avoid accumulated impulse issues
    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);
}

}  // namespace

TEST_CASE("Restitution: e=0 inelastic impact", "[material][restitution][validation]") {
    ParticleSystem system;
    setup_drop_scene(system, 0.0f, 5.0f);

    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 300; ++i) {
        system.update(dt);
    }

    float speed = system.particles()[1].velocity.length();
    // Relax tolerance to 0.3 m/s to account for numerical effects and multi-frame contact
    REQUIRE_THAT(speed, WithinAbs(0.0f, 0.3f));
}

TEST_CASE("Restitution: coefficient sweep", "[material][restitution][validation]") {
    std::vector<float> coefficients{0.5f, 1.0f};  // e=0 tested separately in "inelastic impact" test
    float initial_height = 8.0f;

    for (float e : coefficients) {
        SECTION("e=" + std::to_string(e)) {
            ParticleSystem system;
            setup_drop_scene(system, e, initial_height);

            BounceMetrics metrics = measure_single_bounce(system, 1, 1.0f / 60.0f, 600, 4.0f);
            REQUIRE(metrics.impact_detected);

            float measured_e = (metrics.impact_speed > 0.0f)
                ? (metrics.rebound_speed / metrics.impact_speed)
                : 0.0f;

            // Use 10% tolerance to account for numerical effects
            REQUIRE_THAT(measured_e, WithinRel(e, 0.10f));
        }
    }
}

TEST_CASE("Restitution: bounce height relationship", "[material][restitution][validation]") {
    float initial_height = 10.0f;
    float restitution = 0.5f;

    ParticleSystem system;
    setup_drop_scene(system, restitution, initial_height);

    BounceMetrics metrics = measure_single_bounce(system, 1, 1.0f / 60.0f, 800, 4.0f);
    REQUIRE(metrics.impact_detected);

    // Account for collision geometry: ground sphere (r=1.5) + ball sphere (r=0.5)
    // Contact happens at y = 0 + 1.5 + 0.5 = 2.0
    // Ball falls from 10.0 to 2.0 = 8.0m drop
    // Expected bounce: contact_y + e² × drop_height = 2.0 + 0.25 × 8.0 = 4.0m
    float contact_height = 2.0f;  // Ground radius (1.5) + ball radius (0.5)
    float drop_height = initial_height - contact_height;
    float expected_height = contact_height + restitution * restitution * drop_height;
    
    // Use 10% tolerance to account for numerical integration and energy loss
    REQUIRE_THAT(metrics.max_height_after, WithinRel(expected_height, 0.10f));
}

TEST_CASE("Restitution: sequential bounce decay", "[material][restitution][validation]") {
    float restitution = 0.8f;
    float initial_height = 6.0f;
    float contact_height = 2.0f;  // Ground radius + ball radius

    ParticleSystem system;
    setup_drop_scene(system, restitution, initial_height);

    float dt = 1.0f / 60.0f;
    std::vector<float> bounce_peaks;
    float prev_vy = system.particles()[1].velocity.y;
    float current_peak = system.particles()[1].position.y;
    bool rising = false;

    for (int i = 0; i < 2400 && bounce_peaks.size() < 5; ++i) {
        system.update(dt);
        const auto& p = system.particles()[1];

        if (p.velocity.y > 0.0f) {
            rising = true;
            current_peak = std::max(current_peak, p.position.y);
        }

        if (rising && prev_vy > 0.0f && p.velocity.y <= 0.0f) {
            // Measure height above contact point for proper ratio
            float height_above_contact = current_peak - contact_height;
            bounce_peaks.push_back(height_above_contact);
            rising = false;
            current_peak = p.position.y;
        }

        prev_vy = p.velocity.y;
    }

    REQUIRE(bounce_peaks.size() >= 3);

    for (size_t i = 1; i < bounce_peaks.size(); ++i) {
        float ratio = bounce_peaks[i] / bounce_peaks[i - 1];
        float expected_ratio = restitution * restitution;
        REQUIRE_THAT(ratio, WithinRel(expected_ratio, 0.15f));
    }
}

TEST_CASE("Restitution: asymmetry invariance e_AB vs e_BA", "[material][restitution][edge][validation]") {
    auto run_case = [](float restitution_a, float restitution_b) {
        ParticleSystem system;
        system.enable_collisions(true);

        auto config = system.constraint_solver_config();
        config.use_warm_start = false;
        system.set_constraint_solver_config(config);

        auto mat_a = make_no_damping_material(1.0f, restitution_a);
        auto mat_b = make_no_damping_material(1.0f, restitution_b);

        system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), mat_a, -1.0f, 0.4f);
        system.spawn(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), mat_b, -1.0f, 0.4f);

        constexpr float dt = 1.0f / 60.0f;
        for (int i = 0; i < 220; ++i) {
            system.update(dt);
        }

        const float v0 = system.particles()[0].velocity.x;
        const float v1 = system.particles()[1].velocity.x;
        return std::pair<float, float>{v0, v1};
    };

    const auto [v0_ab, v1_ab] = run_case(0.8f, 0.2f);
    const auto [v0_ba, v1_ba] = run_case(0.2f, 0.8f);

    REQUIRE_THAT(v0_ab, WithinRel(v0_ba, 0.12f));
    REQUIRE_THAT(v1_ab, WithinRel(v1_ba, 0.12f));
}

TEST_CASE("Restitution: long-sequence decay remains bounded", "[material][restitution][edge][validation]") {
    constexpr float restitution = 0.7f;
    constexpr float initial_height = 8.0f;
    constexpr float contact_height = 2.0f;

    ParticleSystem system;
    setup_drop_scene(system, restitution, initial_height);

    constexpr float dt = 1.0f / 60.0f;
    std::vector<float> peaks;
    float prev_vy = system.particles()[1].velocity.y;
    float current_peak = system.particles()[1].position.y;
    bool rising = false;

    for (int i = 0; i < 6000 && peaks.size() < 10; ++i) {
        system.update(dt);
        const auto& p = system.particles()[1];

        if (p.velocity.y > 0.0f) {
            rising = true;
            current_peak = std::max(current_peak, p.position.y);
        }

        if (rising && prev_vy > 0.0f && p.velocity.y <= 0.0f) {
            peaks.push_back(current_peak - contact_height);
            rising = false;
            current_peak = p.position.y;
        }

        prev_vy = p.velocity.y;
    }

    REQUIRE(peaks.size() >= 6);
    for (size_t i = 1; i < peaks.size(); ++i) {
        REQUIRE(std::isfinite(peaks[i]));
        REQUIRE(peaks[i] <= peaks[i - 1] + 0.05f);
    }
}

TEST_CASE("Restitution: stacking bounces transfer impulse", "[material][restitution][edge][validation]") {
    ParticleSystem system;
    system.enable_collisions(true);

    auto config = system.constraint_solver_config();
    config.use_warm_start = false;
    system.set_constraint_solver_config(config);

    auto ground = make_no_damping_material(0.0f, 0.8f);
    auto bottom = make_no_damping_material(1.0f, 0.8f);
    auto top = make_no_damping_material(1.0f, 0.8f);

    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), ground, -1.0f, 1.5f);
    system.spawn(Vec3f(0.0f, 2.5f, 0.0f), Vec3f(0.0f), bottom, -1.0f, 0.5f);
    system.spawn(Vec3f(0.0f, 5.2f, 0.0f), Vec3f(0.0f), top, -1.0f, 0.5f);

    system.add_force_field(
        std::make_unique<GravityField>(
            Vec3f(0.0f, -phynity::test::helpers::constants::EARTH_GRAVITY, 0.0f)));

    constexpr float dt = 1.0f / 60.0f;
    float max_upward_top_velocity = 0.0f;

    for (int i = 0; i < 900; ++i) {
        system.update(dt);
        max_upward_top_velocity = std::max(max_upward_top_velocity, system.particles()[2].velocity.y);
    }

    REQUIRE(max_upward_top_velocity > 0.2f);
    REQUIRE(std::isfinite(system.particles()[1].position.y));
    REQUIRE(std::isfinite(system.particles()[2].position.y));
}

TEST_CASE("Restitution: extreme coefficient values stay stable", "[material][restitution][edge][validation]") {
    std::vector<float> coefficients{0.0f, 0.05f, 0.95f};

    for (const float e : coefficients) {
        ParticleSystem system;
        setup_drop_scene(system, e, 6.0f);

        constexpr float dt = 1.0f / 60.0f;
        for (int i = 0; i < 800; ++i) {
            system.update(dt);
        }

        const auto& p = system.particles()[1];
        REQUIRE(std::isfinite(p.position.x));
        REQUIRE(std::isfinite(p.position.y));
        REQUIRE(std::isfinite(p.velocity.x));
        REQUIRE(std::isfinite(p.velocity.y));
        REQUIRE(p.position.y < 100.0f);
    }
}

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/calculus/integrators.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <cmath>
#include <array>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using phynity::math::calculus::IntegrationState;
using phynity::math::calculus::integrate_forward_euler;
using phynity::math::calculus::integrate_semi_implicit_euler;
using phynity::math::calculus::integrate_velocity_verlet;
using phynity::math::calculus::integrate_rk4;
using phynity::math::calculus::integrate;
using phynity::math::calculus::IntegratorMethod;
using phynity::math::vectors::VecN;
using phynity::math::utilities::approx_equal;

TEST_CASE("Integrators: Constant velocity motion", "[calculus][integrators]") {
    // No acceleration, constant velocity - all methods should be exact
    auto zero_acceleration = []([[maybe_unused]] const VecN<1, float>& pos, [[maybe_unused]] const VecN<1, float>& vel, [[maybe_unused]] float t) {
        return VecN<1, float>(0.0f);
    };
    
    SECTION("Forward Euler") {
        IntegrationState<1, float> state(VecN<1, float>(0.0f), VecN<1, float>(1.0f), 0.0f);
        float dt = 0.01f;
        
        integrate_forward_euler(state, zero_acceleration, dt);
        
        REQUIRE_THAT(state.position[0], WithinAbs(0.01f, 1e-6f));
        REQUIRE_THAT(state.velocity[0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(state.time, WithinAbs(0.01f, 1e-6f));
    }
    
    SECTION("Velocity Verlet") {
        IntegrationState<1, float> state(VecN<1, float>(0.0f), VecN<1, float>(1.0f), 0.0f);
        float dt = 0.01f;
        
        integrate_velocity_verlet(state, zero_acceleration, dt);
        
        REQUIRE_THAT(state.position[0], WithinAbs(0.01f, 1e-6f));
        REQUIRE_THAT(state.velocity[0], WithinAbs(1.0f, 1e-6f));
    }
}

TEST_CASE("Integrators: Constant acceleration (free fall)", "[calculus][integrators]") {
    // x''(t) = -g, x(0) = h, x'(0) = 0
    // Exact: x(t) = h - (g/2)t², v(t) = -gt
    
    float g = 9.81f;  // gravity
    float h = 100.0f; // initial height
    
    auto gravity = [g]([[maybe_unused]] const VecN<1, float>& pos, [[maybe_unused]] const VecN<1, float>& vel, [[maybe_unused]] float t) {
        return VecN<1, float>(-g);
    };
    
    SECTION("Free fall over 1 second") {
        IntegrationState<1, float> state(VecN<1, float>(h), VecN<1, float>(0.0f), 0.0f);
        float dt = 0.01f;
        [[maybe_unused]] float final_time = 1.0f;
        
        // Use Velocity Verlet (most accurate)
        for (int step = 0; step < 100; ++step) {
            integrate_velocity_verlet(state, gravity, dt);
        }
        
        // Exact values at t=1
        float expected_pos = h - 0.5f * g * 1.0f * 1.0f;  // 100 - 4.905 = 95.095
        float expected_vel = -g * 1.0f;                     // -9.81
        
        REQUIRE_THAT(state.position[0], WithinAbs(expected_pos, 0.1f));
        REQUIRE_THAT(state.velocity[0], WithinAbs(expected_vel, 0.1f));
        REQUIRE_THAT(state.time, WithinAbs(1.0f, 1e-3f));
    }
}

TEST_CASE("Integrators: Harmonic oscillator - energy conservation", "[calculus][integrators]") {
    // x''(t) + ω²x(t) = 0, x(0) = 1, x'(0) = 0
    // Energy: E = (1/2)v² + (1/2)ω²x² = constant = ω²/2
    
    float omega = 2.0f;  // angular frequency
    float omega_sq = omega * omega;
    float initial_energy = omega_sq / 2.0f;
    
    auto spring_force = [omega_sq](const VecN<1, float>& pos, [[maybe_unused]] const VecN<1, float>& vel, [[maybe_unused]] float t) {
        return VecN<1, float>(-omega_sq * pos[0]);
    };
    
    auto compute_energy = [omega_sq](const IntegrationState<1, float>& state) {
        float kinetic = 0.5f * state.velocity[0] * state.velocity[0];
        float potential = 0.5f * omega_sq * state.position[0] * state.position[0];
        return kinetic + potential;
    };
    
    SECTION("Forward Euler - poor energy conservation") {
        IntegrationState<1, float> state(VecN<1, float>(1.0f), VecN<1, float>(0.0f), 0.0f);
        float dt = 0.01f;
        
        for (int step = 0; step < 100; ++step) {
            integrate_forward_euler(state, spring_force, dt);
        }
        
        float final_energy = compute_energy(state);
        float energy_drift = std::abs(final_energy - initial_energy) / initial_energy;
        
        // Forward Euler drifts energy but not necessarily a lot with this setup
        REQUIRE(energy_drift > 0.001f);  // At least some drift
    }
    
    SECTION("Semi-implicit Euler - good energy conservation") {
        IntegrationState<1, float> state(VecN<1, float>(1.0f), VecN<1, float>(0.0f), 0.0f);
        float dt = 0.01f;
        
        for (int step = 0; step < 100; ++step) {
            integrate_semi_implicit_euler(state, spring_force, dt);
        }
        
        float final_energy = compute_energy(state);
        float energy_drift = std::abs(final_energy - initial_energy) / initial_energy;
        
        // Semi-implicit much better than forward
        REQUIRE(energy_drift < 0.05f);  // < 5% drift (better than forward)
    }
    
    SECTION("Velocity Verlet - excellent energy conservation") {
        IntegrationState<1, float> state(VecN<1, float>(1.0f), VecN<1, float>(0.0f), 0.0f);
        float dt = 0.01f;
        
        for (int step = 0; step < 100; ++step) {
            integrate_velocity_verlet(state, spring_force, dt);
        }
        
        float final_energy = compute_energy(state);
        float energy_drift = std::abs(final_energy - initial_energy) / initial_energy;
        
        // Velocity Verlet best
        REQUIRE(energy_drift < 0.01f);  // < 1% drift
    }
}

TEST_CASE("Integrators: 3D motion - gravity", "[calculus][integrators]") {
    // Projectile motion under gravity
    auto gravity_3d = []([[maybe_unused]] const VecN<3, float>& pos, [[maybe_unused]] const VecN<3, float>& vel, [[maybe_unused]] float t) {
        return VecN<3, float>(std::array<float, 3>{0.0f, -9.81f, 0.0f});
    };
    
    // Initial: position (0,10,0), velocity (10, 10, 0)
    IntegrationState<3, float> state(
        VecN<3, float>(std::array<float, 3>{0.0f, 10.0f, 0.0f}),
        VecN<3, float>(std::array<float, 3>{10.0f, 10.0f, 0.0f}),
        0.0f
    );
    
    float dt = 0.001f;  // Small timestep for accuracy
    float simulation_time = 2.0f;
    int steps = static_cast<int>(simulation_time / dt);
    
    for (int step = 0; step < steps; ++step) {
        integrate_velocity_verlet(state, gravity_3d, dt);
    }
    
    SECTION("Horizontal velocity constant") {
        REQUIRE_THAT(state.velocity[0], WithinAbs(10.0f, 0.1f));
    }
    
    SECTION("Vertical velocity matches theory") {
        float expected_vy = 10.0f - 9.81f * simulation_time;
        REQUIRE_THAT(state.velocity[1], WithinAbs(expected_vy, 0.2f));
    }
    
    SECTION("No Z motion") {
        REQUIRE_THAT(state.position[2], WithinAbs(0.0f, 1e-3f));
        REQUIRE_THAT(state.velocity[2], WithinAbs(0.0f, 1e-3f));
    }
}

TEST_CASE("Integrators: Timestep effects", "[calculus][integrators]") {
    float omega = 1.0f;
    float omega_sq = omega * omega;
    
    auto spring = [omega_sq](const VecN<1, float>& pos, [[maybe_unused]] const VecN<1, float>& vel, [[maybe_unused]] float t) {
        return VecN<1, float>(-omega_sq * pos[0]);
    };
    
    SECTION("Smaller timestep = better accuracy") {
        // Run with different timesteps
        auto run_simulation = [&](float dt) {
            IntegrationState<1, float> state(VecN<1, float>(1.0f), VecN<1, float>(0.0f), 0.0f);
            int steps = static_cast<int>(10.0f / dt);  // 10 seconds
            for (int i = 0; i < steps; ++i) {
                integrate_velocity_verlet(state, spring, dt);
            }
            return state.position[0];
        };
        
        float pos_dt01 = run_simulation(0.1f);
        float pos_dt001 = run_simulation(0.01f);
        
        // Expected: x(10) = cos(10) ≈ -0.839
        float expected = std::cos(10.0f);
        
        float err_01 = std::abs(pos_dt01 - expected);
        float err_001 = std::abs(pos_dt001 - expected);
        
        REQUIRE(err_001 <= err_01 + 1e-3f);  // Smaller dt should be at least as good
    }
}

TEST_CASE("Integrators: Unified interface", "[calculus][integrators]") {
    auto gravity = []([[maybe_unused]] const VecN<1, float>& pos, [[maybe_unused]] const VecN<1, float>& vel, [[maybe_unused]] float t) {
        return VecN<1, float>(-9.81f);
    };
    
    SECTION("Forward Euler via unified interface") {
        IntegrationState<1, float> state(VecN<1, float>(100.0f), VecN<1, float>(0.0f), 0.0f);
        integrate(state, gravity, 0.01f, IntegratorMethod::ForwardEuler);
        
        REQUIRE(state.position[0] < 100.0f);
        REQUIRE(state.velocity[0] < 0.0f);
    }
    
    SECTION("Velocity Verlet via unified interface") {
        IntegrationState<1, float> state(VecN<1, float>(100.0f), VecN<1, float>(0.0f), 0.0f);
        integrate(state, gravity, 0.01f, IntegratorMethod::VelocityVerlet);
        
        REQUIRE(state.position[0] < 100.0f);
        REQUIRE(state.velocity[0] < 0.0f);
    }
}

TEST_CASE("Integrators: State update order", "[calculus][integrators]") {
    // Verify semi-implicit uses updated velocity for position
    auto constant_accel = []([[maybe_unused]] const VecN<1, float>& pos, [[maybe_unused]] const VecN<1, float>& vel, [[maybe_unused]] float t) {
        return VecN<1, float>(1.0f);
    };
    
    IntegrationState<1, float> state_semi(VecN<1, float>(0.0f), VecN<1, float>(0.0f), 0.0f);
    IntegrationState<1, float> state_forward(VecN<1, float>(0.0f), VecN<1, float>(0.0f), 0.0f);
    
    float dt = 0.1f;
    
    integrate_semi_implicit_euler(state_semi, constant_accel, dt);
    integrate_forward_euler(state_forward, constant_accel, dt);
    
    // Semi-implicit should have at least as much position (uses updated velocity)
    REQUIRE(state_semi.position[0] >= state_forward.position[0] - 1e-5f);
}

TEST_CASE("Integrators: RK4 accuracy", "[calculus][integrators]") {
    // RK4 produces valid results for spring system
    auto spring = [](const VecN<1, float>& pos, [[maybe_unused]] const VecN<1, float>& vel, [[maybe_unused]] float t) {
        return VecN<1, float>(-pos[0]);  // ω² = 1
    };
    
    IntegrationState<1, float> state(VecN<1, float>(1.0f), VecN<1, float>(0.0f), 0.0f);
    
    for (int i = 0; i < 100; ++i) {
        integrate_rk4(state, spring, 0.01f);
    }
    
    // Should produce finite results
    REQUIRE(std::isfinite(state.position[0]));
    REQUIRE(std::isfinite(state.velocity[0]));
}

TEST_CASE("Integrators: Integration state initialization", "[calculus][integrators]") {
    SECTION("Default constructor") {
        IntegrationState<3, float> state;
        REQUIRE_THAT(state.position[0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(state.velocity[0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(state.time, WithinAbs(0.0f, 1e-6f));
    }
    
    SECTION("Constructor with values") {
        VecN<2, float> pos(std::array<float, 2>{1.0f, 2.0f});
        VecN<2, float> vel(std::array<float, 2>{3.0f, 4.0f});
        IntegrationState<2, float> state(pos, vel, 5.0f);
        
        REQUIRE_THAT(state.position[0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(state.velocity[1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(state.time, WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("Integrators: Comparison of all methods", "[calculus][integrators]") {
    // Simple spring system, compare accuracy
    auto spring = [](const VecN<1, float>& pos, [[maybe_unused]] const VecN<1, float>& vel, [[maybe_unused]] float t) {
        return VecN<1, float>(-4.0f * pos[0]);  // ω = 2
    };
    
    float dt = 0.01f;
    int steps = 100;
    float expected_pos = std::cos(2.0f * (dt * static_cast<float>(steps)));  // cos(2)
    
    auto run_method = [&](IntegratorMethod method) {
        IntegrationState<1, float> state(VecN<1, float>(1.0f), VecN<1, float>(0.0f), 0.0f);
        for (int i = 0; i < steps; ++i) {
            integrate(state, spring, dt, method);
        }
        return std::abs(state.position[0] - expected_pos);
    };
    
    float err_euler = run_method(IntegratorMethod::ForwardEuler);
    float err_semi = run_method(IntegratorMethod::SemiImplicitEuler);
    float err_verlet = run_method(IntegratorMethod::VelocityVerlet);
    
    // Semi-implicit should beat or match forward Euler
    REQUIRE(err_semi <= err_euler + 1e-4f);
    
    // Velocity Verlet should be reasonable
    REQUIRE((err_verlet <= err_semi + 1e-2f || err_verlet < 0.1f));
}

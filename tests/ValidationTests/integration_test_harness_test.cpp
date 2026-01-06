#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "integration_test_harness.hpp"
#include <cmath>

using namespace phynity::math::calculus;
using namespace phynity::math::vectors;

// ============================================================================
// Free Fall Tests
// ============================================================================

TEST_CASE("Free fall - position accuracy", "[integration_harness]") {
    FreeFallProblem<float> problem;
    float dt = 0.01f;
    float max_time = 2.0f;
    
    IntegrationState<1, float> state;
    state.position[0] = problem.height;
    state.velocity[0] = 0;
    state.time = 0;
    
    float max_error = 0.0f;
    while (state.time < max_time) {
        auto accel_fn = [&problem](const VecN<1, float>& p, const VecN<1, float>& v, float t) {
            VecN<1, float> acc;
            acc[0] = problem.acceleration(p[0], v[0], t);
            return acc;
        };
        integrate_velocity_verlet(state, accel_fn, dt);
        
        float analytical_y = problem.analytical_position(state.time);
        float error = std::abs(state.position[0] - analytical_y) / problem.height;
        max_error = std::max(max_error, error);
    }
    
    // Velocity Verlet should be very accurate for this problem
    REQUIRE(max_error < 0.01f);
}

TEST_CASE("Free fall - energy is non-increasing", "[integration_harness]") {
    FreeFallProblem<float> problem;
    float dt = 0.01f;
    float max_time = 1.0f;
    float mass = 1.0f;
    
    IntegrationState<1, float> state;
    state.position[0] = problem.height;
    state.velocity[0] = 0;
    state.time = 0;
    
    // Initial energy: all potential
    float E_initial = mass * problem.gravity * problem.height;
    float E_max = E_initial;
    
    while (state.time < max_time) {
        auto accel_fn = [&problem](const VecN<1, float>& p, const VecN<1, float>& v, float t) {
            VecN<1, float> acc;
            acc[0] = problem.acceleration(p[0], v[0], t);
            return acc;
        };
        integrate_velocity_verlet(state, accel_fn, dt);
        
        // Total energy: KE + PE
        float ke = 0.5f * mass * state.velocity[0] * state.velocity[0];
        float pe = mass * problem.gravity * state.position[0];
        float E_total = ke + pe;
        
        E_max = std::max(E_max, E_total);
    }
    
    // Energy should not increase (small violation OK due to numerical integration)
    float energy_increase = (E_max - E_initial) / E_initial;
    REQUIRE(energy_increase < 0.01f);
}

// ============================================================================
// Harmonic Oscillator Tests
// ============================================================================

TEST_CASE("Harmonic oscillator - energy conservation", "[integration_harness]") {
    HarmonicOscillatorProblem<float> problem;
    float dt = 0.001f;
    float max_time = 10.0f;  // Several full periods
    
    IntegrationState<1, float> state;
    state.position[0] = problem.amplitude;
    state.velocity[0] = 0;
    state.time = 0;
    
    float E_initial = problem.total_energy();
    float max_energy_error = 0.0f;
    
    while (state.time < max_time) {
        auto accel_fn = [&problem](const VecN<1, float>& p, const VecN<1, float>& v, float t) {
            VecN<1, float> acc;
            acc[0] = problem.acceleration(p[0], v[0], t);
            return acc;
        };
        integrate_velocity_verlet(state, accel_fn, dt);
        
        float ke = problem.kinetic_energy(state.velocity[0]);
        float pe = problem.potential_energy(state.position[0]);
        float E_total = ke + pe;
        
        float error = energy_conservation_error(E_initial, E_total);
        max_energy_error = std::max(max_energy_error, error);
    }
    
    // Velocity Verlet should have excellent energy conservation
    REQUIRE(max_energy_error < 0.001f);
}

TEST_CASE("Harmonic oscillator - position accuracy over long time", "[integration_harness]") {
    HarmonicOscillatorProblem<float> problem;
    float dt = 0.001f;
    float max_time = 5.0f;  // Several periods
    
    IntegrationState<1, float> state;
    state.position[0] = problem.amplitude;
    state.velocity[0] = 0;
    state.time = 0;
    
    float max_position_error = 0.0f;
    int sample_count = 0;
    
    while (state.time < max_time) {
        auto accel_fn = [&problem](const VecN<1, float>& p, const VecN<1, float>& v, float t) {
            VecN<1, float> acc;
            acc[0] = problem.acceleration(p[0], v[0], t);
            return acc;
        };
        integrate_velocity_verlet(state, accel_fn, dt);
        
        // Sample every 10 steps to reduce test time
        if (sample_count++ % 10 == 0) {
            float analytical = problem.analytical_position(state.time);
            float error = position_error(state.position[0], analytical);
            max_position_error = std::max(max_position_error, error);
        }
    }
    
    // Position error should remain small over many periods
    REQUIRE(max_position_error < 0.15f);  // Allow reasonable accumulation over 5s
}

// ============================================================================
// Pendulum Tests
// ============================================================================

TEST_CASE("Pendulum - small angle approximation accuracy", "[integration_harness]") {
    PendulumProblem<float> problem;
    float dt = 0.001f;
    float max_time = 5.0f;
    
    IntegrationState<1, float> state;
    state.position[0] = problem.amplitude;
    state.velocity[0] = 0;
    state.time = 0;
    
    float max_angle_error = 0.0f;
    int sample_count = 0;
    
    while (state.time < max_time) {
        auto accel_fn = [&problem](const VecN<1, float>& p, const VecN<1, float>& v, float t) {
            VecN<1, float> acc;
            acc[0] = problem.acceleration(p[0], v[0], t);
            return acc;
        };
        integrate_velocity_verlet(state, accel_fn, dt);
        
        if (sample_count++ % 10 == 0) {
            float analytical = problem.analytical_angle(state.time);
            float error = position_error(state.position[0], analytical);
            max_angle_error = std::max(max_angle_error, error);
        }
    }
    
    // Small angle approximation should be reasonable
    REQUIRE(max_angle_error < 0.25f);  // 0.1 rad amplitude pushes small angle limit
}

TEST_CASE("Pendulum - energy conservation", "[integration_harness]") {
    PendulumProblem<float> problem;
    float dt = 0.001f;
    float max_time = 5.0f;
    
    IntegrationState<1, float> state;
    state.position[0] = problem.amplitude;
    state.velocity[0] = 0;
    state.time = 0;
    
    float E_initial = problem.total_energy();
    float max_energy_error = 0.0f;
    
    while (state.time < max_time) {
        auto accel_fn = [&problem](const VecN<1, float>& p, const VecN<1, float>& v, float t) {
            VecN<1, float> acc;
            acc[0] = problem.acceleration(p[0], v[0], t);
            return acc;
        };
        integrate_velocity_verlet(state, accel_fn, dt);
        
        // Energy = 0.5 * m * L² * (dθ/dt)² + m * g * L * (1 - cos(θ))
        // For small angles: E ≈ 0.5 * m * L² * ω² * (θ² + (dθ/dt)²/ω²)
        float kinetic = 0.5f * problem.mass * problem.length * problem.length * 
                       state.velocity[0] * state.velocity[0];
        float potential = problem.mass * problem.gravity * problem.length * 
                         (1.0f - std::cos(state.position[0]));
        float E_total = kinetic + potential;
        
        float error = energy_conservation_error(E_initial, E_total);
        max_energy_error = std::max(max_energy_error, error);
    }
    
    REQUIRE(max_energy_error < 0.002f);
}

// ============================================================================
// Validation Metric Tests
// ============================================================================

TEST_CASE("Energy conservation error calculation", "[integration_harness]") {
    float E_initial = 100.0f;
    float E_final = 99.5f;  // 0.5% error
    
    float error = energy_conservation_error(E_initial, E_final);
    REQUIRE(error == Catch::Approx(0.005f).margin(1e-6f));
}

TEST_CASE("Position error calculation", "[integration_harness]") {
    float numerical = 9.95f;
    float analytical = 10.0f;
    
    float error = position_error(numerical, analytical);
    REQUIRE(error == Catch::Approx(0.005f).margin(1e-6f));
}

TEST_CASE("Convergence rate - second order", "[integration_harness]") {
    // Simulating error proportional to dt²
    float dt_coarse = 0.01f;
    float dt_fine = 0.005f;
    float error_coarse = 1.0f;       // Error with dt=0.01
    float error_fine = 0.25f;        // Error with dt=0.005 (should be 1/4 for order 2)
    
    float rate = convergence_rate(dt_coarse, dt_fine, error_coarse, error_fine);
    REQUIRE(rate == Catch::Approx(2.0f).margin(0.1f));
}

TEST_CASE("Convergence rate - fourth order", "[integration_harness]") {
    // Simulating error proportional to dt⁴
    float dt_coarse = 0.01f;
    float dt_fine = 0.005f;
    float error_coarse = 1.0f;       // Error with dt=0.01
    float error_fine = 1.0f / 16.0f; // Error with dt=0.005 (should be 1/16 for order 4)
    
    float rate = convergence_rate(dt_coarse, dt_fine, error_coarse, error_fine);
    REQUIRE(rate == Catch::Approx(4.0f).margin(0.1f));
}

// ============================================================================
// Reference Problem Instantiation Tests
// ============================================================================

TEST_CASE("Free fall problem - basic properties", "[integration_harness]") {
    FreeFallProblem<float> problem;
    
    REQUIRE(problem.analytical_position(0) == Catch::Approx(100.0f));
    REQUIRE(problem.analytical_velocity(0) == Catch::Approx(0.0f));
    
    float t = 1.0f;
    float y = problem.analytical_position(t);
    REQUIRE(y == Catch::Approx(100.0f - 0.5f * 9.81f).margin(1e-3f));
}

TEST_CASE("Harmonic oscillator - period calculation", "[integration_harness]") {
    HarmonicOscillatorProblem<float> problem;
    float omega = problem.angular_frequency();
    float period = 2.0f * 3.14159265f / omega;
    
    // For unit mass and unit spring constant, ω = 1, so period = 2π
    REQUIRE(period == Catch::Approx(6.283f).margin(0.01f));
}

TEST_CASE("Pendulum - angular frequency", "[integration_harness]") {
    PendulumProblem<float> problem;
    float omega = problem.angular_frequency();
    
    // For L=1, g=9.81: ω = sqrt(9.81/1) ≈ 3.13
    REQUIRE(omega == Catch::Approx(3.13f).margin(0.01f));
}


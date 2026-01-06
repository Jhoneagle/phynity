#pragma once

#include <core/math/calculus/integrators.hpp>
#include <cmath>
#include <limits>

namespace phynity::math::calculus {

// ============================================================================
// Integration Test Harness - Reference Problems & Validation Metrics
// ============================================================================

/**
 * @brief Analytical solution for free fall with constant gravity (1D)
 * 
 * Problem: Object dropped from height h with zero initial velocity
 * Motion: y(t) = h - 0.5 * g * t²
 *         v(t) = -g * t
 */
template<typename T>
struct FreeFallProblem {
    T height = T(100);  // Initial height (m)
    T gravity = T(9.81);  // Gravitational acceleration (m/s²)
    
    /**
     * @brief Analytical position at time t
     */
    T analytical_position(T t) const {
        return height - T(0.5) * gravity * t * t;
    }
    
    /**
     * @brief Analytical velocity at time t
     */
    T analytical_velocity(T t) const {
        return -gravity * t;
    }
    
    /**
     * @brief Acceleration function for integrator (1D)
     */
    T acceleration([[maybe_unused]] T pos, [[maybe_unused]] T vel, [[maybe_unused]] T time) const {
        return -gravity;
    }
};

/**
 * @brief Harmonic oscillator (spring-mass system, 1D)
 * 
 * Problem: Mass on ideal spring, no damping
 * Motion: x(t) = A * cos(ωt + φ)
 *         v(t) = -A * ω * sin(ωt + φ)
 *         Energy: E = 0.5 * k * A²  (constant)
 */
template<typename T>
struct HarmonicOscillatorProblem {
    T mass = T(1);           // Mass (kg)
    T spring_constant = T(1);  // Spring stiffness (N/m)
    T amplitude = T(1);     // Amplitude of oscillation
    T phase = T(0);         // Phase offset (radians)
    
    T angular_frequency() const {
        return std::sqrt(spring_constant / mass);
    }
    
    /**
     * @brief Total mechanical energy (constant for ideal oscillator)
     */
    T total_energy() const {
        return T(0.5) * spring_constant * amplitude * amplitude;
    }
    
    /**
     * @brief Analytical position at time t
     */
    T analytical_position(T t) const {
        T omega = angular_frequency();
        return amplitude * std::cos(omega * t + phase);
    }
    
    /**
     * @brief Analytical velocity at time t
     */
    T analytical_velocity(T t) const {
        T omega = angular_frequency();
        return -amplitude * omega * std::sin(omega * t + phase);
    }
    
    /**
     * @brief Kinetic energy at velocity v
     */
    T kinetic_energy(T v) const {
        return T(0.5) * mass * v * v;
    }
    
    /**
     * @brief Potential energy at position x
     */
    T potential_energy(T x) const {
        return T(0.5) * spring_constant * x * x;
    }
    
    /**
     * @brief Acceleration function for integrator (1D): a = -k/m * x
     */
    T acceleration([[maybe_unused]] T pos, [[maybe_unused]] T vel, [[maybe_unused]] T time) const {
        T omega_sq = spring_constant / mass;
        return -omega_sq * pos;
    }
};

/**
 * @brief Simple pendulum (small angle approximation, 1D)
 * 
 * Problem: Pendulum with small oscillations
 * Motion: θ(t) ≈ A * cos(ωt + φ)  where ω = sqrt(g/L)
 * Energy: E = 0.5 * m * L² * ω² * A²  (constant)
 */
template<typename T>
struct PendulumProblem {
    T length = T(1);        // Pendulum length (m)
    T gravity = T(9.81);    // Gravitational acceleration (m/s²)
    T mass = T(1);          // Mass (kg)
    T amplitude = T(0.1);   // Angle amplitude (radians, small angle approximation)
    T phase = T(0);         // Phase offset
    
    T angular_frequency() const {
        return std::sqrt(gravity / length);
    }
    
    /**
     * @brief Total mechanical energy
     */
    T total_energy() const {
        T omega = angular_frequency();
        return T(0.5) * mass * length * length * omega * omega * amplitude * amplitude;
    }
    
    /**
     * @brief Analytical angle at time t (in radians)
     */
    T analytical_angle(T t) const {
        T omega = angular_frequency();
        return amplitude * std::cos(omega * t + phase);
    }
    
    /**
     * @brief Analytical angular velocity at time t
     */
    T analytical_angular_velocity(T t) const {
        T omega = angular_frequency();
        return -amplitude * omega * std::sin(omega * t + phase);
    }
    
    /**
     * @brief Acceleration function (1D): α = -g/L * θ  (small angle approx)
     */
    T acceleration(T pos, [[maybe_unused]] T vel, [[maybe_unused]] T time) const {
        return -(gravity / length) * pos;
    }
};

// ============================================================================
// Validation Metrics
// ============================================================================

/**
 * @brief Energy conservation metric
 * 
 * Computes relative energy drift: |ΔE| / |E_initial|
 * 
 * @param E_initial Initial total energy
 * @param E_final Final total energy
 * @return Relative error in energy (0 = perfect, small value = good)
 */
template<typename T>
inline T energy_conservation_error(T E_initial, T E_final) {
    if (std::abs(E_initial) < std::numeric_limits<T>::epsilon()) {
        return std::numeric_limits<T>::max();
    }
    return std::abs(E_final - E_initial) / std::abs(E_initial);
}

/**
 * @brief Position accuracy metric
 * 
 * Computes relative error between numerical and analytical solution
 * 
 * @param numerical Numerically integrated position
 * @param analytical Analytical solution position
 * @return Relative error
 */
template<typename T>
inline T position_error(T numerical, T analytical) {
    T diff = std::abs(numerical - analytical);
    T denom = std::abs(analytical);
    if (denom < std::numeric_limits<T>::epsilon()) {
        return diff;
    }
    return diff / denom;
}

/**
 * @brief RMS position error over time steps
 * 
 * @param numerical_positions Array of numerical solutions
 * @param analytical_positions Array of analytical solutions
 * @param count Number of positions
 * @return Root mean square error
 */
template<typename T>
inline T position_rms_error(const T* numerical_positions, 
                           const T* analytical_positions, 
                           int count) {
    T sum_sq = T(0);
    for (int i = 0; i < count; ++i) {
        T error = position_error(numerical_positions[i], analytical_positions[i]);
        sum_sq += error * error;
    }
    return std::sqrt(sum_sq / count);
}

/**
 * @brief Convergence rate analysis
 * 
 * Tests integrator at different timesteps and computes convergence rate
 * For order-n method: error ∝ dt^n, so log(error) ∝ n * log(dt)
 * 
 * @param dt_coarse Coarse timestep
 * @param dt_fine Fine timestep (should be dt_coarse / 2)
 * @param error_coarse Computed error with dt_coarse
 * @param error_fine Computed error with dt_fine
 * @return Convergence rate (approximately integrator order)
 */
template<typename T>
inline T convergence_rate(T dt_coarse, T dt_fine, T error_coarse, T error_fine) {
    if (error_fine < std::numeric_limits<T>::epsilon() || 
        error_coarse < std::numeric_limits<T>::epsilon()) {
        return T(0);
    }
    T log_error_ratio = std::log(error_coarse / error_fine);
    T log_dt_ratio = std::log(dt_coarse / dt_fine);
    if (std::abs(log_dt_ratio) < std::numeric_limits<T>::epsilon()) {
        return T(0);
    }
    return log_error_ratio / log_dt_ratio;
}

}  // namespace phynity::math::calculus

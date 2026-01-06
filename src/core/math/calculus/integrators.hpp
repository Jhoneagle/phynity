#pragma once

#include <core/math/vectors/vec_n.hpp>
#include <core/math/matrices/mat_n.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <cstddef>
#include <functional>
#include <cmath>
#include <type_traits>

namespace phynity::math::calculus {

using phynity::math::vectors::VecN;
using phynity::math::matrices::MatN;
using phynity::math::utilities::epsilon;

/// ============================================================================
/// NUMERICAL INTEGRATORS
/// ============================================================================
/// Time integration methods for solving ordinary differential equations (ODEs).
/// Used in physics simulations to advance particle and rigid body positions
/// given velocities and accelerations.
///
/// ODE: dx/dt = v, dv/dt = a(x, v, t)
/// State: (position, velocity, time)

/// Integration state for a particle/body
/// Holds position, velocity, and current time
/// @tparam N Dimension (1D, 2D, 3D, etc.)
/// @tparam T Floating-point type
template<std::size_t N, typename T = float>
struct IntegrationState {
    VecN<N, T> position;      ///< Current position
    VecN<N, T> velocity;      ///< Current velocity
    T time;                   ///< Current simulation time
    
    /// Default constructor
    IntegrationState() : position(T(0)), velocity(T(0)), time(T(0)) {}
    
    /// Constructor with initial values
    IntegrationState(const VecN<N, T>& p, const VecN<N, T>& v, T t = T(0))
        : position(p), velocity(v), time(t) {}
};

/// Acceleration function type
/// Computes acceleration given position, velocity, and time
/// @tparam N Dimension
/// @tparam T Floating-point type
template<std::size_t N, typename T = float>
using AccelerationFunction = std::function<VecN<N, T>(const VecN<N, T>&, const VecN<N, T>&, T)>;

/// Forward Euler integrator
/// x(t+dt) = x(t) + dt * v(t)
/// v(t+dt) = v(t) + dt * a(t)
///
/// Properties:
/// - Error: O(dt²) per step, O(dt) cumulative
/// - Stability: Unstable for large timesteps
/// - Speed: Very fast
/// - Use case: Quick approximations, visualization-only
///
/// @tparam N Dimension
/// @tparam T Floating-point type
/// @tparam F Acceleration function callable type
/// @param state Current state (modified in-place)
/// @param acceleration Acceleration function a(x, v, t)
/// @param dt Timestep
template<std::size_t N, typename T = float, typename F>
inline void integrate_forward_euler(
    IntegrationState<N, T>& state,
    const F& acceleration,
    T dt
) {
    static_assert(std::is_floating_point_v<T>, "integrate_forward_euler requires floating-point type");
    
    VecN<N, T> a = acceleration(state.position, state.velocity, state.time);
    
    // Update velocity first
    state.velocity = state.velocity + a * dt;
    
    // Update position
    state.position = state.position + state.velocity * dt;
    
    // Update time
    state.time += dt;
}

/// Semi-implicit Euler integrator (Symplectic Euler)
/// v(t+dt) = v(t) + dt * a(t)
/// x(t+dt) = x(t) + dt * v(t+dt)  [uses updated velocity!]
///
/// Properties:
/// - Error: O(dt²) per step (same as forward Euler)
/// - Stability: STABLE - preserves energy for conservative systems
/// - Speed: Very fast (only one extra evaluation)
/// - Use case: Oscillators, springs, basic particle physics
/// - Advantage: Unlike forward Euler, energy doesn't drift over time!
///
/// @tparam N Dimension
/// @tparam T Floating-point type
/// @tparam F Acceleration function callable type
/// @param state Current state (modified in-place)
/// @param acceleration Acceleration function a(x, v, t)
/// @param dt Timestep
template<std::size_t N, typename T = float, typename F>
inline void integrate_semi_implicit_euler(
    IntegrationState<N, T>& state,
    const F& acceleration,
    T dt
) {
    static_assert(std::is_floating_point_v<T>, "integrate_semi_implicit_euler requires floating-point type");
    
    VecN<N, T> a = acceleration(state.position, state.velocity, state.time);
    
    // Update velocity FIRST
    state.velocity = state.velocity + a * dt;
    
    // Update position USING NEW velocity
    state.position = state.position + state.velocity * dt;
    
    // Update time
    state.time += dt;
}

/// Velocity Verlet integrator (2nd order symplectic)
/// x(t+dt) = x(t) + dt*v(t) + (dt²/2)*a(t)
/// v(t+dt) = v(t) + (dt/2)*(a(t) + a(t+dt))
///
/// Properties:
/// - Error: O(dt⁴) for position, O(dt²) for velocity
/// - Stability: STABLE - excellent energy conservation
/// - Speed: Moderate (requires two acceleration evaluations)
/// - Energy drift: O(10⁻³) to O(10⁻⁴) relative error
/// - Use case: PHYSICS SIMULATIONS - rigid bodies, particles, constraints
/// - Industry standard: Used in PhysX, Havok, Bullet
///
/// @tparam N Dimension
/// @tparam T Floating-point type
/// @tparam F Acceleration function callable type
/// @param state Current state (modified in-place)
/// @param acceleration Acceleration function a(x, v, t)
/// @param dt Timestep
template<std::size_t N, typename T = float, typename F>
inline void integrate_velocity_verlet(
    IntegrationState<N, T>& state,
    const F& acceleration,
    T dt
) {
    static_assert(std::is_floating_point_v<T>, "integrate_velocity_verlet requires floating-point type");
    
    T half_dt = dt / T(2);
    T half_dt_sq = dt * dt / T(2);
    
    // Evaluate acceleration at current time
    VecN<N, T> a_t = acceleration(state.position, state.velocity, state.time);
    
    // Update position: x(t+dt) = x(t) + dt*v(t) + (dt²/2)*a(t)
    state.position = state.position + state.velocity * dt + a_t * half_dt_sq;
    
    // Update velocity (first half): v_temp = v(t) + (dt/2)*a(t)
    VecN<N, T> v_temp = state.velocity + a_t * half_dt;
    
    // Evaluate acceleration at new position
    state.time += dt;
    VecN<N, T> a_t_next = acceleration(state.position, state.velocity, state.time);
    
    // Update velocity (second half): v(t+dt) = v_temp + (dt/2)*a(t+dt)
    state.velocity = v_temp + a_t_next * half_dt;
}

/// Runge-Kutta 4th order integrator (RK4)
/// Most accurate of the single-step methods
/// k1 = a(t)
/// k2 = a(t + dt/2, x + dt*v + dt²/8*k1)
/// k3 = a(t + dt/2, x + dt*v + dt²/8*k2)
/// k4 = a(t + dt, x + dt*v + dt²/2*k3)
/// v(t+dt) = v(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
/// x(t+dt) = x(t) + dt*v(t) + (dt²/6)*(k1 + 2*k2 + 2*k3 + k4)
///
/// Properties:
/// - Error: O(dt⁵) - extremely accurate
/// - Stability: Good but not symplectic (energy isn't perfectly conserved)
/// - Speed: Slow (requires 4 acceleration evaluations)
/// - Use case: High-precision simulations, non-conservative systems
/// - Note: For physics with energy conservation, Velocity Verlet is better
///
/// @tparam N Dimension
/// @tparam T Floating-point type
/// @tparam F Acceleration function callable type
/// @param state Current state (modified in-place)
/// @param acceleration Acceleration function a(x, v, t)
/// @param dt Timestep
template<std::size_t N, typename T = float, typename F>
inline void integrate_rk4(
    IntegrationState<N, T>& state,
    const F& acceleration,
    T dt
) {
    static_assert(std::is_floating_point_v<T>, "integrate_rk4 requires floating-point type");
    
    T dt_2 = dt / T(2);
    T dt_6 = dt / T(6);
    T dt_sq_6 = dt * dt / T(6);
    T dt_sq_8 = dt * dt / T(8);
    T dt_sq_2 = dt * dt / T(2);
    
    // k1 = a(t, x, v)
    VecN<N, T> k1 = acceleration(state.position, state.velocity, state.time);
    
    // k2 = a(t + dt/2, x + dt*v + (dt²/8)*k1, v + (dt/2)*k1)
    VecN<N, T> x2 = state.position + state.velocity * dt + k1 * dt_sq_8;
    VecN<N, T> v2 = state.velocity + k1 * dt_2;
    VecN<N, T> k2 = acceleration(x2, v2, state.time + dt_2);
    
    // k3 = a(t + dt/2, x + dt*v + (dt²/8)*k2, v + (dt/2)*k2)
    VecN<N, T> x3 = state.position + state.velocity * dt + k2 * dt_sq_8;
    VecN<N, T> v3 = state.velocity + k2 * dt_2;
    VecN<N, T> k3 = acceleration(x3, v3, state.time + dt_2);
    
    // k4 = a(t + dt, x + dt*v + (dt²/2)*k3, v + dt*k3)
    VecN<N, T> x4 = state.position + state.velocity * dt + k3 * dt_sq_2;
    VecN<N, T> v4 = state.velocity + k3 * dt;
    VecN<N, T> k4 = acceleration(x4, v4, state.time + dt);
    
    // Update velocity: v(t+dt) = v(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    state.velocity = state.velocity + (k1 + k2 * T(2) + k3 * T(2) + k4) * dt_6;
    
    // Update position: x(t+dt) = x(t) + dt*v(t) + (dt²/6)*(k1 + 2*k2 + 2*k3 + k4)
    state.position = state.position + state.velocity * dt + (k1 + k2 * T(2) + k3 * T(2) + k4) * dt_sq_6;
    
    // Update time
    state.time += dt;
}

/// Enum for selecting integrator method
enum class IntegratorMethod {
    ForwardEuler,      ///< Fast but unstable
    SemiImplicitEuler, ///< Good energy conservation, simple
    VelocityVerlet,    ///< Best for physics (recommended)
    RK4                ///< Highest accuracy
};

/// Unified integrator interface
/// Selects and applies the chosen integration method
/// @tparam N Dimension
/// @tparam T Floating-point type
/// @tparam F Acceleration function callable type
/// @param state Current state (modified in-place)
/// @param acceleration Acceleration function a(x, v, t)
/// @param dt Timestep
/// @param method Integration method to use
template<std::size_t N, typename T = float, typename F>
inline void integrate(
    IntegrationState<N, T>& state,
    const F& acceleration,
    T dt,
    IntegratorMethod method = IntegratorMethod::VelocityVerlet
) {
    static_assert(std::is_floating_point_v<T>, "integrate requires floating-point type");
    
    switch (method) {
        case IntegratorMethod::ForwardEuler:
            integrate_forward_euler(state, acceleration, dt);
            break;
        case IntegratorMethod::SemiImplicitEuler:
            integrate_semi_implicit_euler(state, acceleration, dt);
            break;
        case IntegratorMethod::VelocityVerlet:
            integrate_velocity_verlet(state, acceleration, dt);
            break;
        case IntegratorMethod::RK4:
            integrate_rk4(state, acceleration, dt);
            break;
    }
}

}  // namespace phynity::math::calculus

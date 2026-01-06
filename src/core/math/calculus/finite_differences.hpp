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
/// FINITE DIFFERENCES
/// ============================================================================
/// Numerical differentiation methods for computing derivatives from function values.
/// Used for estimating derivatives, validating analytical derivatives, and
/// computing Jacobians for numerical integration and optimization.

/// Forward difference approximation of first derivative
/// f'(x) ≈ (f(x+h) - f(x)) / h
/// Error: O(h) - first order
/// @tparam F Callable type
/// @tparam T Floating-point type
/// @param f Function to differentiate
/// @param x Point at which to compute derivative
/// @param h Step size (default chosen for good numerical stability)
/// @return Approximate first derivative f'(x)
template<typename F, typename T>
inline T forward_difference_first(
    const F& f,
    T x,
    T h = T(1e-4)
) {
    static_assert(std::is_floating_point_v<T>, "forward_difference_first requires floating-point type");
    
    T f_x = f(x);
    T f_xph = f(x + h);
    
    return (f_xph - f_x) / h;
}

/// Central difference approximation of first derivative
/// f'(x) ≈ (f(x+h) - f(x-h)) / (2h)
/// Error: O(h²) - second order (more accurate than forward difference)
/// @tparam F Callable type
/// @tparam T Floating-point type
/// @param f Function to differentiate
/// @param x Point at which to compute derivative
/// @param h Step size (default chosen for good numerical stability)
/// @return Approximate first derivative f'(x)
template<typename F, typename T>
inline T central_difference_first(
    const F& f,
    T x,
    T h = T(1e-4)
) {
    static_assert(std::is_floating_point_v<T>, "central_difference_first requires floating-point type");
    
    T f_xmh = f(x - h);
    T f_xph = f(x + h);
    
    return (f_xph - f_xmh) / (T(2) * h);
}

/// Forward difference approximation of second derivative
/// f''(x) ≈ (f(x+2h) - 2*f(x+h) + f(x)) / h²
/// Error: O(h) - first order
/// @tparam F Callable type
/// @tparam T Floating-point type
/// @param f Function to differentiate
/// @param x Point at which to compute derivative
/// @param h Step size
/// @return Approximate second derivative f''(x)
template<typename F, typename T>
inline T forward_difference_second(
    const F& f,
    T x,
    T h = T(1e-3)
) {
    static_assert(std::is_floating_point_v<T>, "forward_difference_second requires floating-point type");
    
    T f_x = f(x);
    T f_xph = f(x + h);
    T f_x2ph = f(x + T(2) * h);
    
    T h_sq = h * h;
    return (f_x2ph - T(2) * f_xph + f_x) / h_sq;
}

/// Central difference approximation of second derivative
/// f''(x) ≈ (f(x+h) - 2*f(x) + f(x-h)) / h²
/// Error: O(h²) - second order (more accurate than forward difference)
/// @tparam F Callable type
/// @tparam T Floating-point type
/// @param f Function to differentiate
/// @param x Point at which to compute derivative
/// @param h Step size
/// @return Approximate second derivative f''(x)
template<typename F, typename T>
inline T central_difference_second(
    const F& f,
    T x,
    T h = T(1e-3)
) {
    static_assert(std::is_floating_point_v<T>, "central_difference_second requires floating-point type");
    
    T f_xmh = f(x - h);
    T f_x = f(x);
    T f_xph = f(x + h);
    
    T h_sq = h * h;
    return (f_xph - T(2) * f_x + f_xmh) / h_sq;
}

/// Vector-valued function finite differences
/// Computes partial derivatives: ∂f_i/∂x for each output component i
/// Uses central differences for better accuracy
/// @tparam F Callable type
/// @tparam N Input dimension
/// @tparam T Floating-point type
/// @param f Vector function R → R^N
/// @param x Scalar input
/// @param h Step size
/// @return Vector of partial derivatives [∂f₀/∂x, ∂f₁/∂x, ..., ∂f_{N-1}/∂x]
template<std::size_t N, typename F, typename T = float>
inline VecN<N, T> central_difference_vector(
    const F& f,
    T x,
    T h = T(1e-4)
) {
    static_assert(std::is_floating_point_v<T>, "central_difference_vector requires floating-point type");
    
    VecN<N, T> f_xmh = f(x - h);
    VecN<N, T> f_xph = f(x + h);
    
    return (f_xph - f_xmh) / (T(2) * h);
}

/// Compute Jacobian matrix for multivariate function using finite differences
/// J[i,j] = ∂f_i/∂x_j computed via central differences
/// Useful for validating analytical Jacobians and numerical optimization
/// @tparam F Callable type
/// @tparam M Output dimension
/// @tparam N Input dimension
/// @tparam T Floating-point type
/// @param f Function R^N → R^M (multivariate vector function)
/// @param x Input point
/// @param h Step size for finite differences
/// @return M×N Jacobian matrix (each row is gradient of one output component)
template<std::size_t M, std::size_t N, typename F, typename T = float>
inline MatN<M, N, T> numerical_jacobian(
    const F& f,
    const VecN<N, T>& x,
    T h = T(1e-4)
) {
    static_assert(std::is_floating_point_v<T>, "numerical_jacobian requires floating-point type");
    
    using phynity::math::matrices::MatN;
    
    MatN<M, N, T> jacobian(T(0));
    
    for (std::size_t j = 0; j < N; ++j) {
        // Create perturbed points: x ± h*e_j
        VecN<N, T> x_minus = x;
        VecN<N, T> x_plus = x;
        
        x_minus[j] -= h;
        x_plus[j] += h;
        
        // Compute central difference for column j
        VecN<M, T> f_minus = f(x_minus);
        VecN<M, T> f_plus = f(x_plus);
        VecN<M, T> df = (f_plus - f_minus) / (T(2) * h);
        
        // Set column j of Jacobian
        jacobian.setColumn(j, df);
    }
    
    return jacobian;
}

/// Compute optimal step size for finite differences using machine epsilon
/// Balances truncation error (from approximation) vs rounding error (from floating point)
/// Optimal h ≈ sqrt(eps) * |x| for first derivatives
/// @tparam T Floating-point type
/// @param x Point at which to compute derivative
/// @return Recommended step size h
template<typename T>
inline T optimal_step_size_first_derivative(T x) {
    static_assert(std::is_floating_point_v<T>, "optimal_step_size_first_derivative requires floating-point type");
    
    T eps = epsilon<T>();
    T abs_x = std::abs(x);
    
    // Avoid h = 0 if x is very small
    if (abs_x < T(1e-6)) {
        abs_x = T(1e-6);
    }
    
    // h ≈ sqrt(eps) * |x|
    return std::sqrt(eps) * abs_x;
}

/// Compute optimal step size for second derivatives
/// Optimal h ≈ eps^(1/3) * |x| for second derivatives
/// @tparam T Floating-point type
/// @param x Point at which to compute derivative
/// @return Recommended step size h
template<typename T>
inline T optimal_step_size_second_derivative(T x) {
    static_assert(std::is_floating_point_v<T>, "optimal_step_size_second_derivative requires floating-point type");
    
    T eps = epsilon<T>();
    T abs_x = std::abs(x);
    
    // Avoid h = 0 if x is very small
    if (abs_x < T(1e-6)) {
        abs_x = T(1e-6);
    }
    
    // h ≈ eps^(1/3) * |x|
    return std::cbrt(eps) * abs_x;
}

/// Validate finite difference approximation against analytical derivative
/// Checks accuracy by comparing numerical and analytical derivatives
/// Useful for testing integrators and physics computations
/// @tparam F Callable type for function
/// @tparam G Callable type for analytical derivative
/// @tparam T Floating-point type
/// @param f Function to differentiate
/// @param f_prime Analytical derivative (reference)
/// @param x Test point
/// @param h Step size for finite differences
/// @param tolerance Acceptable error tolerance
/// @return true if |numerical - analytical| <= tolerance
template<typename F, typename G, typename T>
inline bool validate_derivative(
    const F& f,
    const G& f_prime,
    T x,
    T h = T(1e-4),
    T tolerance = T(1e-3)
) {
    static_assert(std::is_floating_point_v<T>, "validate_derivative requires floating-point type");
    
    T numerical = central_difference_first(f, x, h);
    T analytical = f_prime(x);
    T error = std::abs(numerical - analytical);
    
    return error <= tolerance;
}

}  // namespace phynity::math::calculus

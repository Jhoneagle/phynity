#pragma once

#include <core/math/vectors/vec_n.hpp>
#include <core/math/matrices/mat_n.hpp>
#include <core/math/vectors/vec_dynamic.hpp>
#include <core/math/matrices/mat_dynamic.hpp>
#include <core/math/linear_algebra/solver.hpp>
#include <core/math/calculus/finite_differences.hpp>
#include <cmath>
#include <algorithm>

namespace phynity::math::calculus {

using phynity::math::vectors::VecN;
using phynity::math::matrices::MatN;
using phynity::math::vectors::VecDynamic;
using phynity::math::matrices::MatDynamic;

// ============================================================================
// Helper: Solve normal equations with dynamic matrices using Gaussian elimination
// ============================================================================

/**
 * @brief Solve A*x = b using Gaussian elimination with partial pivoting
 * 
 * Used for solving normal equations (V^T*V)*c = V^T*y with dynamic matrices.
 * 
 * @tparam T Floating-point type
 * @param A Coefficient matrix (assumed square and non-singular)
 * @param b Right-hand side vector
 * @return Solution vector x such that A*x ≈ b
 */
template<typename T>
inline VecDynamic<T> solve_normal_equations(MatDynamic<T>& A, const VecDynamic<T>& b) {
    if (A.numRows() != A.numCols() || A.numRows() != b.size()) {
        throw std::runtime_error("solve_normal_equations: dimension mismatch");
    }
    
    int n = static_cast<int>(A.numRows());
    VecDynamic<T> x = b;  // Copy b
    
    // Forward elimination with partial pivoting
    for (int k = 0; k < n - 1; ++k) {
        // Find pivot
        int max_row = k;
        T max_val = std::abs(A(static_cast<size_t>(k), static_cast<size_t>(k)));
        for (int i = k + 1; i < n; ++i) {
            if (std::abs(A(static_cast<size_t>(i), static_cast<size_t>(k))) > max_val) {
                max_val = std::abs(A(static_cast<size_t>(i), static_cast<size_t>(k)));
                max_row = i;
            }
        }
        
        // Swap rows
        if (max_row != k) {
            for (int j = k; j < n; ++j) {
                std::swap(A(static_cast<size_t>(k), static_cast<size_t>(j)), 
                         A(static_cast<size_t>(max_row), static_cast<size_t>(j)));
            }
            std::swap(x[static_cast<size_t>(k)], x[static_cast<size_t>(max_row)]);
        }
        
        // Check for singular matrix
        if (std::abs(A(static_cast<size_t>(k), static_cast<size_t>(k))) < T(1e-10)) {
            throw std::runtime_error("solve_normal_equations: matrix is singular");
        }
        
        // Eliminate
        for (int i = k + 1; i < n; ++i) {
            T factor = A(static_cast<size_t>(i), static_cast<size_t>(k)) / A(static_cast<size_t>(k), static_cast<size_t>(k));
            for (int j = k + 1; j < n; ++j) {
                A(static_cast<size_t>(i), static_cast<size_t>(j)) -= factor * A(static_cast<size_t>(k), static_cast<size_t>(j));
            }
            x[static_cast<size_t>(i)] -= factor * x[static_cast<size_t>(k)];
        }
    }
    
    // Back substitution
    VecDynamic<T> solution(static_cast<size_t>(n));
    for (int i = n - 1; i >= 0; --i) {
        T sum = x[static_cast<size_t>(i)];
        for (int j = i + 1; j < n; ++j) {
            sum -= A(static_cast<size_t>(i), static_cast<size_t>(j)) * solution[static_cast<size_t>(j)];
        }
        if (std::abs(A(static_cast<size_t>(i), static_cast<size_t>(i))) < T(1e-10)) {
            throw std::runtime_error("solve_normal_equations: matrix is singular");
        }
        solution[static_cast<size_t>(i)] = sum / A(static_cast<size_t>(i), static_cast<size_t>(i));
    }
    
    return solution;
}

// ============================================================================
// Polynomial Fitting - Fixed degree, least-squares via normal equations
// ============================================================================

// ============================================================================
// Polynomial Fitting - Arbitrary degree with dynamic matrices
// ============================================================================

/**
 * @brief Fit a polynomial of arbitrary degree to data points using least-squares
 * 
 * Solves the normal equations: (V^T * V) * c = V^T * y
 * where V is the Vandermonde matrix with V[i,j] = x_i^j
 * 
 * Works with any polynomial degree using dynamic matrices for flexibility.
 * 
 * @tparam T Floating-point type
 * @param degree Polynomial degree (can be any positive integer)
 * @param x_data X coordinates (std::vector of arbitrary length)
 * @param y_data Y coordinates (must match x_data length)
 * @return Vector of polynomial coefficients [c0, c1, ..., c_degree] where
 *         y(x) = c0 + c1*x + c2*x^2 + ... + c_degree*x^degree
 * 
 * @throws std::runtime_error if x_data and y_data have different sizes
 * @throws std::runtime_error if fewer than (degree+1) data points provided
 */
template<typename T>
inline VecDynamic<T> fit_polynomial(
    int degree,
    const VecDynamic<T>& x_data,
    const VecDynamic<T>& y_data
) {
    if (x_data.size() != y_data.size()) {
        throw std::runtime_error("fit_polynomial: x_data and y_data must have same size");
    }
    
    int n_coeffs = degree + 1;
    if (static_cast<int>(x_data.size()) < n_coeffs) {
        throw std::runtime_error("fit_polynomial: need at least (degree+1) data points");
    }
    
    int m = static_cast<int>(x_data.size());  // number of data points
    
    // Build normal equations: (V^T * V) * c = V^T * y using dynamic matrices
    MatDynamic<T> VtV(static_cast<size_t>(n_coeffs), static_cast<size_t>(n_coeffs), T(0));
    VecDynamic<T> Vty(static_cast<size_t>(n_coeffs), T(0));
    
    // Accumulate V^T * V and V^T * y
    for (int k = 0; k < m; ++k) {
        T x = x_data[static_cast<size_t>(k)];
        T y = y_data[static_cast<size_t>(k)];
        T x_pow_i = T(1);  // x^0
        
        for (int i = 0; i < n_coeffs; ++i) {
            T x_pow_j = T(1);  // x^0
            for (int j = 0; j < n_coeffs; ++j) {
                VtV(static_cast<size_t>(i), static_cast<size_t>(j)) += x_pow_i * x_pow_j;
                x_pow_j *= x;
            }
            Vty[static_cast<size_t>(i)] += x_pow_i * y;
            x_pow_i *= x;
        }
    }
    
    // Solve (V^T * V) * c = V^T * y using Gaussian elimination with partial pivoting
    // Since we can't use fixed-size solver with dynamic matrices, implement direct solve
    VecDynamic<T> coeffs = solve_normal_equations(VtV, Vty);
    
    return coeffs;
}

/**
 * @brief Evaluate polynomial at a point
 * 
 * Computes y = c0 + c1*x + c2*x^2 + ... using Horner's method
 * 
 * @tparam T Floating-point type
 * @param coeffs Polynomial coefficients [c0, c1, ..., c_n]
 * @param x Evaluation point
 * @return Polynomial value at x
 */
template<typename T>
inline T evaluate_polynomial(
    const VecDynamic<T>& coeffs,
    T x
) {
    if (coeffs.empty()) return T(0);
    
    int n = static_cast<int>(coeffs.size()) - 1;
    T result = coeffs[static_cast<size_t>(n)];
    for (int i = n - 1; i >= 0; --i) {
        result = result * x + coeffs[static_cast<size_t>(i)];
    }
    return result;
}

/**
 * @brief Compute polynomial derivative at a point
 * 
 * For y(x) = c0 + c1*x + c2*x^2 + ... + c_n*x^n
 * Returns dy/dx = c1 + 2*c2*x + 3*c3*x^2 + ...
 * 
 * @tparam T Floating-point type
 * @param coeffs Polynomial coefficients [c0, c1, ..., c_n]
 * @param x Evaluation point
 * @return First derivative at x
 */
template<typename T>
inline T polynomial_derivative(
    const VecDynamic<T>& coeffs,
    T x
) {
    if (coeffs.size() < 2) return T(0);  // Constant polynomial
    
    int n = static_cast<int>(coeffs.size()) - 1;
    T result = T(n) * coeffs[static_cast<size_t>(n)];
    for (int i = n - 1; i >= 1; --i) {
        result = result * x + T(i) * coeffs[static_cast<size_t>(i)];
    }
    return result;
}

// ============================================================================
// Derivative Estimation from Data (using existing finite_differences)
// ============================================================================

/**
 * @brief Estimate derivatives from data using local polynomial fitting
 * 
 * For each point, fits a low-degree polynomial to nearby points and
 * evaluates its derivative. This provides smoothed derivative estimates
 * suitable for noisy data.
 * 
 * @tparam T Floating-point type
 * @param x_data X coordinates (should be sorted)
 * @param y_data Y values (must match x_data size)
 * @param poly_degree Degree of local polynomial (1=linear, 2=quadratic, etc)
 * @param window_size Number of points to use (should be odd, >= 2*poly_degree+1)
 * @return Vector of derivative estimates at each x value
 * 
 * @throws std::runtime_error if x_data and y_data sizes don't match
 * @throws std::runtime_error if window_size is even or too small
 */
template<typename T>
inline VecDynamic<T> estimate_derivatives_from_data(
    const VecDynamic<T>& x_data,
    const VecDynamic<T>& y_data,
    int poly_degree = 2,
    int window_size = -1
) {
    if (x_data.size() != y_data.size()) {
        throw std::runtime_error("estimate_derivatives_from_data: size mismatch");
    }
    
    // Default window size: 2*degree + 1
    if (window_size < 0) {
        window_size = 2 * poly_degree + 1;
    }
    
    if (window_size % 2 == 0 || window_size < 2 * poly_degree + 1) {
        throw std::runtime_error("estimate_derivatives_from_data: window_size must be odd and >= 2*degree+1");
    }
    
    int n = static_cast<int>(x_data.size());
    int half_win = window_size / 2;
    VecDynamic<T> derivatives(static_cast<size_t>(n));
    
    for (int i = 0; i < n; ++i) {
        int start = std::max(0, i - half_win);
        int end = std::min(n - 1, i + half_win);
        
        // Collect window of points
        VecDynamic<T> x_window(static_cast<size_t>(end - start + 1));
        VecDynamic<T> y_window(static_cast<size_t>(end - start + 1));
        for (int j = start; j <= end; ++j) {
            x_window[static_cast<size_t>(j - start)] = x_data[static_cast<size_t>(j)];
            y_window[static_cast<size_t>(j - start)] = y_data[static_cast<size_t>(j)];
        }
        
        // Fit polynomial and evaluate derivative
        if (static_cast<int>(x_window.size()) >= poly_degree + 1) {
            auto coeffs = fit_polynomial(poly_degree, x_window, y_window);
            derivatives[static_cast<size_t>(i)] = polynomial_derivative(coeffs, x_data[static_cast<size_t>(i)]);
        } else {
            // Fallback: use central difference
            if (i > 0 && i < n - 1) {
                derivatives[static_cast<size_t>(i)] = (y_data[static_cast<size_t>(i + 1)] - y_data[static_cast<size_t>(i - 1)]) / 
                                                      (x_data[static_cast<size_t>(i + 1)] - x_data[static_cast<size_t>(i - 1)]);
            } else if (i == 0 && n > 1) {
                derivatives[static_cast<size_t>(i)] = (y_data[1] - y_data[0]) / (x_data[1] - x_data[0]);
            } else {
                derivatives[static_cast<size_t>(i)] = T(0);
            }
        }
    }
    
    return derivatives;
}

}  // namespace phynity::math::calculus

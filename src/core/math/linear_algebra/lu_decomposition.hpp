#pragma once

#include <core/math/matrices/mat_n.hpp>
#include <core/math/vectors/vec_n.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/math/utilities/validity_checks.hpp>
#include <array>
#include <cstddef>
#include <type_traits>

namespace phynity::math::linear_algebra {

using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;
using phynity::math::utilities::epsilon;

/// ============================================================================
/// LU DECOMPOSITION
/// ============================================================================
/// Decomposes a square matrix A into L (lower triangular) and U (upper triangular)
/// such that A = LU, using Doolittle's method with partial pivoting for numerical stability.
///
/// Partial pivoting improves numerical stability by selecting the pivot with
/// the largest absolute value, reducing error propagation.
///
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type (float, double)
template<std::size_t N, typename T = float>
struct LUDecomposition {
    static_assert(std::is_floating_point_v<T>, "LUDecomposition requires floating-point type");
    static_assert(N > 0, "LUDecomposition matrix size must be > 0");

    MatN<N, N, T> lu;                      // Combined LU matrix (L below diagonal, U on and above)
    std::array<int, N> pivot_indices;      // Permutation vector from partial pivoting
    int pivot_count;                       // Number of row swaps (sign indicator)
    bool is_singular;                      // True if matrix is (nearly) singular

    /// Default constructor - all zeros, no decomposition
    LUDecomposition() 
        : lu(T(0)), pivot_count(0), is_singular(false) {
        for (std::size_t i = 0; i < N; ++i) {
            pivot_indices[i] = static_cast<int>(i);
        }
    }

    /// Perform LU decomposition with partial pivoting
    /// @param A Matrix to decompose
    /// @param tolerance Tolerance for singularity detection
    explicit LUDecomposition(const MatN<N, N, T>& A, T tolerance = epsilon<T>() * T(100)) {
        lu = A;
        pivot_count = 0;
        is_singular = false;

        // Initialize pivot indices
        for (std::size_t i = 0; i < N; ++i) {
            pivot_indices[i] = static_cast<int>(i);
        }

        // Doolittle's method with partial pivoting
        for (std::size_t k = 0; k < N; ++k) {
            // Find pivot (largest absolute value in column k from row k onward)
            std::size_t pivot_row = k;
            T max_abs = std::abs(lu(k, k));

            for (std::size_t i = k + 1; i < N; ++i) {
                T abs_val = std::abs(lu(i, k));
                if (abs_val > max_abs) {
                    max_abs = abs_val;
                    pivot_row = i;
                }
            }

            // Check for singularity
            if (max_abs < tolerance) {
                is_singular = true;
                return;
            }

            // Swap rows if necessary
            if (pivot_row != k) {
                // Swap rows in LU matrix
                VecN<N, T> temp_row = lu.getRow(k);
                lu.setRow(k, lu.getRow(pivot_row));
                lu.setRow(pivot_row, temp_row);

                // Update pivot vector
                std::swap(pivot_indices[k], pivot_indices[pivot_row]);
                pivot_count++;
            }

            // Compute multipliers and eliminate
            for (std::size_t i = k + 1; i < N; ++i) {
                T multiplier = lu(i, k) / lu(k, k);
                lu(i, k) = multiplier;

                // Eliminate column k in row i
                for (std::size_t j = k + 1; j < N; ++j) {
                    lu(i, j) -= multiplier * lu(k, j);
                }
            }
        }
    }

    /// Get the L matrix (lower triangular with unit diagonal)
    MatN<N, N, T> getL() const {
        MatN<N, N, T> L(T(0));
        
        // Diagonal of L is always 1
        for (std::size_t i = 0; i < N; ++i) {
            L(i, i) = T(1);
        }

        // Below diagonal from lu matrix
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < i; ++j) {
                L(i, j) = lu(i, j);
            }
        }

        return L;
    }

    /// Get the U matrix (upper triangular)
    MatN<N, N, T> getU() const {
        MatN<N, N, T> U(T(0));
        
        // On and above diagonal from lu matrix
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = i; j < N; ++j) {
                U(i, j) = lu(i, j);
            }
        }

        return U;
    }

    /// Get permutation matrix from pivot indices
    MatN<N, N, T> getP() const {
        MatN<N, N, T> P(T(0));
        
        // Build permutation matrix: P[i] has a 1 in column pivot_indices[i]
        for (std::size_t i = 0; i < N; ++i) {
            P(i, static_cast<std::size_t>(pivot_indices[i])) = T(1);
        }

        return P;
    }

    /// Compute determinant from LU decomposition
    /// det(A) = (-1)^p * product(diagonal of U)
    T determinant() const {
        if (is_singular) {
            return T(0);
        }

        T det = T(1);
        for (std::size_t i = 0; i < N; ++i) {
            det *= lu(i, i);
        }

        // Multiply by sign from row swaps
        if (pivot_count % 2 != 0) {
            det = -det;
        }

        return det;
    }
};

/// Solve Ax = b using LU decomposition
/// First solves Ly = Pb (forward substitution)
/// Then solves Ux = y (back substitution)
/// @param lu_decomp LU decomposition of A
/// @param b Right-hand side vector
/// @return Solution vector x, or zero vector if singular
template<std::size_t N, typename T = float>
inline VecN<N, T> solve_lu(const LUDecomposition<N, T>& lu_decomp, const VecN<N, T>& b) {
    if (lu_decomp.is_singular) {
        return VecN<N, T>(T(0));
    }

    // Apply row permutation to b: Pb = P*b
    VecN<N, T> Pb(T(0));
    for (std::size_t i = 0; i < N; ++i) {
        Pb[i] = b[static_cast<std::size_t>(lu_decomp.pivot_indices[i])];
    }

    // Forward substitution: solve L*y = Pb for y
    VecN<N, T> y(T(0));
    for (std::size_t i = 0; i < N; ++i) {
        T sum = Pb[i];
        for (std::size_t j = 0; j < i; ++j) {
            sum -= lu_decomp.lu(i, j) * y[j];
        }
        y[i] = sum; // L(i,i) = 1, so no division needed
    }

    // Back substitution: solve U*x = y for x
    VecN<N, T> x(T(0));
    for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
        std::size_t ui = static_cast<std::size_t>(i);
        T sum = y[ui];
        for (int j = i + 1; j < static_cast<int>(N); ++j) {
            sum -= lu_decomp.lu(ui, static_cast<std::size_t>(j)) * x[static_cast<std::size_t>(j)];
        }
        if (std::abs(lu_decomp.lu(ui, ui)) < phynity::math::utilities::epsilon<T>() * T(100)) {
            return VecN<N, T>(T(0)); // Singular
        }
        x[ui] = sum / lu_decomp.lu(ui, ui);
    }

    return x;
}

}  // namespace phynity::math::linear_algebra

#pragma once

#include <core/math/matrices/mat_n.hpp>
#include <core/math/vectors/vec_n.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <cstddef>
#include <cmath>
#include <type_traits>

namespace phynity::math::linear_algebra {

using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;
using phynity::math::utilities::epsilon;

/// ============================================================================
/// CHOLESKY DECOMPOSITION
/// ============================================================================
/// Decomposes a symmetric positive-definite matrix A into L*L^T, where L is
/// a lower triangular matrix with positive diagonal entries.
///
/// Cholesky decomposition is optimal for SPD systems because:
/// - Time complexity O(N³/3) is roughly 2x faster than LU/QR
/// - Requires half the storage of general factorizations
/// - Numerically stable without pivoting (for SPD matrices)
/// - Perfect for covariance matrices and constraint Hessians in physics
///
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type (float, double)
template<std::size_t N, typename T = float>
struct CholeskyDecomposition {
    static_assert(std::is_floating_point_v<T>, "CholeskyDecomposition requires floating-point type");
    static_assert(N > 0, "CholeskyDecomposition matrix size must be > 0");

    MatN<N, N, T> L;              // Lower triangular matrix (only lower part is valid)
    bool is_positive_definite;    // True if matrix is SPD and decomposition succeeded
    bool is_definite;             // True if positive or negative definite

    /// Default constructor
    CholeskyDecomposition() 
        : L(T(0)), is_positive_definite(false), is_definite(false) {}

    /// Perform Cholesky decomposition using Doolittle's variant
    /// @param A Symmetric positive-definite matrix to decompose
    /// @param tolerance Tolerance for positivity check
    explicit CholeskyDecomposition(
        const MatN<N, N, T>& A,
        T tolerance = epsilon<T>() * T(100)
    ) {
        perform_decomposition(A, tolerance);
    }

    /// Get the transpose of L (which is the upper triangular R = L^T)
    MatN<N, N, T> getLT() const {
        return L.transposed();
    }

    /// Reconstruct the original matrix: A ≈ L * L^T
    MatN<N, N, T> reconstruct() const {
        return L * L.transposed();
    }

private:
    /// Perform Cholesky decomposition using row-wise algorithm
    void perform_decomposition(const MatN<N, N, T>& A, T tolerance) {
        L = MatN<N, N, T>(T(0));
        is_positive_definite = false;
        is_definite = false;

        // Doolittle's variant of Cholesky decomposition
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j <= i; ++j) {
                T sum = A(i, j);

                // Subtract contributions from previously computed L elements
                for (std::size_t k = 0; k < j; ++k) {
                    sum -= L(i, k) * L(j, k);
                }

                if (i == j) {
                    // Diagonal element: L(i,i) = sqrt(A(i,i) - sum(L(i,k)^2))
                    if (sum < tolerance) {
                        // Not positive definite
                        return;
                    }

                    L(i, i) = std::sqrt(sum);
                } else {
                    // Off-diagonal: L(i,j) = (A(i,j) - sum) / L(j,j)
                    if (std::abs(L(j, j)) < tolerance) {
                        return; // Singular
                    }

                    L(i, j) = sum / L(j, j);
                }
            }
        }

        is_positive_definite = true;
        is_definite = true;
    }
};

/// Solve Ax = b where A is symmetric positive-definite using Cholesky decomposition
/// @param chol Cholesky decomposition of A
/// @param b Right-hand side vector
/// @return Solution vector x, or zero vector if A is not SPD
template<std::size_t N, typename T = float>
inline VecN<N, T> solve_cholesky(const CholeskyDecomposition<N, T>& chol, const VecN<N, T>& b) {
    if (!chol.is_positive_definite) {
        return VecN<N, T>(T(0));
    }

    // Solve L*y = b by forward substitution
    VecN<N, T> y(T(0));
    for (std::size_t i = 0; i < N; ++i) {
        T sum = b[i];
        for (std::size_t j = 0; j < i; ++j) {
            sum -= chol.L(i, j) * y[j];
        }

        if (std::abs(chol.L(i, i)) < epsilon<T>() * T(100)) {
            return VecN<N, T>(T(0)); // Singular
        }

        y[i] = sum / chol.L(i, i);
    }

    // Solve L^T*x = y by back substitution
    VecN<N, T> x(T(0));
    for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
        std::size_t ui = static_cast<std::size_t>(i);
        T sum = y[ui];
        for (int j = i + 1; j < static_cast<int>(N); ++j) {
            std::size_t uj = static_cast<std::size_t>(j);
            sum -= chol.L(uj, ui) * x[uj];
        }

        if (std::abs(chol.L(ui, ui)) < epsilon<T>() * T(100)) {
            return VecN<N, T>(T(0)); // Singular
        }

        x[ui] = sum / chol.L(ui, ui);
    }

    return x;
}

/// Compute determinant of SPD matrix using Cholesky decomposition
/// det(A) = det(L * L^T) = (det(L))^2 = (product of diagonal)^2
/// @param chol Cholesky decomposition of A
/// @return Determinant of A, or 0 if not SPD
template<std::size_t N, typename T = float>
inline T determinant_cholesky(const CholeskyDecomposition<N, T>& chol) {
    if (!chol.is_positive_definite) {
        return T(0);
    }

    T det_L = T(1);
    for (std::size_t i = 0; i < N; ++i) {
        det_L *= chol.L(i, i);
    }

    return det_L * det_L;
}

/// Compute matrix inverse of SPD matrix using Cholesky decomposition
/// More efficient than general matrix inversion
/// @param chol Cholesky decomposition of A
/// @return Inverse of A, or zero matrix if not SPD
template<std::size_t N, typename T = float>
inline MatN<N, N, T> inverse_cholesky(const CholeskyDecomposition<N, T>& chol) {
    if (!chol.is_positive_definite) {
        return MatN<N, N, T>(T(0));
    }

    MatN<N, N, T> inv(T(0));

    // Solve A*X = I column by column
    MatN<N, N, T> I = MatN<N, N, T>::identity();
    for (std::size_t col = 0; col < N; ++col) {
        VecN<N, T> e_col = I.getColumn(col);
        VecN<N, T> x_col = solve_cholesky(chol, e_col);
        inv.setColumn(col, x_col);
    }

    return inv;
}

}  // namespace phynity::math::linear_algebra

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
/// QR DECOMPOSITION (Gram-Schmidt with Re-orthogonalization)
/// ============================================================================
/// Decomposes a matrix A into orthogonal matrix Q and upper triangular matrix R
/// such that A = QR, using modified Gram-Schmidt for improved numerical stability.
///
/// Modified Gram-Schmidt is preferred over classical because it:
/// - Reduces error accumulation in floating-point arithmetic
/// - Better preserves orthogonality in practice
/// - More stable for ill-conditioned matrices
///
/// @tparam N Size of NxN matrix (square matrices only for this version)
/// @tparam T Floating-point type (float, double)
template<std::size_t N, typename T = float>
struct QRDecomposition {
    static_assert(std::is_floating_point_v<T>, "QRDecomposition requires floating-point type");
    static_assert(N > 0, "QRDecomposition matrix size must be > 0");

    MatN<N, N, T> Q; // Orthogonal matrix (columns are orthonormal)
    MatN<N, N, T> R; // Upper triangular matrix
    bool is_singular; // True if matrix is (nearly) singular

    /// Default constructor - identity Q, zero R
    QRDecomposition() 
        : Q(MatN<N, N, T>::identity()), R(T(0)), is_singular(false) {}

    /// Perform QR decomposition using modified Gram-Schmidt
    /// @param A Matrix to decompose
    explicit QRDecomposition(const MatN<N, N, T>& A) {
        is_singular = false;
        Q = MatN<N, N, T>(T(0));
        R = MatN<N, N, T>(T(0));

        // Copy columns of A as working vectors
        std::array<VecN<N, T>, N> a;
        for (std::size_t i = 0; i < N; ++i) {
            a[i] = A.getColumn(i);
        }

        // Modified Gram-Schmidt orthogonalization
        for (std::size_t j = 0; j < N; ++j) {
            // Project out previously computed orthonormal vectors
            for (std::size_t i = 0; i < j; ++i) {
                R(i, j) = a[j].dot(Q.getColumn(i));
                a[j] = a[j] - Q.getColumn(i) * R(i, j);
            }

            // Compute norm and check for linear dependence
            T norm = a[j].length();
            R(j, j) = norm;

            if (norm < epsilon<T>() * T(100)) {
                is_singular = true;
                // Set Q column to zero or unit vector for numerical stability
                VecN<N, T> zero_col(T(0));
                Q.setColumn(j, zero_col);
            } else {
                // Normalize to get orthonormal vector
                VecN<N, T> q_col = a[j] * (T(1) / norm);
                Q.setColumn(j, q_col);
            }
        }
    }

    /// Get transpose of Q (useful for solving systems)
    MatN<N, N, T> getQT() const {
        return Q.transposed();
    }
};

/// Solve Ax = b using QR decomposition
/// Q*R*x = b  =>  R*x = Q^T*b
/// @param qr_decomp QR decomposition of A
/// @param b Right-hand side vector
/// @return Solution vector x, or zero vector if singular
template<std::size_t N, typename T = float>
inline VecN<N, T> solve_qr(const QRDecomposition<N, T>& qr_decomp, const VecN<N, T>& b) {
    if (qr_decomp.is_singular) {
        return VecN<N, T>(T(0));
    }

    // Compute y = Q^T * b
    VecN<N, T> y(T(0));
    MatN<N, N, T> QT = qr_decomp.getQT();
    for (std::size_t i = 0; i < N; ++i) {
        y[i] = QT.getRow(i).dot(b);
    }

    // Back substitution: solve R*x = y for x
    // R is upper triangular
    VecN<N, T> x(T(0));
    for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
        std::size_t ui = static_cast<std::size_t>(i);
        T sum = y[ui];
        for (int j = i + 1; j < static_cast<int>(N); ++j) {
            sum -= qr_decomp.R(ui, static_cast<std::size_t>(j)) * x[static_cast<std::size_t>(j)];
        }

        if (std::abs(qr_decomp.R(ui, ui)) < epsilon<T>() * T(100)) {
            return VecN<N, T>(T(0)); // Singular
        }

        x[ui] = sum / qr_decomp.R(ui, ui);
    }

    return x;
}

}  // namespace phynity::math::linear_algebra

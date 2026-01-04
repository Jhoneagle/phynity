#pragma once

#include <core/math/linear_algebra/lu_decomposition.hpp>
#include <core/math/linear_algebra/qr_decomposition.hpp>
#include <core/math/vectors/vec_n.hpp>
#include <cstddef>
#include <type_traits>

namespace phynity::math::linear_algebra {

using phynity::math::vectors::VecN;

/// ============================================================================
/// LINEAR SYSTEM SOLVER - Ax = b
/// ============================================================================
/// Solves the linear system Ax = b using specified decomposition method.
/// Useful for constraint solving, least-squares fitting, and physics computations.

enum class SolveMethod {
    LU,  ///< LU decomposition (faster, general purpose)
    QR   ///< QR decomposition (more stable for ill-conditioned systems)
};

/// Solve Ax = b for x
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Coefficient matrix
/// @param b Right-hand side vector
/// @param method Solution method (LU or QR)
/// @return Solution vector x
template<std::size_t N, typename T = float>
inline VecN<N, T> solve(const MatN<N, N, T>& A, const VecN<N, T>& b, SolveMethod method = SolveMethod::LU) {
    static_assert(std::is_floating_point_v<T>, "solve requires floating-point type");

    if (method == SolveMethod::LU) {
        LUDecomposition<N, T> lu(A);
        return solve_lu(lu, b);
    } else { // QR
        QRDecomposition<N, T> qr(A);
        return solve_qr(qr, b);
    }
}

/// Compute matrix inverse using LU decomposition
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Matrix to invert
/// @return Inverse of A, or zero matrix if singular
template<std::size_t N, typename T = float>
inline MatN<N, N, T> inverse(const MatN<N, N, T>& A) {
    static_assert(std::is_floating_point_v<T>, "inverse requires floating-point type");

    LUDecomposition<N, T> lu(A);
    
    if (lu.is_singular) {
        return MatN<N, N, T>(T(0));
    }

    MatN<N, N, T> inv(T(0));

    // Solve A*X = I column by column
    MatN<N, N, T> I = MatN<N, N, T>::identity();
    for (std::size_t col = 0; col < N; ++col) {
        VecN<N, T> e_col = I.getColumn(col);
        VecN<N, T> x_col = solve_lu(lu, e_col);
        inv.setColumn(col, x_col);
    }

    return inv;
}

/// Compute determinant of matrix using LU decomposition
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Matrix
/// @return Determinant of A, or 0 if singular
template<std::size_t N, typename T = float>
inline T determinant(const MatN<N, N, T>& A) {
    static_assert(std::is_floating_point_v<T>, "determinant requires floating-point type");

    LUDecomposition<N, T> lu(A);
    return lu.determinant();
}

}  // namespace phynity::math::linear_algebra

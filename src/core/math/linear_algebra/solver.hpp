#pragma once

#include <core/math/linear_algebra/lu_decomposition.hpp>
#include <core/math/linear_algebra/qr_decomposition.hpp>
#include <core/math/linear_algebra/svd_decomposition.hpp>
#include <core/math/linear_algebra/cholesky_decomposition.hpp>
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

/// Solve least-squares problem using SVD: minimize ||Ax - b||²
/// Returns the solution with minimum norm when system is underdetermined
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Coefficient matrix
/// @param b Right-hand side vector
/// @return Least-squares solution
template<std::size_t N, typename T = float>
inline VecN<N, T> least_squares_solve(const MatN<N, N, T>& A, const VecN<N, T>& b) {
    static_assert(std::is_floating_point_v<T>, "least_squares_solve requires floating-point type");

    SVDDecomposition<N, T> svd(A);
    MatN<N, N, T> A_pinv = pseudoinverse(svd);
    return A_pinv * b;
}

/// Get numerical rank of matrix using SVD
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Matrix
/// @param tolerance Tolerance for singular value threshold
/// @return Numerical rank of A
template<std::size_t N, typename T = float>
inline int matrix_rank(const MatN<N, N, T>& A, T tolerance = epsilon<T>() * T(100)) {
    static_assert(std::is_floating_point_v<T>, "matrix_rank requires floating-point type");

    SVDDecomposition<N, T> svd(A, 100, tolerance);
    return svd.rank;
}

/// Get condition number of matrix using SVD
/// κ(A) = σ_max / σ_min (ratio of largest to smallest singular value)
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Matrix
/// @return Condition number of A
template<std::size_t N, typename T = float>
inline T condition_number(const MatN<N, N, T>& A) {
    static_assert(std::is_floating_point_v<T>, "condition_number requires floating-point type");

    SVDDecomposition<N, T> svd(A);
    return svd.condition_number;
}

/// Compute Moore-Penrose pseudoinverse using SVD
/// Robust inverse for singular or ill-conditioned matrices
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Matrix to invert
/// @return Pseudoinverse of A
template<std::size_t N, typename T = float>
inline MatN<N, N, T> pseudo_inverse(const MatN<N, N, T>& A) {
    static_assert(std::is_floating_point_v<T>, "pseudo_inverse requires floating-point type");

    SVDDecomposition<N, T> svd(A);
    return pseudoinverse(svd);
}

/// Solve Ax = b where A is symmetric positive-definite using Cholesky decomposition
/// Most efficient for covariance matrices and constraint Hessians
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Symmetric positive-definite matrix
/// @param b Right-hand side vector
/// @return Solution vector x, or zero vector if A is not SPD
template<std::size_t N, typename T = float>
inline VecN<N, T> solve_spd(const MatN<N, N, T>& A, const VecN<N, T>& b) {
    static_assert(std::is_floating_point_v<T>, "solve_spd requires floating-point type");

    CholeskyDecomposition<N, T> chol(A);
    return solve_cholesky(chol, b);
}

/// Compute determinant of symmetric positive-definite matrix using Cholesky
/// More efficient than general determinant computation
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Symmetric positive-definite matrix
/// @return Determinant of A, or 0 if not SPD
template<std::size_t N, typename T = float>
inline T determinant_spd(const MatN<N, N, T>& A) {
    static_assert(std::is_floating_point_v<T>, "determinant_spd requires floating-point type");

    CholeskyDecomposition<N, T> chol(A);
    return determinant_cholesky(chol);
}

/// Compute inverse of symmetric positive-definite matrix using Cholesky
/// More efficient than general matrix inversion
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type
/// @param A Symmetric positive-definite matrix
/// @return Inverse of A, or zero matrix if not SPD
template<std::size_t N, typename T = float>
inline MatN<N, N, T> inverse_spd(const MatN<N, N, T>& A) {
    static_assert(std::is_floating_point_v<T>, "inverse_spd requires floating-point type");

    CholeskyDecomposition<N, T> chol(A);
    return inverse_cholesky(chol);
}

}  // namespace phynity::math::linear_algebra

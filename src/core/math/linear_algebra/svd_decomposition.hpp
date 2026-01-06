#pragma once

#include <core/math/matrices/mat_n.hpp>
#include <core/math/vectors/vec_n.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <cstddef>
#include <cmath>
#include <algorithm>
#include <type_traits>

namespace phynity::math::linear_algebra {

using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;
using phynity::math::utilities::epsilon;

/// ============================================================================
/// SINGULAR VALUE DECOMPOSITION (SVD)
/// ============================================================================
/// Decomposes a square matrix A into A = U * Σ * V^T, where:
/// - U and V are orthogonal matrices (columns are orthonormal)
/// - Σ is a diagonal matrix containing singular values (non-negative, sorted descending)
///
/// SVD provides a robust way to:
/// - Compute pseudoinverse (Moore-Penrose inverse)
/// - Determine matrix rank
/// - Find condition number
/// - Solve least-squares problems
/// - Perform principal component analysis
///
/// Implementation uses one-sided Jacobi method for improved accuracy on small matrices
/// and numerical stability across different scaling factors.
///
/// @tparam N Size of NxN matrix
/// @tparam T Floating-point type (float, double)
template<std::size_t N, typename T = float>
struct SVDDecomposition {
    static_assert(std::is_floating_point_v<T>, "SVDDecomposition requires floating-point type");
    static_assert(N > 0, "SVDDecomposition matrix size must be > 0");

    MatN<N, N, T> U;                 // Left singular vectors (orthogonal matrix)
    MatN<N, N, T> V;                 // Right singular vectors (orthogonal matrix)
    std::array<T, N> singular_values; // Diagonal entries of Σ (sorted descending)
    int rank;                         // Numerical rank of matrix
    T condition_number;               // κ(A) = σ_max / σ_min

    /// Default constructor - identity decomposition
    SVDDecomposition() 
        : U(MatN<N, N, T>::identity()), V(MatN<N, N, T>::identity()), rank(static_cast<int>(N)), condition_number(T(1)) {
        for (std::size_t i = 0; i < N; ++i) {
            singular_values[i] = T(1);
        }
    }

    /// Perform SVD decomposition using one-sided Jacobi iteration
    /// @param A Matrix to decompose
    /// @param max_iterations Maximum number of Jacobi iterations
    /// @param tolerance Tolerance for convergence and singularity detection
    explicit SVDDecomposition(
        const MatN<N, N, T>& A,
        std::size_t max_iterations = 100,
        T tolerance = epsilon<T>() * T(100)
    ) {
        svd_jacobi(A, max_iterations, tolerance);
    }

    /// Get the transpose of U
    MatN<N, N, T> getUT() const {
        return U.transposed();
    }

    /// Get the transpose of V
    MatN<N, N, T> getVT() const {
        return V.transposed();
    }

    /// Get the diagonal matrix Σ
    MatN<N, N, T> getSigma() const {
        MatN<N, N, T> sigma(T(0));
        for (std::size_t i = 0; i < N; ++i) {
            sigma(i, i) = singular_values[i];
        }
        return sigma;
    }

    /// Reconstruct the original matrix: A ≈ U * Σ * V^T
    MatN<N, N, T> reconstruct() const {
        MatN<N, N, T> sigma = getSigma();
        MatN<N, N, T> VT = getVT();
        return U * sigma * VT;
    }

private:
    /// One-sided Jacobi SVD algorithm
    /// Computes SVD by diagonalizing A^T * A to find V and Σ,
    /// then computes U from A * V * Σ^-1
    void svd_jacobi(const MatN<N, N, T>& A, std::size_t max_iterations, T tolerance) {
        // Compute A^T * A
        MatN<N, N, T> AT = A.transposed();
        MatN<N, N, T> ATA = AT * A;

        // Initialize V as identity matrix
        V = MatN<N, N, T>::identity();

        // Jacobi iteration to diagonalize A^T * A
        for (std::size_t iter = 0; iter < max_iterations; ++iter) {
            bool converged = true;

            // Sweep through all off-diagonal pairs
            for (std::size_t i = 0; i < N - 1; ++i) {
                for (std::size_t j = i + 1; j < N; ++j) {
                    T a_ii = ATA(i, i);
                    T a_jj = ATA(j, j);
                    T a_ij = ATA(i, j);

                    // Check convergence criterion
                    if (std::abs(a_ij) > tolerance * std::max(std::abs(a_ii), std::abs(a_jj))) {
                        converged = false;

                        // Compute Givens rotation angle
                        T theta = T(0.5) * std::atan2(T(2) * a_ij, a_jj - a_ii);
                        T c = std::cos(theta);
                        T s = std::sin(theta);

                        // Apply rotation to ATA
                        T a_ii_new = c * c * a_ii - T(2) * s * c * a_ij + s * s * a_jj;
                        T a_jj_new = s * s * a_ii + T(2) * s * c * a_ij + c * c * a_jj;
                        T a_ij_new = s * c * (a_ii - a_jj) + (c * c - s * s) * a_ij;

                        ATA(i, i) = a_ii_new;
                        ATA(j, j) = a_jj_new;
                        ATA(i, j) = a_ij_new;
                        ATA(j, i) = a_ij_new;

                        // Apply rotation to other rows/columns
                        for (std::size_t k = 0; k < N; ++k) {
                            if (k != i && k != j) {
                                T a_ki = ATA(k, i);
                                T a_kj = ATA(k, j);
                                ATA(k, i) = c * a_ki - s * a_kj;
                                ATA(i, k) = ATA(k, i);
                                ATA(k, j) = s * a_ki + c * a_kj;
                                ATA(j, k) = ATA(k, j);
                            }
                        }

                        // Apply rotation to V
                        for (std::size_t k = 0; k < N; ++k) {
                            T v_ki = V(k, i);
                            T v_kj = V(k, j);
                            V(k, i) = c * v_ki - s * v_kj;
                            V(k, j) = s * v_ki + c * v_kj;
                        }
                    }
                }
            }

            if (converged) {
                break;
            }
        }

        // Extract singular values from diagonal of ATA
        for (std::size_t i = 0; i < N; ++i) {
            T val = ATA(i, i);
            singular_values[i] = (val > T(0)) ? std::sqrt(val) : T(0);
        }

        // Sort singular values in descending order and reorder V accordingly
        sort_singular_values_descending();

        // Compute U = A * V * Σ^-1
        compute_U(A);

        // Compute rank and condition number
        compute_rank_and_condition(tolerance);
    }

    /// Sort singular values in descending order and permute V and U accordingly
    void sort_singular_values_descending() {
        for (std::size_t i = 0; i < N - 1; ++i) {
            std::size_t max_idx = i;
            for (std::size_t j = i + 1; j < N; ++j) {
                if (singular_values[j] > singular_values[max_idx]) {
                    max_idx = j;
                }
            }

            if (max_idx != i) {
                // Swap singular values
                std::swap(singular_values[i], singular_values[max_idx]);

                // Swap columns of V
                VecN<N, T> v_i = V.getColumn(i);
                VecN<N, T> v_max = V.getColumn(max_idx);
                V.setColumn(i, v_max);
                V.setColumn(max_idx, v_i);
            }
        }
    }

    /// Compute U matrix from A, V, and Σ
    void compute_U(const MatN<N, N, T>& A) {
        U = MatN<N, N, T>(T(0));

        for (std::size_t j = 0; j < N; ++j) {
            VecN<N, T> v_j = V.getColumn(j);

            if (singular_values[j] > epsilon<T>() * T(100)) {
                VecN<N, T> u_j = A * v_j;
                u_j = u_j * (T(1) / singular_values[j]);
                U.setColumn(j, u_j);
            } else {
                // For zero singular values, U column can be arbitrary orthonormal vector
                // For now, set to zero
                VecN<N, T> zero_col(T(0));
                U.setColumn(j, zero_col);
            }
        }

        // Orthonormalize U columns if needed (Gram-Schmidt)
        orthonormalize_matrix(U);
    }

    /// Gram-Schmidt orthonormalization of matrix columns
    void orthonormalize_matrix(MatN<N, N, T>& mat) {
        for (std::size_t j = 0; j < N; ++j) {
            VecN<N, T> col = mat.getColumn(j);

            // Project out previously computed orthonormal columns
            for (std::size_t i = 0; i < j; ++i) {
                VecN<N, T> u_i = mat.getColumn(i);
                T projection = col.dot(u_i);
                col = col - u_i * projection;
            }

            // Normalize
            T norm = col.length();
            if (norm > epsilon<T>() * T(100)) {
                col = col * (T(1) / norm);
            }

            mat.setColumn(j, col);
        }
    }

    /// Compute numerical rank and condition number
    void compute_rank_and_condition(T tolerance) {
        rank = 0;
        for (std::size_t i = 0; i < N; ++i) {
            if (singular_values[i] > tolerance) {
                rank++;
            }
        }

        // Condition number: κ(A) = σ_max / σ_min
        if (singular_values[0] > epsilon<T>() * T(100) && rank > 0) {
            T sigma_min = T(0);
            for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
                std::size_t ui = static_cast<std::size_t>(i);
                if (singular_values[ui] > epsilon<T>() * T(100)) {
                    sigma_min = singular_values[ui];
                    break;
                }
            }

            if (sigma_min > epsilon<T>() * T(100)) {
                condition_number = singular_values[0] / sigma_min;
            } else {
                condition_number = std::numeric_limits<T>::infinity();
            }
        } else {
            condition_number = std::numeric_limits<T>::infinity();
        }
    }
};

/// Compute Moore-Penrose pseudoinverse using SVD
/// A^+ = V * Σ^+ * U^T, where Σ^+ has reciprocals of non-zero singular values
/// @param svd SVD decomposition of A
/// @param tolerance Tolerance for considering singular value as non-zero
/// @return Pseudoinverse of A
template<std::size_t N, typename T = float>
inline MatN<N, N, T> pseudoinverse(const SVDDecomposition<N, T>& svd, T tolerance = epsilon<T>() * T(100)) {
    // Create Σ^+ by taking reciprocals of non-zero singular values
    std::array<T, N> sigma_inv;
    for (std::size_t i = 0; i < N; ++i) {
        if (svd.singular_values[i] > tolerance) {
            sigma_inv[i] = T(1) / svd.singular_values[i];
        } else {
            sigma_inv[i] = T(0);
        }
    }

    // Build Σ^+ as diagonal matrix
    MatN<N, N, T> sigma_plus(T(0));
    for (std::size_t i = 0; i < N; ++i) {
        sigma_plus(i, i) = sigma_inv[i];
    }

    // A^+ = V * Σ^+ * U^T
    MatN<N, N, T> UT = svd.getUT();
    return svd.V * sigma_plus * UT;
}

}  // namespace phynity::math::linear_algebra

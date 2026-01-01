#pragma once

#include <core/math/vectors/vec_n.hpp>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace phynity::math::matrices {

using phynity::math::vectors::VecN;

/// Compile-time sized MxN matrix (row-major storage)
template <std::size_t M, std::size_t N>
struct MatN {
    std::array<float, M * N> data{};  // default zero-initialized

    // Constructors
    MatN() {
        data.fill(0.0f);
        if constexpr (M == N) {
            for (std::size_t i = 0; i < M; ++i) {
                (*this)(i, i) = 1.0f;
            }
        }
    }

    /// Fill constructor - sets all elements to scalar
    explicit MatN(float scalar) {
        data.fill(scalar);
    }

    /// Initialize from contiguous array (row-major)
    explicit MatN(const std::array<float, M * N>& values) : data(values) {}

    /// Element access (row, col)
    float& operator()(std::size_t row, std::size_t col) {
        return data[row * N + col];
    }

    const float& operator()(std::size_t row, std::size_t col) const {
        return data[row * N + col];
    }

    // Arithmetic operators
    MatN operator+(const MatN& other) const {
        MatN result;
        for (std::size_t i = 0; i < M * N; ++i) {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }

    MatN operator-(const MatN& other) const {
        MatN result;
        for (std::size_t i = 0; i < M * N; ++i) {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }

    MatN operator*(float scalar) const {
        MatN result;
        for (std::size_t i = 0; i < M * N; ++i) {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    MatN operator/(float scalar) const {
        MatN result;
        for (std::size_t i = 0; i < M * N; ++i) {
            result.data[i] = data[i] / scalar;
        }
        return result;
    }

    MatN& operator+=(const MatN& other) {
        for (std::size_t i = 0; i < M * N; ++i) {
            data[i] += other.data[i];
        }
        return *this;
    }

    MatN& operator-=(const MatN& other) {
        for (std::size_t i = 0; i < M * N; ++i) {
            data[i] -= other.data[i];
        }
        return *this;
    }

    MatN& operator*=(float scalar) {
        for (std::size_t i = 0; i < M * N; ++i) {
            data[i] *= scalar;
        }
        return *this;
    }

    MatN& operator/=(float scalar) {
        for (std::size_t i = 0; i < M * N; ++i) {
            data[i] /= scalar;
        }
        return *this;
    }

    bool operator==(const MatN& other) const {
        return data == other.data;
    }

    bool operator!=(const MatN& other) const {
        return !(*this == other);
    }

    /// Matrix multiplication (M x K) * (K x P) = (M x P)
    template <std::size_t P>
    MatN<M, P> operator*(const MatN<N, P>& other) const {
        MatN<M, P> result;
        for (std::size_t i = 0; i < M; ++i) {
            for (std::size_t j = 0; j < P; ++j) {
                float sum = 0.0f;
                for (std::size_t k = 0; k < N; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

    /// Matrix-vector multiplication (VecN length N -> VecN length M)
    VecN<M> operator*(const VecN<N>& v) const {
        VecN<M> result;
        for (std::size_t i = 0; i < M; ++i) {
            float sum = 0.0f;
            for (std::size_t k = 0; k < N; ++k) {
                sum += (*this)(i, k) * v[k];
            }
            result[i] = sum;
        }
        return result;
    }

    /// Matrix transpose
    MatN<N, M> transposed() const {
        MatN<N, M> result;
        for (std::size_t i = 0; i < M; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    /// In-place transpose (only for square matrices)
    MatN& transpose() {
        static_assert(M == N, "transpose() in-place requires square matrix");
        for (std::size_t i = 0; i < M; ++i) {
            for (std::size_t j = i + 1; j < N; ++j) {
                std::swap((*this)(i, j), (*this)(j, i));
            }
        }
        return *this;
    }

    /// Trace (square matrices only)
    float trace() const {
        static_assert(M == N, "trace() requires square matrix");
        float sum = 0.0f;
        for (std::size_t i = 0; i < M; ++i) {
            sum += (*this)(i, i);
        }
        return sum;
    }

    /// Minor determinant after removing row r and column c (square, size>1, N<=4)
    float minor(std::size_t r, std::size_t c) const {
        static_assert(M == N, "minor() requires square matrix");
        static_assert(M > 1, "minor() undefined for 1x1 matrices");
        static_assert(M <= 4, "minor() only implemented for N<=4");

        MatN<M - 1, M - 1> sub;
        std::size_t dstRow = 0;
        for (std::size_t i = 0; i < M; ++i) {
            if (i == r) continue;
            std::size_t dstCol = 0;
            for (std::size_t j = 0; j < N; ++j) {
                if (j == c) continue;
                sub(dstRow, dstCol) = (*this)(i, j);
                ++dstCol;
            }
            ++dstRow;
        }
        return determinant_impl(sub);
    }

    /// Cofactor matrix (square, N<=4)
    MatN cofactor() const {
        static_assert(M == N, "cofactor() requires square matrix");
        static_assert(M <= 4, "cofactor() only implemented for N<=4");
        MatN result(0.0f);
        for (std::size_t i = 0; i < M; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                float sign = ((i + j) % 2 == 0) ? 1.0f : -1.0f;
                result(i, j) = sign * minor(i, j);
            }
        }
        return result;
    }

    /// Determinant (square, N<=4)
    float determinant() const {
        static_assert(M == N, "determinant() requires square matrix");
        static_assert(M <= 4, "determinant() only implemented for N<=4");
        return determinant_impl(*this);
    }

    /// Inverse (square, N<=4). Returns zero matrix when singular.
    MatN inverse() const {
        static_assert(M == N, "inverse() requires square matrix");
        static_assert(M <= 4, "inverse() only implemented for N<=4");
        float det = determinant();
        if (std::abs(det) < 1e-6f) {
            return MatN(0.0f);
        }
        MatN cof = cofactor();
        MatN adj = cof.transposed();
        return adj / det;
    }

    /// Create zero matrix
    static MatN zero() {
        return MatN(0.0f);
    }

    /// Create identity matrix (only when square)
    static MatN identity() {
        static_assert(M == N, "identity() only available for square matrices");
        MatN result = MatN::zero();
        for (std::size_t i = 0; i < M; ++i) {
            result(i, i) = 1.0f;
        }
        return result;
    }

private:
    template <std::size_t Size>
    static float determinant_impl(const MatN<Size, Size>& mtx) {
        if constexpr (Size == 1) {
            return mtx(0, 0);
        } else if constexpr (Size == 2) {
            return mtx(0, 0) * mtx(1, 1) - mtx(0, 1) * mtx(1, 0);
        } else if constexpr (Size == 3) {
            return mtx(0, 0) * (mtx(1, 1) * mtx(2, 2) - mtx(1, 2) * mtx(2, 1)) -
                   mtx(0, 1) * (mtx(1, 0) * mtx(2, 2) - mtx(1, 2) * mtx(2, 0)) +
                   mtx(0, 2) * (mtx(1, 0) * mtx(2, 1) - mtx(1, 1) * mtx(2, 0));
        } else if constexpr (Size == 4) {
            // Compute 3x3 minors of first row
            float m00 = mtx(1, 1) * (mtx(2, 2) * mtx(3, 3) - mtx(2, 3) * mtx(3, 2)) -
                        mtx(1, 2) * (mtx(2, 1) * mtx(3, 3) - mtx(2, 3) * mtx(3, 1)) +
                        mtx(1, 3) * (mtx(2, 1) * mtx(3, 2) - mtx(2, 2) * mtx(3, 1));

            float m01 = mtx(1, 0) * (mtx(2, 2) * mtx(3, 3) - mtx(2, 3) * mtx(3, 2)) -
                        mtx(1, 2) * (mtx(2, 0) * mtx(3, 3) - mtx(2, 3) * mtx(3, 0)) +
                        mtx(1, 3) * (mtx(2, 0) * mtx(3, 2) - mtx(2, 2) * mtx(3, 0));

            float m02 = mtx(1, 0) * (mtx(2, 1) * mtx(3, 3) - mtx(2, 3) * mtx(3, 1)) -
                        mtx(1, 1) * (mtx(2, 0) * mtx(3, 3) - mtx(2, 3) * mtx(3, 0)) +
                        mtx(1, 3) * (mtx(2, 0) * mtx(3, 1) - mtx(2, 1) * mtx(3, 0));

            float m03 = mtx(1, 0) * (mtx(2, 1) * mtx(3, 2) - mtx(2, 2) * mtx(3, 1)) -
                        mtx(1, 1) * (mtx(2, 0) * mtx(3, 2) - mtx(2, 2) * mtx(3, 0)) +
                        mtx(1, 2) * (mtx(2, 0) * mtx(3, 1) - mtx(2, 1) * mtx(3, 0));

            return mtx(0, 0) * m00 - mtx(0, 1) * m01 + mtx(0, 2) * m02 - mtx(0, 3) * m03;
        }
    }
};

/// Scalar * Matrix multiplication (commutative for scalar)
template <std::size_t M, std::size_t N>
inline MatN<M, N> operator*(float scalar, const MatN<M, N>& m) {
    return m * scalar;
}

}  // namespace phynity::math::matrices

#pragma once

#include <core/math/vectors/vec2.hpp>
#include <cmath>
#include <ostream>
#include <algorithm>
#include <type_traits>

namespace phynity::math::matrices {

using phynity::math::vectors::Vec2;

/// 2×2 floating-point matrix for 2D transformations with dual-precision support.
/// Storage is in row-major order: m[row][col]
template<typename T = float>
struct Mat2 {
    static_assert(std::is_floating_point_v<T>, "Mat2 template parameter must be a floating-point type");

    // Constructors
    constexpr Mat2() = default;

    /// Fill constructor - creates matrix with all elements set to scalar
    explicit constexpr Mat2(T scalar) {
        m[0][0] = scalar; m[0][1] = scalar;
        m[1][0] = scalar; m[1][1] = scalar;
    }

    /// Element-wise constructor
    constexpr Mat2(T m00, T m01,
         T m10, T m11) {
        m[0][0] = m00; m[0][1] = m01;
        m[1][0] = m10; m[1][1] = m11;
    }

    /// Column vector constructor (two column vectors)
    constexpr Mat2(const Vec2<T>& col0, const Vec2<T>& col1) {
        m[0][0] = col0.x; m[0][1] = col1.x;
        m[1][0] = col0.y; m[1][1] = col1.y;
    }

    // Operators
    constexpr Mat2 operator+(const Mat2& other) const {
        return Mat2(
            m[0][0] + other.m[0][0], m[0][1] + other.m[0][1],
            m[1][0] + other.m[1][0], m[1][1] + other.m[1][1]
        );
    }

    constexpr Mat2 operator-(const Mat2& other) const {
        return Mat2(
            m[0][0] - other.m[0][0], m[0][1] - other.m[0][1],
            m[1][0] - other.m[1][0], m[1][1] - other.m[1][1]
        );
    }

    constexpr Mat2 operator*(T scalar) const {
        return Mat2(
            m[0][0] * scalar, m[0][1] * scalar,
            m[1][0] * scalar, m[1][1] * scalar
        );
    }

    constexpr Mat2 operator/(T scalar) const {
        return Mat2(
            m[0][0] / scalar, m[0][1] / scalar,
            m[1][0] / scalar, m[1][1] / scalar
        );
    }

    /// Matrix multiplication
    constexpr Mat2 operator*(const Mat2& other) const {
        return Mat2(
            m[0][0] * other.m[0][0] + m[0][1] * other.m[1][0],
            m[0][0] * other.m[0][1] + m[0][1] * other.m[1][1],
            m[1][0] * other.m[0][0] + m[1][1] * other.m[1][0],
            m[1][0] * other.m[0][1] + m[1][1] * other.m[1][1]
        );
    }

    /// Matrix-vector multiplication
    constexpr Vec2<T> operator*(const Vec2<T>& v) const {
        return Vec2<T>(
            m[0][0] * v.x + m[0][1] * v.y,
            m[1][0] * v.x + m[1][1] * v.y
        );
    }

    constexpr Mat2 operator-() const {
        return Mat2(
            -m[0][0], -m[0][1],
            -m[1][0], -m[1][1]
        );
    }

    Mat2& operator+=(const Mat2& other) {
        m[0][0] += other.m[0][0]; m[0][1] += other.m[0][1];
        m[1][0] += other.m[1][0]; m[1][1] += other.m[1][1];
        return *this;
    }

    Mat2& operator-=(const Mat2& other) {
        m[0][0] -= other.m[0][0]; m[0][1] -= other.m[0][1];
        m[1][0] -= other.m[1][0]; m[1][1] -= other.m[1][1];
        return *this;
    }

    Mat2& operator*=(T scalar) {
        m[0][0] *= scalar; m[0][1] *= scalar;
        m[1][0] *= scalar; m[1][1] *= scalar;
        return *this;
    }

    Mat2& operator/=(T scalar) {
        m[0][0] /= scalar; m[0][1] /= scalar;
        m[1][0] /= scalar; m[1][1] /= scalar;
        return *this;
    }

    Mat2& operator*=(const Mat2& other) {
        *this = *this * other;
        return *this;
    }

    /// Component-wise (Hadamard) multiplication
    Mat2& mulComponentWise(const Mat2& other) {
        m[0][0] *= other.m[0][0]; m[0][1] *= other.m[0][1];
        m[1][0] *= other.m[1][0]; m[1][1] *= other.m[1][1];
        return *this;
    }

    /// Component-wise division
    Mat2& divComponentWise(const Mat2& other) {
        m[0][0] /= other.m[0][0]; m[0][1] /= other.m[0][1];
        m[1][0] /= other.m[1][0]; m[1][1] /= other.m[1][1];
        return *this;
    }

    constexpr bool operator==(const Mat2& other) const {
        return m[0][0] == other.m[0][0] && m[0][1] == other.m[0][1] &&
               m[1][0] == other.m[1][0] && m[1][1] == other.m[1][1];
    }

    constexpr bool operator!=(const Mat2& other) const {
        return !(*this == other);
    }

    T& operator()(int row, int col) {
        return m[row][col];
    }

    const T& operator()(int row, int col) const {
        return m[row][col];
    }

    /// Get row as vector
    Vec2<T> getRow(int row) const {
        return Vec2<T>(m[row][0], m[row][1]);
    }

    /// Get column as vector
    Vec2<T> getColumn(int col) const {
        return Vec2<T>(m[0][col], m[1][col]);
    }

    /// Set row from vector
    void setRow(int row, const Vec2<T>& v) {
        m[row][0] = v.x;
        m[row][1] = v.y;
    }

    /// Set column from vector
    void setColumn(int col, const Vec2<T>& v) {
        m[0][col] = v.x;
        m[1][col] = v.y;
    }

    /// Determinant
    T determinant() const {
        return m[0][0] * m[1][1] - m[0][1] * m[1][0];
    }

    /// Matrix inverse
    Mat2 inverse() const {
        T det = determinant();
        if (std::abs(det) < T(1e-6)) {
            return Mat2(T(0));  // Singular matrix, return zero
        }
        T invDet = T(1) / det;
        return Mat2(
             m[1][1] * invDet, -m[0][1] * invDet,
            -m[1][0] * invDet,  m[0][0] * invDet
        );
    }

    /// Matrix transpose
    Mat2 transposed() const {
        return Mat2(
            m[0][0], m[1][0],
            m[0][1], m[1][1]
        );
    }

    Mat2& transpose() {
        std::swap(m[0][1], m[1][0]);
        return *this;
    }

    /// Trace (sum of diagonal elements)
    T trace() const {
        return m[0][0] + m[1][1];
    }

    /// Approximate equality with epsilon tolerance
    bool approxEqual(const Mat2& other, T epsilon = T(1e-5)) const {
        return std::abs(m[0][0] - other.m[0][0]) < epsilon &&
               std::abs(m[0][1] - other.m[0][1]) < epsilon &&
               std::abs(m[1][0] - other.m[1][0]) < epsilon &&
               std::abs(m[1][1] - other.m[1][1]) < epsilon;
    }

    /// Return matrix with absolute values of all elements
    Mat2 abs() const {
        return Mat2(
            std::abs(m[0][0]), std::abs(m[0][1]),
            std::abs(m[1][0]), std::abs(m[1][1])
        );
    }

    /// Create identity matrix
    static Mat2 identity() {
        return Mat2(
            T(1), T(0),
            T(0), T(1)
        );
    }

    /// Create zero matrix
    static Mat2 zero() {
        return Mat2(
            T(0), T(0),
            T(0), T(0)
        );
    }

    /// Create 2D rotation matrix (angle in radians)
    static Mat2 rotation(T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        return Mat2(
            c, -s,
            s,  c
        );
    }

    /// Create 2D scale matrix
    static Mat2 scale(T sx, T sy) {
        return Mat2(
            sx,  T(0),
            T(0), sy
        );
    }

    /// Create 2D scale matrix (uniform)
    static Mat2 scale(T s) {
        return scale(s, s);
    }

    // Raw pointer to first element (row-major)
    T* dataPtr() {
        return &m[0][0];
    }

    const T* dataPtr() const {
        return &m[0][0];
    }

private:
    T m[2][2] = {{T(1), T(0)}, {T(0), T(1)}};
};

/// Scalar * Matrix multiplication
template<typename T = float>
inline Mat2<T> operator*(T scalar, const Mat2<T>& m) {
    return m * scalar;
}

/// Vector * Matrix multiplication (treats vector as row vector)
template<typename T = float>
inline Vec2<T> operator*(const Vec2<T>& v, const Mat2<T>& m) {
    return Vec2<T>(
        v.x * m(0, 0) + v.y * m(1, 0),
        v.x * m(0, 1) + v.y * m(1, 1)
    );
}

template<typename T = float>
inline std::ostream& operator<<(std::ostream& os, const Mat2<T>& m) {
    os << "[(" << m(0, 0) << ", " << m(0, 1) << "), "
       << "(" << m(1, 0) << ", " << m(1, 1) << ")]";
    return os;
}

// Type aliases
using Mat2f = Mat2<float>;
using Mat2d = Mat2<double>;

}  // namespace phynity::math::matrices

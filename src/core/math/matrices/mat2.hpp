#pragma once

#include <core/math/vectors/vec2.hpp>
#include <cmath>
#include <ostream>
#include <algorithm>

namespace phynity::math::matrices {

/// 2×2 floating-point matrix for 2D transformations.
/// Storage is in row-major order: m[row][col]
struct Mat2 {
    float m[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};

    // Constructors
    Mat2() = default;

    /// Fill constructor - creates matrix with all elements set to scalar
    explicit Mat2(float scalar) {
        m[0][0] = scalar; m[0][1] = scalar;
        m[1][0] = scalar; m[1][1] = scalar;
    }

    /// Element-wise constructor
    Mat2(float m00, float m01,
         float m10, float m11) {
        m[0][0] = m00; m[0][1] = m01;
        m[1][0] = m10; m[1][1] = m11;
    }

    /// Column vector constructor (two column vectors)
    Mat2(const Vec2& col0, const Vec2& col1) {
        m[0][0] = col0.x; m[0][1] = col1.x;
        m[1][0] = col0.y; m[1][1] = col1.y;
    }

    // Operators
    Mat2 operator+(const Mat2& other) const {
        return Mat2(
            m[0][0] + other.m[0][0], m[0][1] + other.m[0][1],
            m[1][0] + other.m[1][0], m[1][1] + other.m[1][1]
        );
    }

    Mat2 operator-(const Mat2& other) const {
        return Mat2(
            m[0][0] - other.m[0][0], m[0][1] - other.m[0][1],
            m[1][0] - other.m[1][0], m[1][1] - other.m[1][1]
        );
    }

    Mat2 operator*(float scalar) const {
        return Mat2(
            m[0][0] * scalar, m[0][1] * scalar,
            m[1][0] * scalar, m[1][1] * scalar
        );
    }

    Mat2 operator/(float scalar) const {
        return Mat2(
            m[0][0] / scalar, m[0][1] / scalar,
            m[1][0] / scalar, m[1][1] / scalar
        );
    }

    /// Matrix multiplication
    Mat2 operator*(const Mat2& other) const {
        return Mat2(
            m[0][0] * other.m[0][0] + m[0][1] * other.m[1][0],
            m[0][0] * other.m[0][1] + m[0][1] * other.m[1][1],
            m[1][0] * other.m[0][0] + m[1][1] * other.m[1][0],
            m[1][0] * other.m[0][1] + m[1][1] * other.m[1][1]
        );
    }

    /// Matrix-vector multiplication
    Vec2 operator*(const Vec2& v) const {
        return Vec2(
            m[0][0] * v.x + m[0][1] * v.y,
            m[1][0] * v.x + m[1][1] * v.y
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

    Mat2& operator*=(float scalar) {
        m[0][0] *= scalar; m[0][1] *= scalar;
        m[1][0] *= scalar; m[1][1] *= scalar;
        return *this;
    }

    Mat2& operator/=(float scalar) {
        m[0][0] /= scalar; m[0][1] /= scalar;
        m[1][0] /= scalar; m[1][1] /= scalar;
        return *this;
    }

    Mat2& operator*=(const Mat2& other) {
        *this = *this * other;
        return *this;
    }

    bool operator==(const Mat2& other) const {
        return m[0][0] == other.m[0][0] && m[0][1] == other.m[0][1] &&
               m[1][0] == other.m[1][0] && m[1][1] == other.m[1][1];
    }

    bool operator!=(const Mat2& other) const {
        return !(*this == other);
    }

    float& operator()(int row, int col) {
        return m[row][col];
    }

    const float& operator()(int row, int col) const {
        return m[row][col];
    }

    /// Determinant
    float determinant() const {
        return m[0][0] * m[1][1] - m[0][1] * m[1][0];
    }

    /// Matrix inverse
    Mat2 inverse() const {
        float det = determinant();
        if (std::abs(det) < 1e-6f) {
            return Mat2(0.0f);  // Singular matrix, return zero
        }
        float invDet = 1.0f / det;
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
    float trace() const {
        return m[0][0] + m[1][1];
    }

    /// Create identity matrix
    static Mat2 identity() {
        return Mat2(
            1.0f, 0.0f,
            0.0f, 1.0f
        );
    }

    /// Create zero matrix
    static Mat2 zero() {
        return Mat2(
            0.0f, 0.0f,
            0.0f, 0.0f
        );
    }

    /// Create 2D rotation matrix (angle in radians)
    static Mat2 rotation(float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        return Mat2(
            c, -s,
            s,  c
        );
    }

    /// Create 2D scale matrix
    static Mat2 scale(float sx, float sy) {
        return Mat2(
            sx,  0.0f,
            0.0f, sy
        );
    }

    /// Create 2D scale matrix (uniform)
    static Mat2 scale(float s) {
        return scale(s, s);
    }
};

/// Scalar * Matrix multiplication
inline Mat2 operator*(float scalar, const Mat2& m) {
    return m * scalar;
}

/// Vector * Matrix multiplication (treats vector as row vector)
inline Vec2 operator*(const Vec2& v, const Mat2& m) {
    return Vec2(
        v.x * m.m[0][0] + v.y * m.m[1][0],
        v.x * m.m[0][1] + v.y * m.m[1][1]
    );
}

inline std::ostream& operator<<(std::ostream& os, const Mat2& m) {
    os << "[(" << m.m[0][0] << ", " << m.m[0][1] << "), "
       << "(" << m.m[1][0] << ", " << m.m[1][1] << ")]";
    return os;
}

}  // namespace phynity::math::matrices

#pragma once

#include <core/math/vectors/vec3.hpp>
#include <cmath>
#include <ostream>
#include <algorithm>

namespace phynity::math::matrices {

using phynity::math::vectors::Vec3;

/// 3×3 floating-point matrix for 3D transformations and rotations.
/// Storage is in row-major order: m[row][col]
struct Mat3 {
    float m[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    // Constructors
    Mat3() = default;

    /// Fill constructor - creates matrix with all elements set to scalar
    explicit Mat3(float scalar) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] = scalar;
            }
        }
    }

    /// Element-wise constructor
    Mat3(float m00, float m01, float m02,
         float m10, float m11, float m12,
         float m20, float m21, float m22) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    /// Column vector constructor (three column vectors)
    Mat3(const Vec3& col0, const Vec3& col1, const Vec3& col2) {
        m[0][0] = col0.x; m[0][1] = col1.x; m[0][2] = col2.x;
        m[1][0] = col0.y; m[1][1] = col1.y; m[1][2] = col2.y;
        m[2][0] = col0.z; m[2][1] = col1.z; m[2][2] = col2.z;
    }

    // Operators
    Mat3 operator+(const Mat3& other) const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = m[i][j] + other.m[i][j];
            }
        }
        return result;
    }

    Mat3 operator-(const Mat3& other) const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = m[i][j] - other.m[i][j];
            }
        }
        return result;
    }

    Mat3 operator*(float scalar) const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = m[i][j] * scalar;
            }
        }
        return result;
    }

    Mat3 operator/(float scalar) const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = m[i][j] / scalar;
            }
        }
        return result;
    }

    /// Matrix multiplication
    Mat3 operator*(const Mat3& other) const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    /// Matrix-vector multiplication
    Vec3 operator*(const Vec3& v) const {
        return Vec3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    Mat3 operator-() const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = -m[i][j];
            }
        }
        return result;
    }

    Mat3& operator+=(const Mat3& other) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] += other.m[i][j];
            }
        }
        return *this;
    }

    Mat3& operator-=(const Mat3& other) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] -= other.m[i][j];
            }
        }
        return *this;
    }

    Mat3& operator*=(float scalar) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] *= scalar;
            }
        }
        return *this;
    }

    Mat3& operator/=(float scalar) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] /= scalar;
            }
        }
        return *this;
    }

    Mat3& operator*=(const Mat3& other) {
        *this = *this * other;
        return *this;
    }

    /// Component-wise (Hadamard) multiplication
    Mat3& mulComponentWise(const Mat3& other) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] *= other.m[i][j];
            }
        }
        return *this;
    }

    /// Component-wise division
    Mat3& divComponentWise(const Mat3& other) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] /= other.m[i][j];
            }
        }
        return *this;
    }

    bool operator==(const Mat3& other) const {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (m[i][j] != other.m[i][j]) {
                    return false;
                }
            }
        }
        return true;
    }

    bool operator!=(const Mat3& other) const {
        return !(*this == other);
    }

    float& operator()(int row, int col) {
        return m[row][col];
    }

    const float& operator()(int row, int col) const {
        return m[row][col];
    }

    /// Get row as vector
    Vec3 getRow(int row) const {
        return Vec3(m[row][0], m[row][1], m[row][2]);
    }

    /// Get column as vector
    Vec3 getColumn(int col) const {
        return Vec3(m[0][col], m[1][col], m[2][col]);
    }

    /// Set row from vector
    void setRow(int row, const Vec3& v) {
        m[row][0] = v.x;
        m[row][1] = v.y;
        m[row][2] = v.z;
    }

    /// Set column from vector
    void setColumn(int col, const Vec3& v) {
        m[0][col] = v.x;
        m[1][col] = v.y;
        m[2][col] = v.z;
    }

    /// Determinant
    float determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    /// Matrix inverse
    Mat3 inverse() const {
        float det = determinant();
        if (std::abs(det) < 1e-6f) {
            return Mat3(0.0f);  // Singular matrix, return zero
        }

        // Compute cofactor matrix
        Mat3 cof;
        cof.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
        cof.m[0][1] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]);
        cof.m[0][2] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        cof.m[1][0] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]);
        cof.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]);
        cof.m[1][2] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]);

        cof.m[2][0] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]);
        cof.m[2][1] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]);
        cof.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]);

        // Adjugate is transpose of cofactor matrix
        Mat3 adj = cof.transposed();
        return adj / det;
    }

    /// Matrix transpose
    Mat3 transposed() const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = m[j][i];
            }
        }
        return result;
    }

    Mat3& transpose() {
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                std::swap(m[i][j], m[j][i]);
            }
        }
        return *this;
    }

    /// Trace (sum of diagonal elements)
    float trace() const {
        return m[0][0] + m[1][1] + m[2][2];
    }

    /// Approximate equality with epsilon tolerance
    bool approxEqual(const Mat3& other, float epsilon = 1e-5f) const {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (std::abs(m[i][j] - other.m[i][j]) >= epsilon) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Return matrix with absolute values of all elements
    Mat3 abs() const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = std::abs(m[i][j]);
            }
        }
        return result;
    }

    /// Create identity matrix
    static Mat3 identity() {
        return Mat3(
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        );
    }

    /// Create zero matrix
    static Mat3 zero() {
        return Mat3(
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        );
    }

    /// Create 3D rotation matrix around X axis (angle in radians)
    static Mat3 rotationX(float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        return Mat3(
            1.0f, 0.0f, 0.0f,
            0.0f, c,    -s,
            0.0f, s,    c
        );
    }

    /// Create 3D rotation matrix around Y axis (angle in radians)
    static Mat3 rotationY(float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        return Mat3(
            c,    0.0f, s,
            0.0f, 1.0f, 0.0f,
            -s,   0.0f, c
        );
    }

    /// Create 3D rotation matrix around Z axis (angle in radians)
    static Mat3 rotationZ(float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        return Mat3(
            c,    -s,   0.0f,
            s,    c,    0.0f,
            0.0f, 0.0f, 1.0f
        );
    }

    /// Create rotation matrix around an arbitrary axis (angle in radians, axis must be normalized)
    static Mat3 rotationAxis(const Vec3& axis, float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        float omc = 1.0f - c;

        return Mat3(
            c + axis.x * axis.x * omc,
            axis.x * axis.y * omc - axis.z * s,
            axis.x * axis.z * omc + axis.y * s,

            axis.y * axis.x * omc + axis.z * s,
            c + axis.y * axis.y * omc,
            axis.y * axis.z * omc - axis.x * s,

            axis.z * axis.x * omc - axis.y * s,
            axis.z * axis.y * omc + axis.x * s,
            c + axis.z * axis.z * omc
        );
    }

    /// Create 3D scale matrix
    static Mat3 scale(float sx, float sy, float sz) {
        return Mat3(
            sx,   0.0f, 0.0f,
            0.0f, sy,   0.0f,
            0.0f, 0.0f, sz
        );
    }

    /// Create 3D scale matrix (uniform)
    static Mat3 scale(float s) {
        return scale(s, s, s);
    }
};

/// Scalar * Matrix multiplication
inline Mat3 operator*(float scalar, const Mat3& m) {
    return m * scalar;
}

/// Vector * Matrix multiplication (treats vector as row vector)
inline Vec3 operator*(const Vec3& v, const Mat3& m) {
    return Vec3(
        v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0],
        v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1],
        v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2]
    );
}

inline std::ostream& operator<<(std::ostream& os, const Mat3& m) {
    os << "[(" << m.m[0][0] << ", " << m.m[0][1] << ", " << m.m[0][2] << "), "
       << "(" << m.m[1][0] << ", " << m.m[1][1] << ", " << m.m[1][2] << "), "
       << "(" << m.m[2][0] << ", " << m.m[2][1] << ", " << m.m[2][2] << ")]";
    return os;
}

}  // namespace phynity::math::matrices

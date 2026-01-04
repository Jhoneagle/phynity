#pragma once

#include <core/math/vectors/vec3.hpp>
#include <cmath>
#include <ostream>
#include <algorithm>
#include <type_traits>

namespace phynity::math::matrices {

/// 3×3 floating-point matrix for 3D transformations and rotations with dual-precision support.
/// Storage is in row-major order: m[row][col]
template<typename T = float>
struct Mat3 {
    static_assert(std::is_floating_point_v<T>, "Mat3 template parameter must be a floating-point type");
    T m[3][3] = {{T(1), T(0), T(0)}, {T(0), T(1), T(0)}, {T(0), T(0), T(1)}};

    // Constructors
    Mat3() = default;

    /// Fill constructor - creates matrix with all elements set to scalar
    explicit Mat3(T scalar) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] = scalar;
            }
        }
    }

    /// Element-wise constructor
    Mat3(T m00, T m01, T m02,
         T m10, T m11, T m12,
         T m20, T m21, T m22) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    /// Column vector constructor (three column vectors)
    Mat3(const phynity::math::vectors::Vec3<T>& col0, const phynity::math::vectors::Vec3<T>& col1, const phynity::math::vectors::Vec3<T>& col2) {
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

    Mat3 operator*(T scalar) const {
        Mat3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m[i][j] = m[i][j] * scalar;
            }
        }
        return result;
    }

    Mat3 operator/(T scalar) const {
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
                result.m[i][j] = T(0);
                for (int k = 0; k < 3; ++k) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    /// Matrix-vector multiplication
    phynity::math::vectors::Vec3<T> operator*(const phynity::math::vectors::Vec3<T>& v) const {
        return phynity::math::vectors::Vec3<T>(
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

    Mat3& operator*=(T scalar) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] *= scalar;
            }
        }
        return *this;
    }

    Mat3& operator/=(T scalar) {
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
    phynity::math::vectors::Vec3<T> getRow(int row) const {
        return phynity::math::vectors::Vec3<T>(m[row][0], m[row][1], m[row][2]);
    }

    /// Get column as vector
    phynity::math::vectors::Vec3<T> getColumn(int col) const {
        return phynity::math::vectors::Vec3<T>(m[0][col], m[1][col], m[2][col]);
    }

    /// Set row from vector
    void setRow(int row, const phynity::math::vectors::Vec3<T>& v) {
        m[row][0] = v.x;
        m[row][1] = v.y;
        m[row][2] = v.z;
    }

    /// Set column from vector
    void setColumn(int col, const phynity::math::vectors::Vec3<T>& v) {
        m[0][col] = v.x;
        m[1][col] = v.y;
        m[2][col] = v.z;
    }

    /// Determinant
    T determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    /// Matrix inverse
    Mat3 inverse() const {
        T det = determinant();
        if (std::abs(det) < T(1e-6)) {
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
    T trace() const {
        return m[0][0] + m[1][1] + m[2][2];
    }

    /// Approximate equality with epsilon tolerance
    bool approxEqual(const Mat3& other, T epsilon = T(1e-5)) const {
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
            T(1), T(0), T(0),
            T(0), T(1), T(0),
            T(0), T(0), T(1)
        );
    }

    /// Create zero matrix
    static Mat3 zero() {
        return Mat3(
            T(0), T(0), T(0),
            T(0), T(0), T(0),
            T(0), T(0), T(0)
        );
    }

    /// Create 3D rotation matrix around X axis (angle in radians)
    static Mat3 rotationX(T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        return Mat3(
            T(1), T(0), T(0),
            T(0), c,    -s,
            T(0), s,    c
        );
    }

    /// Create 3D rotation matrix around Y axis (angle in radians)
    static Mat3 rotationY(T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        return Mat3(
            c,    T(0), s,
            T(0), T(1), T(0),
            -s,   T(0), c
        );
    }

    /// Create 3D rotation matrix around Z axis (angle in radians)
    static Mat3 rotationZ(T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        return Mat3(
            c,    -s,   T(0),
            s,    c,    T(0),
            T(0), T(0), T(1)
        );
    }

    /// Create rotation matrix around an arbitrary axis (angle in radians, axis must be normalized)
    static Mat3 rotationAxis(const phynity::math::vectors::Vec3<T>& axis, T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        T omc = T(1) - c;

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
    static Mat3 scale(T sx, T sy, T sz) {
        return Mat3(
            sx,   T(0), T(0),
            T(0), sy,   T(0),
            T(0), T(0), sz
        );
    }

    /// Create 3D scale matrix (uniform)
    static Mat3 scale(T s) {
        return scale(s, s, s);
    }
};

/// Scalar * Matrix multiplication
template<typename T = float>
inline Mat3<T> operator*(T scalar, const Mat3<T>& m) {
    return m * scalar;
}

/// Vector * Matrix multiplication (treats vector as row vector)
template<typename T = float>
inline phynity::math::vectors::Vec3<T> operator*(const phynity::math::vectors::Vec3<T>& v, const Mat3<T>& m) {
    return phynity::math::vectors::Vec3<T>(
        v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0],
        v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1],
        v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2]
    );
}

/// Matrix * Vector multiplication (treats vector as column vector) - for Vec3
template<typename T = float>
inline phynity::math::vectors::Vec3<T> operator*(const Mat3<T>& m, const phynity::math::vectors::Vec3<T>& v) {
    return m * v;
}

template<typename T = float>
inline std::ostream& operator<<(std::ostream& os, const Mat3<T>& m) {
    os << "[(" << m.m[0][0] << ", " << m.m[0][1] << ", " << m.m[0][2] << "), "
       << "(" << m.m[1][0] << ", " << m.m[1][1] << ", " << m.m[1][2] << "), "
       << "(" << m.m[2][0] << ", " << m.m[2][1] << ", " << m.m[2][2] << ")]";
    return os;
}

// Type aliases
using Mat3f = Mat3<float>;
using Mat3d = Mat3<double>;

}  // namespace phynity::math::matrices

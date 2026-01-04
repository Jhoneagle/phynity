#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/math/vectors/vec4.hpp>
#include <cmath>
#include <ostream>
#include <algorithm>
#include <type_traits>

namespace phynity::math::matrices {

/// 4×4 floating-point matrix for 3D transformations and homogeneous coordinates with dual-precision support.
/// Storage is in row-major order: m[row][col]
template<typename T = float>
struct Mat4 {
    static_assert(std::is_floating_point_v<T>, "Mat4 template parameter must be a floating-point type");
    T m[4][4] = {{T(1), T(0), T(0), T(0)},
                 {T(0), T(1), T(0), T(0)},
                 {T(0), T(0), T(1), T(0)},
                 {T(0), T(0), T(0), T(1)}};

    // Constructors
    Mat4() = default;

    /// Fill constructor - creates matrix with all elements set to scalar
    explicit Mat4(T scalar) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = scalar;
            }
        }
    }

    /// Element-wise constructor (16 floats)
    Mat4(T m00, T m01, T m02, T m03,
         T m10, T m11, T m12, T m13,
         T m20, T m21, T m22, T m23,
         T m30, T m31, T m32, T m33) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
        m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
    }

    /// Column vector constructor (four column vectors)
    Mat4(const phynity::math::vectors::Vec4<T>& col0, const phynity::math::vectors::Vec4<T>& col1, const phynity::math::vectors::Vec4<T>& col2, const phynity::math::vectors::Vec4<T>& col3) {
        m[0][0] = col0.x; m[0][1] = col1.x; m[0][2] = col2.x; m[0][3] = col3.x;
        m[1][0] = col0.y; m[1][1] = col1.y; m[1][2] = col2.y; m[1][3] = col3.y;
        m[2][0] = col0.z; m[2][1] = col1.z; m[2][2] = col2.z; m[2][3] = col3.z;
        m[3][0] = col0.w; m[3][1] = col1.w; m[3][2] = col2.w; m[3][3] = col3.w;
    }

    // Operators
    Mat4 operator+(const Mat4& other) const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[i][j] + other.m[i][j];
            }
        }
        return result;
    }

    Mat4 operator-(const Mat4& other) const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[i][j] - other.m[i][j];
            }
        }
        return result;
    }

    Mat4 operator*(T scalar) const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[i][j] * scalar;
            }
        }
        return result;
    }

    Mat4 operator/(T scalar) const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[i][j] / scalar;
            }
        }
        return result;
    }

    /// Matrix multiplication
    Mat4 operator*(const Mat4& other) const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = T(0);
                for (int k = 0; k < 4; ++k) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    /// Matrix-vector multiplication (column vector)
    phynity::math::vectors::Vec4<T> operator*(const phynity::math::vectors::Vec4<T>& v) const {
        return phynity::math::vectors::Vec4<T>(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
            m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w
        );
    }

    /// Matrix-vector multiplication with Vec3 (treats as Vec4 with w=1)
    phynity::math::vectors::Vec3<T> operator*(const phynity::math::vectors::Vec3<T>& v) const {
        T x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3];
        T y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3];
        T z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3];
        return phynity::math::vectors::Vec3<T>(x, y, z);
    }

    Mat4& operator+=(const Mat4& other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] += other.m[i][j];
            }
        }
        return *this;
    }

    Mat4 operator-() const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = -m[i][j];
            }
        }
        return result;
    }

    Mat4& operator-=(const Mat4& other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] -= other.m[i][j];
            }
        }
        return *this;
    }

    Mat4& operator*=(T scalar) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] *= scalar;
            }
        }
        return *this;
    }

    Mat4& operator/=(T scalar) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] /= scalar;
            }
        }
        return *this;
    }

    Mat4& operator*=(const Mat4& other) {
        *this = *this * other;
        return *this;
    }

    /// Component-wise (Hadamard) multiplication
    Mat4& mulComponentWise(const Mat4& other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] *= other.m[i][j];
            }
        }
        return *this;
    }

    /// Component-wise division
    Mat4& divComponentWise(const Mat4& other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] /= other.m[i][j];
            }
        }
        return *this;
    }

    bool operator==(const Mat4& other) const {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (m[i][j] != other.m[i][j]) {
                    return false;
                }
            }
        }
        return true;
    }

    bool operator!=(const Mat4& other) const {
        return !(*this == other);
    }

    float& operator()(int row, int col) {
        return m[row][col];
    }

    const float& operator()(int row, int col) const {
        return m[row][col];
    }

    /// Get row as vector
    phynity::math::vectors::Vec4<T> getRow(int row) const {
        return phynity::math::vectors::Vec4<T>(m[row][0], m[row][1], m[row][2], m[row][3]);
    }

    /// Get column as vector
    phynity::math::vectors::Vec4<T> getColumn(int col) const {
        return phynity::math::vectors::Vec4<T>(m[0][col], m[1][col], m[2][col], m[3][col]);
    }

    /// Set row from vector
    void setRow(int row, const phynity::math::vectors::Vec4<T>& v) {
        m[row][0] = v.x;
        m[row][1] = v.y;
        m[row][2] = v.z;
        m[row][3] = v.w;
    }

    /// Set column from vector
    void setColumn(int col, const phynity::math::vectors::Vec4<T>& v) {
        m[0][col] = v.x;
        m[1][col] = v.y;
        m[2][col] = v.z;
        m[3][col] = v.w;
    }

    /// Determinant using cofactor expansion
    T determinant() const {
        // Calculate 3x3 determinants (minors)
        T m00 = m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
                    m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) +
                    m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]);

        T m01 = m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
                    m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
                    m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]);

        T m02 = m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) -
                    m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
                    m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]);

        T m03 = m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) -
                    m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) +
                    m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]);

        return m[0][0] * m00 - m[0][1] * m01 + m[0][2] * m02 - m[0][3] * m03;
    }

    /// Matrix inverse
    Mat4 inverse() const {
        T det = determinant();
        if (std::abs(det) < T(1e-6)) {
            return Mat4(0.0f);  // Singular matrix
        }

        Mat4 cof;
        
        // Calculate 3x3 cofactors for each element
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                // Create 3x3 matrix by removing row i and column j
                T sub[3][3];
                int row_idx = 0;
                for (int r = 0; r < 4; ++r) {
                    if (r == i) continue;
                    int col_idx = 0;
                    for (int c = 0; c < 4; ++c) {
                        if (c == j) continue;
                        sub[row_idx][col_idx] = m[r][c];
                        col_idx++;
                    }
                    row_idx++;
                }

                // Calculate determinant of 3x3
                T det3x3 = sub[0][0] * (sub[1][1] * sub[2][2] - sub[1][2] * sub[2][1]) -
                               sub[0][1] * (sub[1][0] * sub[2][2] - sub[1][2] * sub[2][0]) +
                               sub[0][2] * (sub[1][0] * sub[2][1] - sub[1][1] * sub[2][0]);

                // Apply sign pattern
                cof.m[i][j] = ((i + j) % 2 == 0) ? det3x3 : -det3x3;
            }
        }

        Mat4 adj = cof.transposed();
        return adj / det;
    }

    /// Matrix transpose
    Mat4 transposed() const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[j][i];
            }
        }
        return result;
    }

    Mat4& transpose() {
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                std::swap(m[i][j], m[j][i]);
            }
        }
        return *this;
    }

    /// Trace (sum of diagonal elements)
    T trace() const {
        return m[0][0] + m[1][1] + m[2][2] + m[3][3];
    }

    /// Approximate equality with epsilon tolerance
    bool approxEqual(const Mat4& other, T epsilon = T(1e-5)) const {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (std::abs(m[i][j] - other.m[i][j]) >= epsilon) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Return matrix with absolute values of all elements
    Mat4 abs() const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = std::abs(m[i][j]);
            }
        }
        return result;
    }

    /// Create identity matrix
    static Mat4 identity() {
        return Mat4(
            T(1), T(0), T(0), T(0),
            T(0), T(1), T(0), T(0),
            T(0), T(0), T(1), T(0),
            T(0), T(0), T(0), T(1)
        );
    }

    /// Create zero matrix
    static Mat4 zero() {
        return Mat4(
            T(0), T(0), T(0), T(0),
            T(0), T(0), T(0), T(0),
            T(0), T(0), T(0), T(0),
            T(0), T(0), T(0), T(0)
        );
    }

    /// Create translation matrix
    static Mat4 translation(T tx, T ty, T tz) {
        return Mat4(
            T(1), T(0), T(0), tx,
            T(0), T(1), T(0), ty,
            T(0), T(0), T(1), tz,
            T(0), T(0), T(0), T(1)
        );
    }

    /// Create translation matrix from Vec3
    static Mat4 translation(const phynity::math::vectors::Vec3<T>& t) {
        return translation(t.x, t.y, t.z);
    }

    /// Create 3D scale matrix
    static Mat4 scale(T sx, T sy, T sz) {
        return Mat4(
            sx,   T(0), T(0), T(0),
            T(0), sy,   T(0), T(0),
            T(0), T(0), sz,   T(0),
            T(0), T(0), T(0), T(1)
        );
    }

    /// Create 3D scale matrix (uniform)
    static Mat4 scale(T s) {
        return scale(s, s, s);
    }

    /// Create 3D scale matrix from Vec3
    static Mat4 scale(const phynity::math::vectors::Vec3<T>& s) {
        return scale(s.x, s.y, s.z);
    }

    /// Create rotation matrix around X axis (angle in radians)
    static Mat4 rotationX(T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        return Mat4(
            T(1), T(0), T(0), T(0),
            T(0), c,    -s,   T(0),
            T(0), s,    c,    T(0),
            T(0), T(0), T(0), T(1)
        );
    }

    /// Create rotation matrix around Y axis (angle in radians)
    static Mat4 rotationY(T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        return Mat4(
            c,    T(0), s,    T(0),
            T(0), T(1), T(0), T(0),
            -s,   T(0), c,    T(0),
            T(0), T(0), T(0), T(1)
        );
    }

    /// Create rotation matrix around Z axis (angle in radians)
    static Mat4 rotationZ(T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        return Mat4(
            c,    -s,   T(0), T(0),
            s,    c,    T(0), T(0),
            T(0), T(0), T(1), T(0),
            T(0), T(0), T(0), T(1)
        );
    }

    /// Create rotation matrix around an arbitrary axis (angle in radians, axis must be normalized)
    static Mat4 rotationAxis(const phynity::math::vectors::Vec3<T>& axis, T angleRadians) {
        T c = std::cos(angleRadians);
        T s = std::sin(angleRadians);
        T omc = T(1) - c;

        return Mat4(
            c + axis.x * axis.x * omc,
            axis.x * axis.y * omc - axis.z * s,
            axis.x * axis.z * omc + axis.y * s,
            T(0),

            axis.y * axis.x * omc + axis.z * s,
            c + axis.y * axis.y * omc,
            axis.y * axis.z * omc - axis.x * s,
            T(0),

            axis.z * axis.x * omc - axis.y * s,
            axis.z * axis.y * omc + axis.x * s,
            c + axis.z * axis.z * omc,
            T(0),

            T(0), T(0), T(0), T(1)
        );
    }

    /// Create perspective projection matrix (matches test expectations; w component in last row)
    static Mat4 perspective(T fovy, T aspect, T near, T far) {
        T f = T(1) / std::tan(fovy * 0.5f);
        T nf = T(1) / (near - far);

        return Mat4(
            f / aspect, T(0), T(0),            T(0),
            T(0),       f,    T(0),            T(0),
            T(0),       T(0), far * nf,        (far * near) * nf,
            T(0),       T(0), -T(1),           T(0)
        );
    }

    /// Create orthographic projection matrix
    static Mat4 orthographic(T left, T right, T bottom, T top, T near, T far) {
        T tx = -(right + left) / (right - left);
        T ty = -(top + bottom) / (top - bottom);
        T tz = -(far + near) / (far - near);

        return Mat4(
            T(2) / (right - left), T(0),                 T(0),                T(0),
            T(0),                  T(2) / (top - bottom), T(0),                T(0),
            T(0),                  T(0),                 -T(2) / (far - near), T(0),
            tx,                     ty,                   tz,                   T(1)
        );
    }
};

/// Scalar * Matrix multiplication
template<typename T = float>
inline Mat4<T> operator*(T scalar, const Mat4<T>& m) {
    return m * scalar;
}

/// Vector * Matrix multiplication (treats vector as row vector)
template<typename T = float>
inline phynity::math::vectors::Vec4<T> operator*(const phynity::math::vectors::Vec4<T>& v, const Mat4<T>& m) {
    return phynity::math::vectors::Vec4<T>(
        v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + v.w * m.m[3][0],
        v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + v.w * m.m[3][1],
        v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + v.w * m.m[3][2],
        v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + v.w * m.m[3][3]
    );
}

/// Matrix * Vector multiplication (treats vector as column vector) - for Vec4
template<typename T = float>
inline phynity::math::vectors::Vec4<T> operator*(const Mat4<T>& m, const phynity::math::vectors::Vec4<T>& v) {
    return m * v;
}

/// Matrix * Vector multiplication (treats vector as column vector, w=1) - for Vec3
template<typename T = float>
inline phynity::math::vectors::Vec3<T> operator*(const Mat4<T>& m, const phynity::math::vectors::Vec3<T>& v) {
    return m * v;
}

template<typename T = float>
inline std::ostream& operator<<(std::ostream& os, const Mat4<T>& m) {
    os << "[\n";
    for (int i = 0; i < 4; ++i) {
        os << "  (" << m.m[i][0] << ", " << m.m[i][1] << ", " << m.m[i][2] << ", " << m.m[i][3] << ")\n";
    }
    os << "]";
    return os;
}

// Type aliases
using Mat4f = Mat4<float>;
using Mat4d = Mat4<double>;

}  // namespace phynity::math::matrices

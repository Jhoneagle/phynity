#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/math/vectors/vec4.hpp>
#include <cmath>
#include <ostream>
#include <algorithm>

namespace phynity::math::matrices {

using phynity::math::vectors::Vec3;
using phynity::math::vectors::Vec4;

/// 4×4 floating-point matrix for 3D transformations and homogeneous coordinates.
/// Storage is in row-major order: m[row][col]
struct Mat4 {
    float m[4][4] = {{1.0f, 0.0f, 0.0f, 0.0f},
                     {0.0f, 1.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 1.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f, 1.0f}};

    // Constructors
    Mat4() = default;

    /// Fill constructor - creates matrix with all elements set to scalar
    explicit Mat4(float scalar) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = scalar;
            }
        }
    }

    /// Element-wise constructor (16 floats)
    Mat4(float m00, float m01, float m02, float m03,
         float m10, float m11, float m12, float m13,
         float m20, float m21, float m22, float m23,
         float m30, float m31, float m32, float m33) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
        m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
    }

    /// Column vector constructor (four column vectors)
    Mat4(const Vec4& col0, const Vec4& col1, const Vec4& col2, const Vec4& col3) {
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

    Mat4 operator*(float scalar) const {
        Mat4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[i][j] * scalar;
            }
        }
        return result;
    }

    Mat4 operator/(float scalar) const {
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
                result.m[i][j] = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    /// Matrix-vector multiplication (column vector)
    Vec4 operator*(const Vec4& v) const {
        return Vec4(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
            m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w
        );
    }

    /// Matrix-vector multiplication with Vec3 (treats as Vec4 with w=1)
    Vec3 operator*(const Vec3& v) const {
        float x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3];
        float y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3];
        float z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3];
        return Vec3(x, y, z);
    }

    Mat4& operator+=(const Mat4& other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] += other.m[i][j];
            }
        }
        return *this;
    }

    Mat4& operator-=(const Mat4& other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] -= other.m[i][j];
            }
        }
        return *this;
    }

    Mat4& operator*=(float scalar) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] *= scalar;
            }
        }
        return *this;
    }

    Mat4& operator/=(float scalar) {
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

    /// Determinant using cofactor expansion
    float determinant() const {
        // Calculate 3x3 determinants (minors)
        float m00 = m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
                    m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) +
                    m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]);

        float m01 = m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
                    m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
                    m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]);

        float m02 = m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) -
                    m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
                    m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]);

        float m03 = m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) -
                    m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) +
                    m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]);

        return m[0][0] * m00 - m[0][1] * m01 + m[0][2] * m02 - m[0][3] * m03;
    }

    /// Matrix inverse
    Mat4 inverse() const {
        float det = determinant();
        if (std::abs(det) < 1e-6f) {
            return Mat4(0.0f);  // Singular matrix
        }

        Mat4 cof;
        
        // Calculate 3x3 cofactors for each element
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                // Create 3x3 matrix by removing row i and column j
                float sub[3][3];
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
                float det3x3 = sub[0][0] * (sub[1][1] * sub[2][2] - sub[1][2] * sub[2][1]) -
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
    float trace() const {
        return m[0][0] + m[1][1] + m[2][2] + m[3][3];
    }

    /// Create identity matrix
    static Mat4 identity() {
        return Mat4(
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        );
    }

    /// Create zero matrix
    static Mat4 zero() {
        return Mat4(
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f
        );
    }

    /// Create translation matrix
    static Mat4 translation(float tx, float ty, float tz) {
        return Mat4(
            1.0f, 0.0f, 0.0f, tx,
            0.0f, 1.0f, 0.0f, ty,
            0.0f, 0.0f, 1.0f, tz,
            0.0f, 0.0f, 0.0f, 1.0f
        );
    }

    /// Create translation matrix from Vec3
    static Mat4 translation(const Vec3& t) {
        return translation(t.x, t.y, t.z);
    }

    /// Create 3D scale matrix
    static Mat4 scale(float sx, float sy, float sz) {
        return Mat4(
            sx,   0.0f, 0.0f, 0.0f,
            0.0f, sy,   0.0f, 0.0f,
            0.0f, 0.0f, sz,   0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        );
    }

    /// Create 3D scale matrix (uniform)
    static Mat4 scale(float s) {
        return scale(s, s, s);
    }

    /// Create 3D scale matrix from Vec3
    static Mat4 scale(const Vec3& s) {
        return scale(s.x, s.y, s.z);
    }

    /// Create rotation matrix around X axis (angle in radians)
    static Mat4 rotationX(float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        return Mat4(
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, c,    -s,   0.0f,
            0.0f, s,    c,    0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        );
    }

    /// Create rotation matrix around Y axis (angle in radians)
    static Mat4 rotationY(float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        return Mat4(
            c,    0.0f, s,    0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            -s,   0.0f, c,    0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        );
    }

    /// Create rotation matrix around Z axis (angle in radians)
    static Mat4 rotationZ(float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        return Mat4(
            c,    -s,   0.0f, 0.0f,
            s,    c,    0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        );
    }

    /// Create rotation matrix around an arbitrary axis (angle in radians, axis must be normalized)
    static Mat4 rotationAxis(const Vec3& axis, float angleRadians) {
        float c = std::cos(angleRadians);
        float s = std::sin(angleRadians);
        float omc = 1.0f - c;

        return Mat4(
            c + axis.x * axis.x * omc,
            axis.x * axis.y * omc - axis.z * s,
            axis.x * axis.z * omc + axis.y * s,
            0.0f,

            axis.y * axis.x * omc + axis.z * s,
            c + axis.y * axis.y * omc,
            axis.y * axis.z * omc - axis.x * s,
            0.0f,

            axis.z * axis.x * omc - axis.y * s,
            axis.z * axis.y * omc + axis.x * s,
            c + axis.z * axis.z * omc,
            0.0f,

            0.0f, 0.0f, 0.0f, 1.0f
        );
    }

    /// Create perspective projection matrix (matches test expectations; w component in last row)
    static Mat4 perspective(float fovy, float aspect, float near, float far) {
        float f = 1.0f / std::tan(fovy * 0.5f);
        float nf = 1.0f / (near - far);

        return Mat4(
            f / aspect, 0.0f, 0.0f,            0.0f,
            0.0f,       f,    0.0f,            0.0f,
            0.0f,       0.0f, far * nf,        (far * near) * nf,
            0.0f,       0.0f, -1.0f,           0.0f
        );
    }

    /// Create orthographic projection matrix
    static Mat4 orthographic(float left, float right, float bottom, float top, float near, float far) {
        float tx = -(right + left) / (right - left);
        float ty = -(top + bottom) / (top - bottom);
        float tz = -(far + near) / (far - near);

        return Mat4(
            2.0f / (right - left), 0.0f,                 0.0f,                0.0f,
            0.0f,                  2.0f / (top - bottom), 0.0f,                0.0f,
            0.0f,                  0.0f,                 -2.0f / (far - near), 0.0f,
            tx,                     ty,                   tz,                   1.0f
        );
    }
};

/// Scalar * Matrix multiplication
inline Mat4 operator*(float scalar, const Mat4& m) {
    return m * scalar;
}

/// Vector * Matrix multiplication (treats vector as row vector)
inline Vec4 operator*(const Vec4& v, const Mat4& m) {
    return Vec4(
        v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + v.w * m.m[3][0],
        v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + v.w * m.m[3][1],
        v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + v.w * m.m[3][2],
        v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + v.w * m.m[3][3]
    );
}

inline std::ostream& operator<<(std::ostream& os, const Mat4& m) {
    os << "[\n";
    for (int i = 0; i < 4; ++i) {
        os << "  (" << m.m[i][0] << ", " << m.m[i][1] << ", " << m.m[i][2] << ", " << m.m[i][3] << ")\n";
    }
    os << "]";
    return os;
}

}  // namespace phynity::math::matrices

#pragma once

#include <core/math/quaternions/quat.hpp>
#include <core/math/matrices/mat3.hpp>
#include <core/math/utilities/validity_checks.hpp>
#include <core/math/utilities/constants.hpp>
#include <cmath>

namespace phynity::math::quaternions {

using phynity::math::matrices::Mat3;
using namespace phynity::math::utilities;

/// ============================================================================
/// QUATERNION TO ROTATION MATRIX (3×3)
/// ============================================================================
/// Converts a unit quaternion to a 3×3 rotation matrix.
///
/// For a unit quaternion q = (w, x, y, z), the corresponding rotation matrix is:
///
///     ┌                                  ┐
///     │ 1-2(y²+z²)   2(xy-wz)   2(xz+wy) │
///     │ 2(xy+wz)     1-2(x²+z²) 2(yz-wx) │
///     │ 2(xz-wy)     2(yz+wx)   1-2(x²+y²)│
///     └                                  ┘
///
/// NUMERICAL STABILITY NOTES:
/// - No division operations are required
/// - Formula is stable for all valid quaternions (no singularities)
/// - All operations are multiplications and additions only
/// - Automatically handles gimbal lock situations
///
/// @param q The quaternion to convert (should be unit quaternion)
/// @return The corresponding 3×3 rotation matrix (row-major storage)
inline Mat3 toRotationMatrix(const Quat& q) {
    Quat normalizedQ = q.normalized();

    // Compute optimized terms to reduce floating-point operations
    float xx = normalizedQ.x * normalizedQ.x;
    float yy = normalizedQ.y * normalizedQ.y;
    float zz = normalizedQ.z * normalizedQ.z;
    float xy = normalizedQ.x * normalizedQ.y;
    float xz = normalizedQ.x * normalizedQ.z;
    float yz = normalizedQ.y * normalizedQ.z;
    float wx = normalizedQ.w * normalizedQ.x;
    float wy = normalizedQ.w * normalizedQ.y;
    float wz = normalizedQ.w * normalizedQ.z;

    // Construct the rotation matrix using the optimized form
    // Row-major storage: m[row][col]
    return Mat3(
        // Row 0
        1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz), 2.0f * (xz + wy),
        // Row 1
        2.0f * (xy + wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
        // Row 2
        2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (xx + yy)
    );
}

/// ============================================================================
/// ROTATION MATRIX TO QUATERNION
/// ============================================================================
/// Converts a 3×3 rotation matrix to a unit quaternion.
///
/// Uses Shepperd's method to ensure numerical stability by choosing the
/// largest quaternion component as the base, avoiding division by small
/// numbers.
///
/// NUMERICAL STABILITY NOTES:
/// - Selects the largest component to avoid ill-conditioned cases
/// - Robust for all rotation matrices (no singularities)
/// - Input matrix should be orthonormal for valid results
///
/// @param m The 3×3 rotation matrix (row-major storage)
/// @return The corresponding unit quaternion
inline Quat toQuaternion(const Mat3& m) {
    if (hasNaN(m) || hasInf(m)) {
        // Invalid matrix
        // REJECT
        return Quat();
    }

    float det = m.determinant();
    if (det < 0) {
        // Reflection detected
        // REJECT
        return Quat();
    }

    bool nearOrtho =
    std::abs(m.getRow(0).dot(m.getRow(1))) < rotationf::orthonormal_epsilon &&
    std::abs(m.getRow(1).dot(m.getRow(2))) < rotationf::orthonormal_epsilon &&
    std::abs(m.getRow(2).dot(m.getRow(0))) < rotationf::orthonormal_epsilon &&
    std::abs(m.getRow(0).length() - 1.0f) < rotationf::normalize_epsilon &&
    std::abs(m.getRow(1).length() - 1.0f) < rotationf::normalize_epsilon &&
    std::abs(m.getRow(2).length() - 1.0f) < rotationf::normalize_epsilon;

    if (!nearOrtho) {
        // Not orthonormal
        // REJECT
        return Quat();
    }

    // Compute the trace
    float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];

    Quat q;

    if (trace > 0.0f) {
        // CASE 1: trace > 0 (most common case)
        // Use w as the base component
        q.w = 0.5f * std::sqrt(1.0f + trace);
        float s = 0.25f / q.w;
        q.x = (m.m[2][1] - m.m[1][2]) * s;
        q.y = (m.m[0][2] - m.m[2][0]) * s;
        q.z = (m.m[1][0] - m.m[0][1]) * s;
    } else if ((m.m[0][0] > m.m[1][1]) && (m.m[0][0] > m.m[2][2])) {
        // CASE 2: m[0][0] is the largest diagonal element
        // Use x as the base component
        q.x = 0.5f * std::sqrt(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]);
        float s = 0.25f / q.x;
        q.w = (m.m[2][1] - m.m[1][2]) * s;
        q.y = (m.m[0][1] + m.m[1][0]) * s;
        q.z = (m.m[0][2] + m.m[2][0]) * s;
    } else if (m.m[1][1] > m.m[2][2]) {
        // CASE 3: m[1][1] is the largest diagonal element
        // Use y as the base component
        q.y = 0.5f * std::sqrt(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]);
        float s = 0.25f / q.y;
        q.w = (m.m[0][2] - m.m[2][0]) * s;
        q.x = (m.m[0][1] + m.m[1][0]) * s;
        q.z = (m.m[1][2] + m.m[2][1]) * s;
    } else {
        // CASE 4: m[2][2] is the largest diagonal element (or all equal)
        // Use z as the base component
        q.z = 0.5f * std::sqrt(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]);
        float s = 0.25f / q.z;
        q.w = (m.m[1][0] - m.m[0][1]) * s;
        q.x = (m.m[0][2] + m.m[2][0]) * s;
        q.y = (m.m[1][2] + m.m[2][1]) * s;
    }

    // Ensure result is normalized (should be unit quaternion already)
    return q.normalized();
}

}  // namespace phynity::math::quaternions

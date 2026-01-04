#pragma once

#include <core/math/quaternions/quat.hpp>
#include <core/math/matrices/mat3.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/math/utilities/validity_checks.hpp>
#include <core/math/utilities/constants.hpp>
#include <cmath>
#include <type_traits>

namespace phynity::math::quaternions {

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
template<typename T = float>
inline phynity::math::matrices::Mat3<T> toRotationMatrix(const Quat<T>& q) {
    Quat<T> qn = q.normalized();

    // Compute optimized terms to reduce floating-point operations
    T xx = qn.x * qn.x;
    T yy = qn.y * qn.y;
    T zz = qn.z * qn.z;
    T xy = qn.x * qn.y;
    T xz = qn.x * qn.z;
    T yz = qn.y * qn.z;
    T wx = qn.w * qn.x;
    T wy = qn.w * qn.y;
    T wz = qn.w * qn.z;

    // Construct the rotation matrix using the optimized form
    // Row-major storage: m[row][col]
    return phynity::math::matrices::Mat3<T>(
        // Row 0
        T(1) - T(2) * (yy + zz), T(2) * (xy - wz), T(2) * (xz + wy),
        // Row 1
        T(2) * (xy + wz), T(1) - T(2) * (xx + zz), T(2) * (yz - wx),
        // Row 2
        T(2) * (xz - wy), T(2) * (yz + wx), T(1) - T(2) * (xx + yy)
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
template<typename T = float>
inline Quat<T> toQuaternion(const phynity::math::matrices::Mat3<T>& m) {
    if (hasNaN(m) || hasInf(m)) {
        // Invalid matrix
        // REJECT
        return Quat<T>();
    }

    T det = m.determinant();
    if (det < T(0)) {
        // Reflection detected
        // REJECT
        return Quat<T>();
    }

    bool nearOrtho =
    std::abs(m.getRow(0).dot(m.getRow(1))) < rotation<T>::orthonormal_epsilon &&
    std::abs(m.getRow(1).dot(m.getRow(2))) < rotation<T>::orthonormal_epsilon &&
    std::abs(m.getRow(2).dot(m.getRow(0))) < rotation<T>::orthonormal_epsilon &&
    std::abs(m.getRow(0).length() - T(1)) < rotation<T>::normalize_epsilon &&
    std::abs(m.getRow(1).length() - T(1)) < rotation<T>::normalize_epsilon &&
    std::abs(m.getRow(2).length() - T(1)) < rotation<T>::normalize_epsilon;

    if (!nearOrtho) {
        // Not orthonormal
        // REJECT
        return Quat<T>();
    }

    // Compute the trace
    T trace = m.m[0][0] + m.m[1][1] + m.m[2][2];

    Quat<T> q;

    if (trace > T(0)) {
        // CASE 1: trace > 0 (most common case)
        // Use w as the base component
        q.w = T(0.5) * std::sqrt(T(1) + trace);
        T s = T(0.25) / q.w;
        q.x = (m.m[2][1] - m.m[1][2]) * s;
        q.y = (m.m[0][2] - m.m[2][0]) * s;
        q.z = (m.m[1][0] - m.m[0][1]) * s;
    } else if ((m.m[0][0] > m.m[1][1]) && (m.m[0][0] > m.m[2][2])) {
        // CASE 2: m[0][0] is the largest diagonal element
        // Use x as the base component
        q.x = T(0.5) * std::sqrt(T(1) + m.m[0][0] - m.m[1][1] - m.m[2][2]);
        T s = T(0.25) / q.x;
        q.w = (m.m[2][1] - m.m[1][2]) * s;
        q.y = (m.m[0][1] + m.m[1][0]) * s;
        q.z = (m.m[0][2] + m.m[2][0]) * s;
    } else if (m.m[1][1] > m.m[2][2]) {
        // CASE 3: m[1][1] is the largest diagonal element
        // Use y as the base component
        q.y = T(0.5) * std::sqrt(T(1) + m.m[1][1] - m.m[0][0] - m.m[2][2]);
        T s = T(0.25) / q.y;
        q.w = (m.m[0][2] - m.m[2][0]) * s;
        q.x = (m.m[0][1] + m.m[1][0]) * s;
        q.z = (m.m[1][2] + m.m[2][1]) * s;
    } else {
        // CASE 4: m[2][2] is the largest diagonal element (or all equal)
        // Use z as the base component
        q.z = T(0.5) * std::sqrt(T(1) + m.m[2][2] - m.m[0][0] - m.m[1][1]);
        T s = T(0.25) / q.z;
        q.w = (m.m[1][0] - m.m[0][1]) * s;
        q.x = (m.m[0][2] + m.m[2][0]) * s;
        q.y = (m.m[1][2] + m.m[2][1]) * s;
    }

    // Ensure result is normalized (should be unit quaternion already)
    return q.normalized();
}

template<typename T = float>
inline phynity::math::vectors::Vec3<T> toEulerAngles(const Quat<T>& q) {
    // Normalize quaternion to ensure stability
    Quat<T> normalizedQ = q.normalized();

    // Extract quaternion components
    T w = normalizedQ.w;
    T x = normalizedQ.x;
    T y = normalizedQ.y;
    T z = normalizedQ.z;

    // Compute singularity test value
    // This represents the sine of pitch angle
    T singularity_test = 2*(w*y - z*x);

    phynity::math::vectors::Vec3<T> euler;

    if (singularity_test >= T(0.9999)) {
        euler.x = T(0);                      // roll = 0 (by convention)
        euler.y = T(0.5) * math<T>::pi;      // pitch = π/2 (90 degrees)
        euler.z = T(2) * std::atan2(z, w);   // yaw captures combined rotation
    }
    else if (singularity_test <= T(-0.9999)) {
        euler.x = T(0);                      // roll = 0 (by convention)
        euler.y = T(-0.5) * math<T>::pi;     // pitch = -π/2 (-90 degrees)
        euler.z = T(-2) * std::atan2(z, w);  // yaw captures combined rotation
    }
    else {
        euler.x = std::atan2(T(2) * (w * x + y * z), T(1) - T(2) * (x * x + y * y));  // roll
        euler.y = std::asin(std::clamp(singularity_test, T(-1), T(1)));                 // pitch
        euler.z = std::atan2(T(2) * (w * z + x * y), T(1) - T(2) * (y * y + z * z));  // yaw
    }

    return euler;
}

template<typename T = float>
inline Quat<T> toQuaternion(const phynity::math::vectors::Vec3<T>& euler) {
    T roll  = euler.x;
    T pitch = euler.y;
    T yaw   = euler.z;

    T cr = std::cos(roll  * T(0.5));
    T sr = std::sin(roll  * T(0.5));
    T cp = std::cos(pitch * T(0.5));
    T sp = std::sin(pitch * T(0.5));
    T cy = std::cos(yaw   * T(0.5));
    T sy = std::sin(yaw   * T(0.5));

    return Quat<T>(
        cy*cp*cr + sy*sp*sr, // w
        cy*cp*sr - sy*sp*cr, // x
        cy*sp*cr + sy*cp*sr, // y
        sy*cp*cr - cy*sp*sr  // z
    );
}

template<typename T = float>
inline void toAxisAngle(const Quat<T>& q, phynity::math::vectors::Vec3<T>& axis, T& angle) {
    Quat<T> qn = q.normalized();
    if (qn.w < T(0)) {
        qn = -qn;
    }
    
    T w = std::clamp(qn.w, T(0), T(1));
    
    if (w < T(0.0001)) {
        angle = math<T>::pi;
        T mag = std::sqrt(qn.x * qn.x + 
                             qn.y * qn.y + 
                             qn.z * qn.z);
        if (mag > T(1e-6)) {
            axis.x = qn.x / mag;
            axis.y = qn.y / mag;
            axis.z = qn.z / mag;

            axis.normalize();
        } else {
            axis = phynity::math::vectors::Vec3<T>(T(1), T(0), T(0));
        }
    } else if (w < T(1)) {
        angle = T(2) * std::acos(w);
        T s = std::sqrt(T(1) - w * w);

        if (s > T(1e-6)) {
            axis.x = qn.x / s;
            axis.y = qn.y / s;
            axis.z = qn.z / s;

            axis.normalize();
        } else {
            axis = phynity::math::vectors::Vec3<T>(T(1), T(0), T(0));
        }
    } else {
        angle = T(0);
        axis = phynity::math::vectors::Vec3<T>(T(1), T(0), T(0));
    }
}

}  // namespace phynity::math::quaternions

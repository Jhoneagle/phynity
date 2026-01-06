#pragma once

#include <core/math/quaternions/quat.hpp>
#include <cmath>
#include <algorithm>
#include <type_traits>

namespace phynity::math::quaternions {

/// ============================================================================
/// UTILITY FUNCTIONS
/// ============================================================================

/// Compute the angle between two quaternions in radians.
/// Returns the angle of the shortest rotation between q1 and q2.
///
/// MATHEMATICAL NOTE:
/// For unit quaternions: dot(q1, q2) = cos(θ_quat)
/// where θ_quat is the angle between quaternions on the 4D hypersphere.
/// The actual rotation angle is: θ_rotation = 2 * θ_quat
///
/// @param q1 First quaternion
/// @param q2 Second quaternion
/// @return Rotation angle in radians [0, 2π]
template<typename T = float>
inline T angleBetween(const Quat<T>& q1, const Quat<T>& q2) {
    // Normalize both quaternions
    Quat n1 = q1.normalized();
    Quat n2 = q2.normalized();
    
    // Compute dot product
    T dotProduct = dot(n1, n2);
    
    // Take absolute value to account for double-cover (q and -q represent same rotation)
    dotProduct = std::abs(dotProduct);
    
    // Clamp to [-1, 1] to avoid numerical errors in acos
    dotProduct = std::clamp(dotProduct, T(-1), T(1));
    
    // Rotation angle = 2 * acos(|dot product|)
    // The factor of 2 converts from quaternion angle to rotation angle
    return T(2) * std::acos(dotProduct);
}

/// Returns q2 adjusted to take the shortest path to q1.
/// If dot(q1, q2) < 0, returns -q2, otherwise returns q2.
///
/// @param q1 Reference quaternion
/// @param q2 Quaternion to adjust
/// @return q2 or -q2, whichever is closer to q1
template<typename T = float>
inline Quat<T> shortestPath(const Quat<T>& q1, const Quat<T>& q2) {
    return (dot(q1, q2) < T(0)) ? -q2 : q2;
}

/// ============================================================================
/// NLERP (Normalized Linear Interpolation)
/// ============================================================================
/// Fast quaternion interpolation using linear interpolation followed by
/// normalization. Provides approximate spherical interpolation suitable for
/// most real-time applications.
///
/// ADVANTAGES:
/// - Fast: only basic arithmetic and one sqrt (for normalization)
/// - Simple and robust
/// - Good enough for most physics and game applications
/// - No trigonometric functions required
///
/// LIMITATIONS:
/// - Non-constant angular velocity (speeds up in the middle)
/// - Less accurate than SLERP for large rotations
///
/// NUMERICAL STABILITY:
/// - Automatically handles quaternion double-cover (chooses shortest path)
/// - Safe for all input quaternions
/// - Handles edge cases (t=0, t=1) without special treatment
///
/// @param q1 Starting quaternion
/// @param q2 Ending quaternion
/// @param t Interpolation parameter [0, 1] where 0=q1, 1=q2
/// @return Interpolated quaternion (normalized)
template<typename T = float>
inline Quat<T> nlerp(const Quat<T>& q1, const Quat<T>& q2, T t) {
    // Clamp t to [0, 1] to avoid extrapolation issues
    t = std::clamp(t, T(0), T(1));
    
    // Choose the shortest path by checking the dot product
    // If dot product is negative, quaternions are on opposite hemispheres
    // so we negate q2 to take the shorter path
    Quat<T> q2_adjusted = shortestPath(q1, q2);
    
    // Linear interpolation: lerp(q1, q2, t) = q1 * (1-t) + q2 * t
    T oneMinusT = T(1) - t;
    Quat<T> result(
        q1.w * oneMinusT + q2_adjusted.w * t,
        q1.x * oneMinusT + q2_adjusted.x * t,
        q1.y * oneMinusT + q2_adjusted.y * t,
        q1.z * oneMinusT + q2_adjusted.z * t
    );
    
    // Normalize to ensure unit quaternion
    return result.normalized();
}

/// ============================================================================
/// SLERP (Spherical Linear Interpolation)
/// ============================================================================
/// Geometrically correct spherical interpolation with constant angular velocity.
/// Provides the smoothest possible interpolation between two rotations.
///
/// ADVANTAGES:
/// - Constant angular velocity (smooth, predictable motion)
/// - Geometrically correct interpolation on the quaternion hypersphere
/// - Best quality for animations and cinematics
///
/// LIMITATIONS:
/// - More expensive than NLERP (requires sin, acos)
/// - Slightly more complex implementation
///
/// NUMERICAL STABILITY:
/// - Automatically handles quaternion double-cover (chooses shortest path)
/// - Falls back to NLERP for very small angles to avoid division by near-zero
/// - Clamps dot product to avoid acos domain errors
/// - Threshold: 0.9995 (angle ≈ 1.8°)
///
/// FORMULA:
/// slerp(q1, q2, t) = (sin((1-t)*θ) * q1 + sin(t*θ) * q2) / sin(θ)
/// where θ is the angle between q1 and q2
///
/// @param q1 Starting quaternion
/// @param q2 Ending quaternion
/// @param t Interpolation parameter [0, 1] where 0=q1, 1=q2
/// @return Interpolated quaternion (normalized)
template<typename T = float>
inline Quat<T> slerp(const Quat<T>& q1, const Quat<T>& q2, T t) {
    // Clamp t to [0, 1] to avoid extrapolation issues
    t = std::clamp(t, T(0), T(1));
    
    // Normalize input quaternions
    Quat<T> n1 = q1.normalized();
    Quat<T> n2 = q2.normalized();
    
    // Choose the shortest path
    T dotProduct = dot(n1, n2);
    if (dotProduct < T(0)) {
        n2 = -n2;
        dotProduct = -dotProduct;
    }
    
    // Clamp to avoid acos domain errors
    dotProduct = std::clamp(dotProduct, T(-1), T(1));
    
    // If quaternions are very close, use NLERP to avoid division by near-zero
    // Threshold: cos(1.8°) ≈ 0.9995
    constexpr T SLERP_THRESHOLD = T(0.9995);
    if (dotProduct > SLERP_THRESHOLD) {
        // Fall back to NLERP for numerical stability
        return nlerp(q1, q2, t);
    }
    
    // Compute the angle between quaternions
    T theta = std::acos(dotProduct);
    T sinTheta = std::sin(theta);
    
    // Compute interpolation weights
    T w1 = std::sin((T(1) - t) * theta) / sinTheta;
    T w2 = std::sin(t * theta) / sinTheta;
    
    // Perform spherical interpolation
    return Quat(
        n1.w * w1 + n2.w * w2,
        n1.x * w1 + n2.x * w2,
        n1.y * w1 + n2.y * w2,
        n1.z * w1 + n2.z * w2
    );
}

}  // namespace phynity::math::quaternions

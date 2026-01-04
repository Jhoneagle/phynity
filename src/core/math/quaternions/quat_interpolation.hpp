#pragma once

#include <core/math/quaternions/quat.hpp>
#include <cmath>
#include <algorithm>

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
inline float angleBetween(const Quat& q1, const Quat& q2) {
    // Normalize both quaternions
    Quat n1 = q1.normalized();
    Quat n2 = q2.normalized();
    
    // Compute dot product
    float dotProduct = dot(n1, n2);
    
    // Take absolute value to account for double-cover (q and -q represent same rotation)
    dotProduct = std::abs(dotProduct);
    
    // Clamp to [-1, 1] to avoid numerical errors in acos
    dotProduct = std::clamp(dotProduct, -1.0f, 1.0f);
    
    // Rotation angle = 2 * acos(|dot product|)
    // The factor of 2 converts from quaternion angle to rotation angle
    return 2.0f * std::acos(dotProduct);
}

/// Returns q2 adjusted to take the shortest path to q1.
/// If dot(q1, q2) < 0, returns -q2, otherwise returns q2.
///
/// @param q1 Reference quaternion
/// @param q2 Quaternion to adjust
/// @return q2 or -q2, whichever is closer to q1
inline Quat shortestPath(const Quat& q1, const Quat& q2) {
    return (dot(q1, q2) < 0.0f) ? -q2 : q2;
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
inline Quat nlerp(const Quat& q1, const Quat& q2, float t) {
    // Clamp t to [0, 1] to avoid extrapolation issues
    t = std::clamp(t, 0.0f, 1.0f);
    
    // Choose the shortest path by checking the dot product
    // If dot product is negative, quaternions are on opposite hemispheres
    // so we negate q2 to take the shorter path
    Quat q2_adjusted = shortestPath(q1, q2);
    
    // Linear interpolation: lerp(q1, q2, t) = q1 * (1-t) + q2 * t
    float oneMinusT = 1.0f - t;
    Quat result(
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
inline Quat slerp(const Quat& q1, const Quat& q2, float t) {
    // Clamp t to [0, 1] to avoid extrapolation issues
    t = std::clamp(t, 0.0f, 1.0f);
    
    // Normalize input quaternions
    Quat n1 = q1.normalized();
    Quat n2 = q2.normalized();
    
    // Choose the shortest path
    float dotProduct = dot(n1, n2);
    if (dotProduct < 0.0f) {
        n2 = -n2;
        dotProduct = -dotProduct;
    }
    
    // Clamp to avoid acos domain errors
    dotProduct = std::clamp(dotProduct, -1.0f, 1.0f);
    
    // If quaternions are very close, use NLERP to avoid division by near-zero
    // Threshold: cos(1.8°) ≈ 0.9995
    constexpr float SLERP_THRESHOLD = 0.9995f;
    if (dotProduct > SLERP_THRESHOLD) {
        // Fall back to NLERP for numerical stability
        return nlerp(q1, q2, t);
    }
    
    // Compute the angle between quaternions
    float theta = std::acos(dotProduct);
    float sinTheta = std::sin(theta);
    
    // Compute interpolation weights
    float w1 = std::sin((1.0f - t) * theta) / sinTheta;
    float w2 = std::sin(t * theta) / sinTheta;
    
    // Perform spherical interpolation
    return Quat(
        n1.w * w1 + n2.w * w2,
        n1.x * w1 + n2.x * w2,
        n1.y * w1 + n2.y * w2,
        n1.z * w1 + n2.z * w2
    );
}

}  // namespace phynity::math::quaternions

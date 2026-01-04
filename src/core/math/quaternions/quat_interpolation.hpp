#pragma once

#include <core/math/quaternions/quat.hpp>
#include <cmath>

namespace phynity::math::quaternions {

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
    float dotProduct = dot(q1, q2);
    
    Quat q2_adjusted = (dotProduct < 0.0f) ? -q2 : q2;
    
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

}  // namespace phynity::math::quaternions

#pragma once

#include <core/math/vectors/vec3.hpp>
#include <cmath>
#include <ostream>
#include <algorithm>
#include <type_traits>

using phynity::math::vectors::Vec3;

namespace phynity::math::quaternions {

/// Quaternion for 3D rotations without gimbal lock with dual-precision support.
/// Represents rotation as q = w + xi + yj + zk where (x, y, z) is the vector part
/// and w is the scalar part.
/// For a rotation of angle θ around axis n̂: q = (cos(θ/2), sin(θ/2) * n̂)
template<typename T = float>
struct Quat {
    static_assert(std::is_floating_point_v<T>, "Quat template parameter must be a floating-point type");
    T w, x, y, z;

    /// Default constructor: identity quaternion (no rotation)
    Quat() : w(T(1)), x(T(0)), y(T(0)), z(T(0)) {}

    /// Component constructor
    explicit Quat(T w, T x, T y, T z)
        : w(w), x(x), y(y), z(z) {}

    /// Axis-angle constructor: creates rotation of angle θ around axis n̂
    /// @param axis Vector representing rotation axis
    /// @param angle Rotation angle in radians
    Quat(const Vec3<T>& axis, T angle) {
        Vec3<T> normAxis = axis.normalized();
        T halfAngle = angle * T(0.5);
        T sinHalf = std::sin(halfAngle);

        w = std::cos(halfAngle);
        x = normAxis.x * sinHalf;
        y = normAxis.y * sinHalf;
        z = normAxis.z * sinHalf;
    }

    // ========================================================================
    // Arithmetic Operators
    // ========================================================================

    Quat operator+(const Quat& q) const {
        return Quat(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    Quat operator-(const Quat& q) const {
        return Quat(w - q.w, x - q.x, y - q.y, z - q.z);
    }

    /// Quaternion multiplication (composition of rotations)
    /// Order matters: this * q applies q's rotation first, then this rotation
    /// q1 * q2 = (w1*w2 - v1·v2, w1*v2 + w2*v1 + v1×v2)
    Quat operator*(const Quat& q) const {
        return Quat(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    /// Scalar multiplication
    Quat operator*(T scalar) const {
        return Quat(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    /// Scalar division
    Quat operator/(T scalar) const {
        T inv = T(1) / scalar;
        return Quat(w * inv, x * inv, y * inv, z * inv);
    }

    /// Negation
    Quat operator-() const {
        return Quat(-w, -x, -y, -z);
    }

    // ========================================================================
    // Compound Assignment Operators
    // ========================================================================

    Quat& operator+=(const Quat& q) {
        w += q.w;
        x += q.x;
        y += q.y;
        z += q.z;
        return *this;
    }

    Quat& operator-=(const Quat& q) {
        w -= q.w;
        x -= q.x;
        y -= q.y;
        z -= q.z;
        return *this;
    }

    Quat& operator*=(const Quat& q) {
        *this = *this * q;
        return *this;
    }

    Quat& operator*=(T scalar) {
        w *= scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Quat& operator/=(T scalar) {
        T inv = T(1) / scalar;
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }

    // ========================================================================
    // Comparison Operators
    // ========================================================================

    bool operator==(const Quat& q) const {
        return w == q.w && x == q.x && y == q.y && z == q.z;
    }

    bool operator!=(const Quat& q) const {
        return !(*this == q);
    }

    // ========================================================================
    // Magnitude and Normalization
    // ========================================================================

    /// Squared magnitude: ||q||² = w² + x² + y² + z²
    T squaredLength() const {
        return w * w + x * x + y * y + z * z;
    }

    /// Magnitude (norm): ||q|| = √(w² + x² + y² + z²)
    T length() const {
        return std::sqrt(squaredLength());
    }

    /// Alias for length()
    T magnitude() const {
        return length();
    }

    /// Alias for squaredLength()
    T magnitudeSquared() const {
        return squaredLength();
    }

    /// Alias for length() - compatibility
    T norm() const {
        return length();
    }

    /// Returns normalized quaternion without modifying this
    Quat normalized() const {
        T mag = length();
        if (mag < T(1e-6)) {
            return Quat(T(1), T(0), T(0), T(0));  // Return identity if near-zero
        }
        return *this / mag;
    }

    /// Normalize this quaternion in-place
    Quat& normalize() {
        T mag = length();
        if (mag < T(1e-6)) {
            w = T(1);
            x = y = z = T(0);
            return *this;
        }
        *this /= mag;
        return *this;
    }

    // ========================================================================
    // Conjugate and Inverse
    // ========================================================================

    /// Conjugate: q* = (w, -x, -y, -z)
    /// Reverses the rotation direction
    Quat conjugate() const {
        return Quat(w, -x, -y, -z);
    }

    /// Inverse: q⁻¹ = q* / ||q||²
    /// For unit quaternions: q⁻¹ = q*
    /// Undoes the rotation
    Quat inverse() const {
        T magSq = squaredLength();
        if (magSq < T(1e-12)) {
            return Quat(T(1), T(0), T(0), T(0));  // Return identity if degenerate
        }
        return conjugate() / magSq;
    }

    // ========================================================================
    // Vector Rotation
    // ========================================================================

    /// Rotate a 3D vector by this quaternion: v' = q * v * q⁻¹
    /// Assumes this quaternion is normalized (or at least a valid rotation)
    /// @param v Vector to rotate
    /// @return Rotated vector
    Vec3<T> rotateVector(const Vec3<T>& v) const {
        // Convert vector to quaternion form: v_quat = (0, v.x, v.y, v.z)
        Quat vQuat(T(0), v.x, v.y, v.z);
        
        // Apply rotation: result = q * v_quat * q⁻¹
        Quat rotated = *this * vQuat * inverse();
        
        return Vec3<T>(rotated.x, rotated.y, rotated.z);
    }

    /// Unrotate (inverse rotate) a 3D vector: v' = q⁻¹ * v * q
    /// This reverses the rotation applied by rotateVector()
    /// @param v Vector to unrotate
    /// @return Unrotated vector
    Vec3<T> unrotateVector(const Vec3<T>& v) const {
        // Convert vector to quaternion form: v_quat = (0, v.x, v.y, v.z)
        Quat vQuat(T(0), v.x, v.y, v.z);
        
        // Apply inverse rotation: result = q⁻¹ * v_quat * q
        Quat inv = inverse();
        Quat rotated = inv * vQuat * *this;
        
        return Vec3<T>(rotated.x, rotated.y, rotated.z);
    }
};

// ============================================================================
// Free Function Operators
// ============================================================================

/// Scalar * Quaternion
template<typename T = float>
inline Quat<T> operator*(T scalar, const Quat<T>& q) {
    return q * scalar;
}

/// Dot product: q1 · q2 = w1*w2 + x1*x2 + y1*y2 + z1*z2
template<typename T = float>
inline T dot(const Quat<T>& q1, const Quat<T>& q2) {
    return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}

/// Stream output operator
template<typename T = float>
inline std::ostream& operator<<(std::ostream& os, const Quat<T>& q) {
    os << "Quat(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
    return os;
}

// Type aliases
using Quatf = Quat<float>;
using Quatd = Quat<double>;

}  // namespace phynity::math::quaternions

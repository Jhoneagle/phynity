#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/math/matrices/mat3.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_conversions.hpp>
#include <cmath>

namespace phynity::physics::inertia {

using phynity::math::vectors::Vec3f;
using phynity::math::matrices::Mat3f;
using phynity::math::quaternions::Quatf;

// ============================================================================
// ANALYTICAL INERTIA TENSOR COMPUTATIONS
// ============================================================================

/// Compute inertia tensor for a solid sphere
/// Formula: I = (2/5) * m * r²
/// @param mass Mass in kg
/// @param radius Sphere radius
/// @return 3x3 diagonal inertia tensor (body-space)
inline Mat3f compute_sphere_inertia(float mass, float radius) {
    float val = (2.0f / 5.0f) * mass * radius * radius;
    Mat3f I(0.0f);
    I(0, 0) = val;
    I(1, 1) = val;
    I(2, 2) = val;
    return I;
}

/// Compute inertia tensor for a solid box (cuboid)
/// Formula:
///   I_xx = (1/12) * m * (b² + c²)
///   I_yy = (1/12) * m * (a² + c²)
///   I_zz = (1/12) * m * (a² + b²)
/// where a, b, c are full dimensions (2 * half_extents)
/// @param mass Mass in kg
/// @param half_extents Half-size in each dimension [hx, hy, hz]
/// @return 3x3 diagonal inertia tensor (body-space)
inline Mat3f compute_box_inertia(float mass, const Vec3f& half_extents) {
    // Full dimensions
    float a2 = 4.0f * half_extents.x * half_extents.x;  // width² = (2*hx)²
    float b2 = 4.0f * half_extents.y * half_extents.y;  // height²
    float c2 = 4.0f * half_extents.z * half_extents.z;  // depth²
    
    float coeff = mass / 12.0f;
    
    Mat3f I(0.0f);
    I(0, 0) = coeff * (b2 + c2);  // I_xx
    I(1, 1) = coeff * (a2 + c2);  // I_yy
    I(2, 2) = coeff * (a2 + b2);  // I_zz
    
    return I;
}

/// Compute inertia tensor for a solid capsule (cylinder + hemispheres)
/// Simplified model: treats as cylinder + contribution from hemispheres.
/// Formula is approximated; for exact results, use numerical integration.
/// @param mass Mass in kg
/// @param radius Capsule radius
/// @param half_height Half-length of cylinder
/// @return 3x3 diagonal inertia tensor (body-space)
inline Mat3f compute_capsule_inertia(float mass, float radius, float half_height) {
    float r2 = radius * radius;
    float h2 = half_height * half_height;
    
    // Approximation: cylindrical approximation
    float coeff_xy = mass * (3.0f * r2 / 4.0f + h2 / 12.0f);
    float coeff_z = mass * r2 / 2.0f;
    
    Mat3f I(0.0f);
    I(0, 0) = coeff_xy;
    I(1, 1) = coeff_xy;
    I(2, 2) = coeff_z;
    
    return I;
}

// ============================================================================
// INERTIA TENSOR TRANSFORMATIONS
// ============================================================================

/// Rotate inertia tensor from body-space to world-space
/// Formula: I_world = R * I_body * R^T
/// where R is the rotation matrix from quaternion
/// @param I_body Inertia tensor in body-space
/// @param q Quaternion representing world orientation
/// @return Inertia tensor rotated to world-space
inline Mat3f rotate_inertia_tensor(const Mat3f& I_body, const Quatf& q) {
    Mat3f R = phynity::math::quaternions::toRotationMatrix(q);
    Mat3f R_transpose = R.transposed();
    
    // I_world = R * I_body * R^T
    Mat3f temp = R * I_body;
    return temp * R_transpose;
}

/// Compute inverse of inertia tensor with numerical stability
/// Returns zero matrix if tensor is singular or near-singular
/// @param I Inertia tensor
/// @return Inverse inertia tensor (I^-1)
inline Mat3f invert_inertia_tensor(const Mat3f& I) {
    // Only invert diagonal elements (inertia tensors are diagonal in principal axes)
    // Off-diagonal elements should be zero
    Mat3f I_inv(0.0f);
    
    float tol = 1e-6f;
    if (std::abs(I(0, 0)) > tol) {
        I_inv(0, 0) = 1.0f / I(0, 0);
    }
    if (std::abs(I(1, 1)) > tol) {
        I_inv(1, 1) = 1.0f / I(1, 1);
    }
    if (std::abs(I(2, 2)) > tol) {
        I_inv(2, 2) = 1.0f / I(2, 2);
    }
    
    return I_inv;
}

// ============================================================================
// ANGULAR MOMENTUM AND KINETIC ENERGY
// ============================================================================

/// Compute angular momentum: L = I * ω
/// @param I Inertia tensor
/// @param omega Angular velocity
/// @return Angular momentum vector
inline Vec3f compute_angular_momentum(const Mat3f& I, const Vec3f& omega) {
    return I * omega;
}

/// Compute angular kinetic energy: KE = 0.5 * ω^T * I * ω
/// @param I Inertia tensor
/// @param omega Angular velocity
/// @return Kinetic energy due to rotation
inline float compute_angular_kinetic_energy(const Mat3f& I, const Vec3f& omega) {
    Vec3f I_omega = I * omega;
    return 0.5f * omega.dot(I_omega);
}

// ============================================================================
// QUATERNION INTEGRATION FOR ANGULAR VELOCITY
// ============================================================================

/// Integrate angular velocity into quaternion (time-step update)
/// Formula: q_new = q_old + 0.5 * dt * ω * q_old
/// where ω is treated as (0, ωx, ωy, ωz) quaternion
/// @param q Current orientation quaternion
/// @param omega Angular velocity (rad/s)
/// @param dt Time step
/// @return Updated quaternion (not normalized)
inline Quatf integrate_quaternion(const Quatf& q, const Vec3f& omega, float dt) {
    // Create quaternion from angular velocity: q_omega = (0, ωx, ωy, ωz)
    Quatf q_omega(0.0f, omega.x, omega.y, omega.z);
    
    // Update: q_new = q + 0.5 * dt * q_omega * q
    Quatf q_omega_scaled = q_omega * (0.5f * dt);
    Quatf dq = q_omega_scaled * q;
    
    return q + dq;
}

}  // namespace phynity::physics::inertia

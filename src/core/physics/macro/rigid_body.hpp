#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/math/matrices/mat3.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_conversions.hpp>
#include <core/physics/common/material.hpp>
#include <core/physics/macro/shape.hpp>
#include <memory>

namespace phynity::physics {

using phynity::math::vectors::Vec3f;
using phynity::math::matrices::Mat3f;
using phynity::math::quaternions::Quatf;

/// Represents a rigid body in the simulation (macro-scale).
/// Extends beyond particles with rotational dynamics, orientation, and inertia.
/// Rigid bodies are used for larger-scale objects that can rotate and interact via constraints.
class RigidBody {
public:
    // ========================================================================
    // Linear Motion State
    // ========================================================================
    
    Vec3f position = Vec3f(0.0f);          ///< World-space position
    Vec3f velocity = Vec3f(0.0f);          ///< World-space linear velocity
    Vec3f force_accumulator = Vec3f(0.0f); ///< Accumulated forces this frame
    float inv_mass = 1.0f;                 ///< Inverse of mass (1/m) for numerical stability

    // ========================================================================
    // Angular Motion State
    // ========================================================================
    
    Quatf orientation = Quatf();           ///< World-space rotation as quaternion
    Vec3f angular_velocity = Vec3f(0.0f);   ///< World-space angular velocity (rad/s)
    Vec3f torque_accumulator = Vec3f(0.0f); ///< Accumulated torques this frame
    Mat3f inertia_tensor = Mat3f(0.0f);    ///< Body-space inertia tensor (I)
    Mat3f inertia_tensor_inv = Mat3f(0.0f); ///< Precomputed inverse (I^-1)

    // ========================================================================
    // Shape and Collision
    // ========================================================================
    
    std::shared_ptr<Shape> shape = nullptr; ///< Collision shape (sphere, box, capsule)
    float collision_radius = 0.5f;          ///< Broadphase collision radius (for AABB)

    // ========================================================================
    // Material Properties and Lifecycle
    // ========================================================================
    
    Material material{};                   ///< Material properties (restitution, friction, damping)
    bool active = true;                    ///< Active flag for pooling/recycling
    int id = -1;                           ///< Unique identifier for constraint linking
    float lifetime = -1.0f;                ///< Remaining lifetime (< 0 = infinite, > 0 = finite)

    // ========================================================================
    // Constructors
    // ========================================================================

    /// Default constructor - creates rigid body at origin with identity orientation
    RigidBody() = default;

    /// Full constructor with position, orientation, and shape
    RigidBody(
        const Vec3f& pos,
        const Quatf& orient,
        std::shared_ptr<Shape> shape_ptr,
        const Material& mat = Material{},
        float mass = 1.0f
    )
        : position(pos),
          orientation(orient),
          shape(shape_ptr),
          material(mat),
          id(-1),
          lifetime(-1.0f)
    {
        set_mass(mass);
    }

    // Move semantics
    RigidBody(RigidBody&& other) noexcept = default;
    RigidBody& operator=(RigidBody&& other) noexcept = default;

    // ========================================================================
    // Mass and Inertia Management
    // ========================================================================

    /// Set the mass of the body and recompute inertia tensor
    /// @param mass The mass in kilograms (must be > 0)
    void set_mass(float mass) {
        if (mass <= 0.0f) {
            // Static body (infinite mass)
            inv_mass = 0.0f;
        } else {
            inv_mass = 1.0f / mass;
        }
        material.mass = mass;
        
        // Recompute inertia tensor based on shape and mass
        if (shape) {
            shape->compute_inertia_tensors(mass, inertia_tensor, inertia_tensor_inv);
        }
    }

    /// Get the mass of the body
    float get_mass() const {
        return (inv_mass > 0.0f) ? 1.0f / inv_mass : 0.0f;
    }

    /// Check if body is static (infinite mass)
    bool is_static() const {
        return inv_mass == 0.0f;
    }

    // ========================================================================
    // State Getters
    // ========================================================================

    /// Get rotation matrix from quaternion (for collision/visualization)
    Mat3f get_rotation_matrix() const {
        return phynity::math::quaternions::toRotationMatrix(orientation);
    }

    /// Get kinetic energy (linear + angular)
    float kinetic_energy() const {
        float linear_ke = 0.5f * get_mass() * velocity.squaredLength();
        
        // Angular KE = 0.5 * ω^T * I * ω
        Vec3f I_omega = inertia_tensor * angular_velocity;
        float angular_ke = 0.5f * angular_velocity.dot(I_omega);
        
        return linear_ke + angular_ke;
    }

    /// Get linear momentum
    Vec3f momentum() const {
        return velocity * get_mass();
    }

    /// Get angular momentum (L = I * ω)
    Vec3f angular_momentum() const {
        return inertia_tensor * angular_velocity;
    }

    // ========================================================================
    // Resetting (for frame-to-frame simulation)
    // ========================================================================

    /// Clear accumulated forces and torques
    void clear_forces_and_torques() {
        force_accumulator = Vec3f(0.0f);
        torque_accumulator = Vec3f(0.0f);
    }

    /// Update lifetime and return whether body is still alive
    bool update_lifetime(float dt) {
        if (lifetime < 0.0f) {
            return true;  // Infinite lifetime
        }
        lifetime -= dt;
        active = lifetime > 0.0f;
        return active;
    }
};

// Type alias for convenience
using RigidBodyID = int;

}  // namespace phynity::physics

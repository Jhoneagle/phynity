#pragma once

#include <core/physics/constraints/constraint.hpp>
#include <core/physics/rigid_body.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_conversions.hpp>
#include <cmath>
#include <vector>

namespace phynity::physics::constraints {

using phynity::math::vectors::Vec3f;
using phynity::math::quaternions::Quatf;
using phynity::math::matrices::Mat3f;
using phynity::math::matrices::MatDynamic;

/// Fixed constraint for rigid bodies (6-DOF lock).
/// Locks both position and orientation between two rigid bodies (or one body to world).
/// Can be used to create welded joints or attach bodies to fixed points.
class FixedConstraintRB : public Constraint {
public:
    FixedConstraintRB(
        physics::RigidBody* body_a,
        physics::RigidBody* body_b,
        const Vec3f& anchor_a_local,  ///< Anchor in body A's local frame
        const Vec3f& anchor_b_local   ///< Anchor in body B's local frame (or world if b is null)
    )
        : body_a_(body_a),
          body_b_(body_b),
          anchor_a_local_(anchor_a_local),
          anchor_b_local_(anchor_b_local),
          accumulated_impulse_(0.0f)
    {
        if (body_a && body_b) {
            // Store initial relative orientation
            initial_relative_q_ = body_b->orientation.conjugate() * body_a->orientation;
        }
    }

    // ========================================================================
    // Constraint Interface Implementation
    // ========================================================================

    float compute_error() const override {
        if (!body_a_) return 0.0f;
        
        // Convert anchors from local to world space
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f anchor_a_world = body_a_->position + R_a * anchor_a_local_;
        
        Vec3f anchor_b_world = anchor_b_local_;
        if (body_b_) {
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            anchor_b_world = body_b_->position + R_b * anchor_b_local_;
        }
        
        // Position error (distance between anchors)
        Vec3f pos_error = anchor_b_world - anchor_a_world;
        float pos_error_mag = pos_error.length();
        
        // Rotation error (angle between orientations)
        Quatf relative_q = body_b_ ? body_b_->orientation * body_a_->orientation.conjugate()
                                    : body_a_->orientation;
        Quatf error_q = relative_q * initial_relative_q_.conjugate();
        
        // Axis-angle from quaternion: angle = 2 * acos(w)
        float rotation_error = 2.0f * std::acos(std::clamp(error_q.w, -1.0f, 1.0f));
        
        return std::max(pos_error_mag, rotation_error);
    }

    MatDynamic<float> compute_jacobian() const override {
        // 6 constraint equations: 3 position + 3 rotation
        // Columns: [v_a, ω_a, v_b, ω_b] (12 columns for 2 bodies)
        
        MatDynamic<float> J(6, 12, 0.0f);
        
        if (!body_a_) return J;
        
        // Convert anchor to world space
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f r_a = R_a * anchor_a_local_;
        
        // Position constraints (3 rows)
        // Jacobian for position: identity for linear, -r× for angular
        for (int i = 0; i < 3; ++i) {
            J(static_cast<size_t>(i), static_cast<size_t>(i)) = 1.0f;  // ∂/∂v_a
        }
        
        // Skew-symmetric matrix of r_a for cross product (-r_a ×)
        // [-r_a ×] = [  0  -rz   ry ]
        //            [ rz   0  -rx ]
        //            [-ry  rx   0  ]
        J(0, 5) =  r_a.z;  // ∂(x)/∂ω_a.y = rz
        J(0, 4) = -r_a.y;  // ∂(x)/∂ω_a.z = -ry
        J(1, 3) = -r_a.z;  // ∂(y)/∂ω_a.x = -rz
        J(1, 5) =  r_a.x;  // ∂(y)/∂ω_a.z = rx
        J(2, 3) =  r_a.y;  // ∂(z)/∂ω_a.x = ry
        J(2, 4) = -r_a.x;  // ∂(z)/∂ω_a.y = -rx
        
        if (body_b_) {
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            Vec3f r_b = R_b * anchor_b_local_;
            
            // Negative coupling to body B
            for (int i = 0; i < 3; ++i) {
                J(static_cast<size_t>(i), static_cast<size_t>(6 + i)) = -1.0f;  // -I for v_b
            }
            
            // Skew-symmetric for body B
            J(0, 11) =  r_b.z;
            J(0, 10) = -r_b.y;
            J(1, 9)  = -r_b.z;
            J(1, 11) =  r_b.x;
            J(2, 9)  =  r_b.y;
            J(2, 10) = -r_b.x;
        }
        
        // Rotation constraints (3 rows) - simplified: relative orientation error
        // Use quaternion error for angular constraint
        for (int i = 0; i < 3; ++i) {
            J(static_cast<size_t>(3 + i), static_cast<size_t>(3 + i)) = 1.0f;  // ω_a affects relative angular velocity
            if (body_b_) {
                J(static_cast<size_t>(3 + i), static_cast<size_t>(9 + i)) = -1.0f;  // -ω_b
            }
        }
        
        return J;
    }

    void apply_impulse(float impulse_magnitude) override {
        if (!body_a_) return;
        
        // Get anchor in world space
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f r_a = R_a * anchor_a_local_;
        
        // For simplicity, apply impulse along constraint direction
        // In full implementation, would use separable impulses for each constraint row
        Vec3f impulse_dir = Vec3f(1, 0, 0);  // TODO: Use Jacobian column
        Vec3f impulse = impulse_dir * impulse_magnitude;
        
        // Apply to body A
        body_a_->velocity += impulse * body_a_->inv_mass;
        Vec3f torque_a = r_a.cross(impulse);
        body_a_->angular_velocity += body_a_->inertia_tensor_inv * torque_a;
        
        // Apply to body B (opposite)
        if (body_b_) {
            body_b_->velocity -= impulse * body_b_->inv_mass;
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            Vec3f r_b = R_b * anchor_b_local_;
            Vec3f torque_b = r_b.cross(-impulse);
            body_b_->angular_velocity += body_b_->inertia_tensor_inv * torque_b;
        }
        
        accumulated_impulse_ += impulse_magnitude;
    }

    std::vector<size_t> get_body_ids() const override {
        std::vector<size_t> ids;
        if (body_a_) ids.push_back(static_cast<size_t>(body_a_->id));
        if (body_b_) ids.push_back(static_cast<size_t>(body_b_->id));
        return ids;
    }

    float get_accumulated_impulse() const override {
        return accumulated_impulse_;
    }

    void set_warm_start_impulse(float impulse) override {
        accumulated_impulse_ = impulse;
    }

private:
    physics::RigidBody* body_a_;           ///< First body (constrained, not null)
    physics::RigidBody* body_b_;           ///< Second body (can be null for fixed-to-world)
    Vec3f anchor_a_local_;                 ///< Anchor point in body A's local frame
    Vec3f anchor_b_local_;                 ///< Anchor point in body B's local frame
    Quatf initial_relative_q_;             ///< Initial relative orientation
    float accumulated_impulse_;
};


/// Hinge constraint for rigid bodies (5-DOF rotation).
/// Allows rotation around a hinge axis while locking the pivot point and other rotations.
/// Useful for doors, hinges, wheels, etc.
class HingeConstraintRB : public Constraint {
public:
    HingeConstraintRB(
        physics::RigidBody* body_a,
        physics::RigidBody* body_b,
        const Vec3f& pivot_a_local,    ///< Pivot point in body A's local frame
        const Vec3f& pivot_b_local,    ///< Pivot point in body B's local frame
        const Vec3f& axis_local        ///< Hinge axis in local frame (will be normalized)
    )
        : body_a_(body_a),
          body_b_(body_b),
          pivot_a_local_(pivot_a_local),
          pivot_b_local_(pivot_b_local),
          axis_local_(axis_local.normalized()),
          accumulated_impulse_(0.0f)
    {
        if (body_a_) {
            // Store initial perpendicular directions
            Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
            axis_world_ = R_a * axis_local_;
        }
    }

    // ========================================================================
    // Constraint Interface Implementation
    // ========================================================================

    float compute_error() const override {
        if (!body_a_) return 0.0f;
        
        // Convert pivots to world space
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f pivot_a_world = body_a_->position + R_a * pivot_a_local_;
        
        Vec3f pivot_b_world = pivot_b_local_;
        if (body_b_) {
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            pivot_b_world = body_b_->position + R_b * pivot_b_local_;
        }
        
        // Position error: pivots must coincide
        Vec3f pos_error = pivot_b_world - pivot_a_world;
        float pos_error_mag = pos_error.length();
        
        // Rotation error: perpendicular directions must align
        Vec3f axis_a = (R_a * axis_local_).normalized();
        // Note: axis_b not used in MVP - TODO for full implementation
        (void)axis_a;  // Suppress unused variable warning
        if (body_b_) {
            // TODO: Get hinge axis for body B if different
        }
        
        // For MVP: just return position error
        return pos_error_mag;
    }

    MatDynamic<float> compute_jacobian() const override {
        // 5 constraint equations: 3 position + 2 rotation (perpendicular to hinge axis)
        MatDynamic<float> J(5, 12, 0.0f);
        
        if (!body_a_) return J;
        
        // Simplified: same as position constraints from FixedConstraint
        // but with extra rows for perpendicular rotation constraints
        // (Full implementation would use proper basis vectors perpendicular to hinge axis)
        
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f r_a = R_a * pivot_a_local_;
        
        // Position constraints (3 rows)
        for (int i = 0; i < 3; ++i) {
            J(static_cast<size_t>(i), static_cast<size_t>(i)) = 1.0f;
        }
        
        J(0, 5) =  r_a.z;
        J(0, 4) = -r_a.y;
        J(1, 3) = -r_a.z;
        J(1, 5) =  r_a.x;
        J(2, 3) =  r_a.y;
        J(2, 4) = -r_a.x;
        
        if (body_b_) {
            for (int i = 0; i < 3; ++i) {
                J(static_cast<size_t>(i), static_cast<size_t>(6 + i)) = -1.0f;
            }
            
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            Vec3f r_b = R_b * pivot_b_local_;
            
            J(0, 11) =  r_b.z;
            J(0, 10) = -r_b.y;
            J(1, 9)  = -r_b.z;
            J(1, 11) =  r_b.x;
            J(2, 9)  =  r_b.y;
            J(2, 10) = -r_b.x;
        }
        
        // Rotation constraints (2 rows)
        // Constrain rotation perpendicular to hinge axis
        // Use two basis vectors perpendicular to axis
        Vec3f axis_normalized = axis_world_.normalized();
        Vec3f perp1 = (std::abs(axis_normalized.x) < 0.9f) 
                        ? Vec3f(1, 0, 0).cross(axis_normalized)
                        : Vec3f(0, 1, 0).cross(axis_normalized);
        perp1 = perp1.normalized();
        Vec3f perp2 = axis_normalized.cross(perp1);
        
        // Constraint: relative angular velocity along perp1 = 0
        // ω_a · perp1 - ω_b · perp1 = 0
        for (int i = 0; i < 3; ++i) {
            J(3, static_cast<size_t>(3 + i)) = perp1[i];
            if (body_b_) J(3, static_cast<size_t>(9 + i)) = -perp1[i];
            
            J(4, static_cast<size_t>(3 + i)) = perp2[i];
            if (body_b_) J(4, static_cast<size_t>(9 + i)) = -perp2[i];
        }
        
        return J;
    }

    void apply_impulse(float impulse_magnitude) override {
        if (!body_a_) return;
        
        // Similar to FixedConstraint but respecting hinge axis
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f r_a = R_a * pivot_a_local_;
        
        Vec3f impulse_dir = Vec3f(1, 0, 0);  // TODO: Use Jacobian column
        Vec3f impulse = impulse_dir * impulse_magnitude;
        
        body_a_->velocity += impulse * body_a_->inv_mass;
        Vec3f torque_a = r_a.cross(impulse);
        body_a_->angular_velocity += body_a_->inertia_tensor_inv * torque_a;
        
        if (body_b_) {
            body_b_->velocity -= impulse * body_b_->inv_mass;
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            Vec3f r_b = R_b * pivot_b_local_;
            Vec3f torque_b = r_b.cross(-impulse);
            body_b_->angular_velocity += body_b_->inertia_tensor_inv * torque_b;
        }
        
        accumulated_impulse_ += impulse_magnitude;
    }

    std::vector<size_t> get_body_ids() const override {
        std::vector<size_t> ids;
        if (body_a_) ids.push_back(static_cast<size_t>(body_a_->id));
        if (body_b_) ids.push_back(static_cast<size_t>(body_b_->id));
        return ids;
    }

    int num_constraint_rows() const override {
        return 5;  // Position (3) + Perpendicular rotation (2)
    }

    float get_accumulated_impulse() const override {
        return accumulated_impulse_;
    }

    void set_warm_start_impulse(float impulse) override {
        accumulated_impulse_ = impulse;
    }

private:
    physics::RigidBody* body_a_;
    physics::RigidBody* body_b_;
    Vec3f pivot_a_local_;
    Vec3f pivot_b_local_;
    Vec3f axis_local_;
    Vec3f axis_world_;  ///< Hinge axis in world space (cached)
    float accumulated_impulse_;
};

}  // namespace phynity::physics::constraints

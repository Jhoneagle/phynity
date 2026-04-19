#pragma once

#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_conversions.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/constraints/constraint.hpp>
#include <core/physics/constraints/weld_joint.hpp>
#include <core/physics/rigid_bodies/rigid_body.hpp>

#include <cmath>
#include <vector>

namespace phynity::physics::constraints
{

using phynity::math::matrices::Mat3f;
using phynity::math::quaternions::Quatf;
using phynity::math::vectors::Vec3f;

/// Hinge constraint for rigid bodies (5-DOF rotation).
/// Allows rotation around a hinge axis while locking the pivot point and other rotations.
/// Useful for doors, hinges, wheels, etc.
class HingeJoint : public Constraint
{
public:
    HingeJoint(physics::RigidBody *body_a,
                      physics::RigidBody *body_b,
                      const Vec3f &pivot_a_local, ///< Pivot point in body A's local frame
                      const Vec3f &pivot_b_local, ///< Pivot point in body B's local frame
                      const Vec3f &axis_local ///< Hinge axis in local frame (will be normalized)
                      )
        : body_a_(body_a),
          body_b_(body_b),
          pivot_a_local_(pivot_a_local),
          pivot_b_local_(pivot_b_local),
          axis_local_(axis_local.normalized()),
          accumulated_impulse_(0.0f)
    {
        if (body_a_)
        {
            // Store initial perpendicular directions
            Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
            axis_world_ = R_a * axis_local_;
        }
    }

    // ========================================================================
    // Constraint Interface Implementation
    // ========================================================================

    float compute_error() const override
    {
        if (!body_a_)
            return 0.0f;

        // Convert pivots to world space
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f pivot_a_world = body_a_->position + R_a * pivot_a_local_;

        Vec3f pivot_b_world = pivot_b_local_;
        if (body_b_)
        {
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            pivot_b_world = body_b_->position + R_b * pivot_b_local_;
        }

        // Position error: pivots must coincide
        Vec3f pos_error = pivot_b_world - pivot_a_world;
        float pos_error_mag = pos_error.length();

        // Rotation error: perpendicular directions must align
        Vec3f axis_a = (R_a * axis_local_).normalized();
        (void) axis_a;
        if (body_b_)
        {
            // TODO: Get hinge axis for body B if different
        }

        // For MVP: just return position error
        return pos_error_mag;
    }

    /// J * v for the positional part of the hinge constraint.
    float compute_jv() const override
    {
        if (!body_a_)
            return 0.0f;

        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f pivot_a_world = body_a_->position + R_a * pivot_a_local_;

        Vec3f pivot_b_world = pivot_b_local_;
        if (body_b_)
        {
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            pivot_b_world = body_b_->position + R_b * pivot_b_local_;
        }

        Vec3f error_vec = pivot_b_world - pivot_a_world;
        float error_len = error_vec.length();
        if (error_len < 1e-6f)
            return 0.0f;

        Vec3f dir = error_vec / error_len;
        Vec3f rel_vel = (body_b_ ? body_b_->velocity : Vec3f(0.0f)) - body_a_->velocity;
        return dir.dot(rel_vel);
    }

    /// Effective mass for the positional constraint direction.
    float compute_effective_mass() const override
    {
        if (!body_a_)
            return 0.0f;

        float eff = body_a_->inv_mass;
        if (body_b_)
            eff += body_b_->inv_mass;
        return eff;
    }

    void apply_impulse(float impulse_magnitude) override
    {
        apply_positional_impulse(body_a_, body_b_, pivot_a_local_, pivot_b_local_, impulse_magnitude);
        accumulated_impulse_ += impulse_magnitude;
    }

    int num_constraint_rows() const override
    {
        return 5; // Position (3) + Perpendicular rotation (2)
    }

    float get_accumulated_impulse() const override
    {
        return accumulated_impulse_;
    }

    void set_warm_start_impulse(float impulse) override
    {
        accumulated_impulse_ = impulse;
    }

private:
    physics::RigidBody *body_a_;
    physics::RigidBody *body_b_;
    Vec3f pivot_a_local_;
    Vec3f pivot_b_local_;
    Vec3f axis_local_;
    Vec3f axis_world_; ///< Hinge axis in world space (cached)
    float accumulated_impulse_;
};

} // namespace phynity::physics::constraints

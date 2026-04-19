#pragma once

#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_conversions.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/constraints/constraint.hpp>
#include <core/physics/rigid_bodies/rigid_body.hpp>

#include <cmath>
#include <vector>

namespace phynity::physics::constraints
{

using phynity::math::matrices::Mat3f;
using phynity::math::quaternions::Quatf;
using phynity::math::vectors::Vec3f;

/// Apply a positional correction impulse between two rigid bodies.
/// Computes world-space anchor positions, derives impulse direction from their separation,
/// and applies equal-and-opposite linear + angular impulses.
inline void apply_positional_impulse(physics::RigidBody *body_a,
                                     physics::RigidBody *body_b,
                                     const Vec3f &local_a,
                                     const Vec3f &local_b,
                                     float impulse_magnitude)
{
    if (!body_a)
        return;

    Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a->orientation);
    Vec3f r_a = R_a * local_a;
    Vec3f world_a = body_a->position + r_a;

    Vec3f world_b = local_b;
    Vec3f r_b = Vec3f(0.0f);
    if (body_b)
    {
        Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b->orientation);
        r_b = R_b * local_b;
        world_b = body_b->position + r_b;
    }

    Vec3f error_vec = world_b - world_a;
    float error_len = error_vec.length();
    Vec3f impulse_dir = (error_len > 1e-6f) ? error_vec / error_len : Vec3f(0.0f);
    Vec3f impulse = impulse_dir * impulse_magnitude;

    body_a->velocity += impulse * body_a->inv_mass;
    body_a->angular_velocity += body_a->inertia_tensor_inv * r_a.cross(impulse);

    if (body_b)
    {
        body_b->velocity -= impulse * body_b->inv_mass;
        body_b->angular_velocity -= body_b->inertia_tensor_inv * r_b.cross(impulse);
    }
}

/// Fixed constraint for rigid bodies (6-DOF lock).
/// Locks both position and orientation between two rigid bodies (or one body to world).
/// Can be used to create welded joints or attach bodies to fixed points.
class WeldJoint : public Constraint
{
public:
    WeldJoint(physics::RigidBody *body_a,
                      physics::RigidBody *body_b,
                      const Vec3f &anchor_a_local, ///< Anchor in body A's local frame
                      const Vec3f &anchor_b_local ///< Anchor in body B's local frame (or world if b is null)
                      )
        : body_a_(body_a),
          body_b_(body_b),
          anchor_a_local_(anchor_a_local),
          anchor_b_local_(anchor_b_local),
          accumulated_impulse_(0.0f)
    {
        if (body_a && body_b)
        {
            // Store initial relative orientation
            initial_relative_q_ = body_b->orientation.conjugate() * body_a->orientation;
        }
    }

    // ========================================================================
    // Constraint Interface Implementation
    // ========================================================================

    float compute_error() const override
    {
        if (!body_a_)
            return 0.0f;

        // Convert anchors from local to world space
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f anchor_a_world = body_a_->position + R_a * anchor_a_local_;

        Vec3f anchor_b_world = anchor_b_local_;
        if (body_b_)
        {
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            anchor_b_world = body_b_->position + R_b * anchor_b_local_;
        }

        // Position error (distance between anchors)
        Vec3f pos_error = anchor_b_world - anchor_a_world;
        float pos_error_mag = pos_error.length();

        // Rotation error (angle between orientations)
        Quatf relative_q = body_b_ ? body_b_->orientation * body_a_->orientation.conjugate() : body_a_->orientation;
        Quatf error_q = relative_q * initial_relative_q_.conjugate();

        // Axis-angle from quaternion: angle = 2 * acos(w)
        float rotation_error = 2.0f * std::acos(std::clamp(error_q.w, -1.0f, 1.0f));

        return std::max(pos_error_mag, rotation_error);
    }

    /// J * v for the positional part of the weld constraint.
    /// Simplified: relative linear velocity of anchors projected onto error direction.
    float compute_jv() const override
    {
        if (!body_a_)
            return 0.0f;

        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f anchor_a_world = body_a_->position + R_a * anchor_a_local_;

        Vec3f anchor_b_world = anchor_b_local_;
        if (body_b_)
        {
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            anchor_b_world = body_b_->position + R_b * anchor_b_local_;
        }

        Vec3f error_vec = anchor_b_world - anchor_a_world;
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
        apply_positional_impulse(body_a_, body_b_, anchor_a_local_, anchor_b_local_, impulse_magnitude);
        accumulated_impulse_ += impulse_magnitude;
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
    physics::RigidBody *body_a_; ///< First body (constrained, not null)
    physics::RigidBody *body_b_; ///< Second body (can be null for fixed-to-world)
    Vec3f anchor_a_local_; ///< Anchor point in body A's local frame
    Vec3f anchor_b_local_; ///< Anchor point in body B's local frame
    Quatf initial_relative_q_; ///< Initial relative orientation
    float accumulated_impulse_;
};

} // namespace phynity::physics::constraints

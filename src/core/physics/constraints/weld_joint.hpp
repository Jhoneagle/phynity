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
using phynity::math::matrices::MatDynamic;
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

    MatDynamic<float> compute_jacobian() const override
    {
        // 6 constraint equations: 3 position + 3 rotation
        // Columns: [v_a, ω_a, v_b, ω_b] (12 columns for 2 bodies)

        MatDynamic<float> J(6, 12, 0.0f);

        if (!body_a_)
            return J;

        // Convert anchor to world space
        Mat3f R_a = phynity::math::quaternions::toRotationMatrix(body_a_->orientation);
        Vec3f r_a = R_a * anchor_a_local_;

        // Position constraints (3 rows)
        // Jacobian for position: identity for linear, -r× for angular
        for (int i = 0; i < 3; ++i)
        {
            J(static_cast<size_t>(i), static_cast<size_t>(i)) = 1.0f; // ∂/∂v_a
        }

        // Skew-symmetric matrix of r_a for cross product (-r_a ×)
        // [-r_a ×] = [  0  -rz   ry ]
        //            [ rz   0  -rx ]
        //            [-ry  rx   0  ]
        J(0, 5) = r_a.z; // ∂(x)/∂ω_a.y = rz
        J(0, 4) = -r_a.y; // ∂(x)/∂ω_a.z = -ry
        J(1, 3) = -r_a.z; // ∂(y)/∂ω_a.x = -rz
        J(1, 5) = r_a.x; // ∂(y)/∂ω_a.z = rx
        J(2, 3) = r_a.y; // ∂(z)/∂ω_a.x = ry
        J(2, 4) = -r_a.x; // ∂(z)/∂ω_a.y = -rx

        if (body_b_)
        {
            Mat3f R_b = phynity::math::quaternions::toRotationMatrix(body_b_->orientation);
            Vec3f r_b = R_b * anchor_b_local_;

            // Negative coupling to body B
            for (int i = 0; i < 3; ++i)
            {
                J(static_cast<size_t>(i), static_cast<size_t>(6 + i)) = -1.0f; // -I for v_b
            }

            // Skew-symmetric for body B
            J(0, 11) = r_b.z;
            J(0, 10) = -r_b.y;
            J(1, 9) = -r_b.z;
            J(1, 11) = r_b.x;
            J(2, 9) = r_b.y;
            J(2, 10) = -r_b.x;
        }

        // Rotation constraints (3 rows) - simplified: relative orientation error
        // Use quaternion error for angular constraint
        for (int i = 0; i < 3; ++i)
        {
            J(static_cast<size_t>(3 + i), static_cast<size_t>(3 + i)) = 1.0f; // ω_a affects relative angular velocity
            if (body_b_)
            {
                J(static_cast<size_t>(3 + i), static_cast<size_t>(9 + i)) = -1.0f; // -ω_b
            }
        }

        return J;
    }

    void apply_impulse(float impulse_magnitude) override
    {
        apply_positional_impulse(body_a_, body_b_, anchor_a_local_, anchor_b_local_, impulse_magnitude);
        accumulated_impulse_ += impulse_magnitude;
    }

    std::vector<size_t> get_body_ids() const override
    {
        std::vector<size_t> ids;
        if (body_a_)
            ids.push_back(static_cast<size_t>(body_a_->id));
        if (body_b_)
            ids.push_back(static_cast<size_t>(body_b_->id));
        return ids;
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

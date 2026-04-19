#pragma once

#include <core/math/quaternions/quat_conversions.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/constraints/body.hpp>
#include <core/physics/constraints/constraint.hpp>

namespace phynity::physics::constraints
{

using phynity::math::matrices::Mat3f;
using phynity::math::quaternions::Quatf;
using phynity::math::vectors::Vec3f;

/// Base class for pivot-based constraints between two bodies.
/// Handles anchor-point error, Jv, effective mass (including angular terms),
/// and impulse application. Caches rotation matrices and world anchors
/// to avoid redundant computation across compute_error/compute_jv/apply_impulse.
class PivotJoint : public Constraint
{
public:
    PivotJoint(Body *body_a, Body *body_b, const Vec3f &anchor_a_local, const Vec3f &anchor_b_local)
        : body_a_(body_a),
          body_b_(body_b),
          anchor_a_local_(anchor_a_local),
          anchor_b_local_(anchor_b_local),
          accumulated_impulse_(0.0f)
    {
    }

    // ========================================================================
    // Cached state — call update_cache() once before using compute methods
    // ========================================================================

    /// Recompute cached world-space quantities from current body state.
    /// The solver calls this implicitly via compute_error (first method called per iteration).
    void update_cache() const
    {
        if (!body_a_)
            return;

        Quatf orient_a = body_a_->get_orientation();
        cached_R_a_ = phynity::math::quaternions::toRotationMatrix(orient_a);
        cached_r_a_ = cached_R_a_ * anchor_a_local_;
        cached_anchor_a_world_ = body_a_->get_position() + cached_r_a_;

        if (body_b_)
        {
            Quatf orient_b = body_b_->get_orientation();
            cached_R_b_ = phynity::math::quaternions::toRotationMatrix(orient_b);
            cached_r_b_ = cached_R_b_ * anchor_b_local_;
            cached_anchor_b_world_ = body_b_->get_position() + cached_r_b_;
        }
        else
        {
            cached_R_b_ = Mat3f(0.0f);
            cached_r_b_ = Vec3f(0.0f);
            cached_anchor_b_world_ = anchor_b_local_;
        }

        cached_error_vec_ = cached_anchor_b_world_ - cached_anchor_a_world_;
        cached_error_len_ = cached_error_vec_.length();
        cached_dir_ = (cached_error_len_ > 1e-6f) ? cached_error_vec_ / cached_error_len_ : Vec3f(0.0f);
        cache_valid_ = true;
    }

    // ========================================================================
    // Constraint Interface
    // ========================================================================

    float compute_error() const override
    {
        if (!body_a_)
            return 0.0f;

        update_cache();
        return compute_pivot_error();
    }

    float compute_jv() const override
    {
        if (!body_a_ || !cache_valid_)
            return 0.0f;

        if (cached_error_len_ < 1e-6f)
            return 0.0f;

        // Relative velocity of anchor points along error direction
        // v_anchor = v_body + omega x r
        Vec3f v_a = body_a_->get_velocity() + body_a_->get_angular_velocity().cross(cached_r_a_);
        Vec3f v_b =
            body_b_ ? body_b_->get_velocity() + body_b_->get_angular_velocity().cross(cached_r_b_) : Vec3f(0.0f);

        return cached_dir_.dot(v_b - v_a);
    }

    float compute_effective_mass() const override
    {
        if (!body_a_ || !cache_valid_)
            return 0.0f;

        // J * M^-1 * J^T for a positional pivot constraint with angular coupling:
        // eff = inv_mass_a + inv_mass_b
        //     + (r_a x n)^T * I_a_inv * (r_a x n)
        //     + (r_b x n)^T * I_b_inv * (r_b x n)
        float eff = body_a_->get_inverse_mass();

        Vec3f raxn = cached_r_a_.cross(cached_dir_);
        Mat3f I_a_inv = body_a_->get_inverse_inertia_world();
        eff += raxn.dot(I_a_inv * raxn);

        if (body_b_)
        {
            eff += body_b_->get_inverse_mass();

            Vec3f rbxn = cached_r_b_.cross(cached_dir_);
            Mat3f I_b_inv = body_b_->get_inverse_inertia_world();
            eff += rbxn.dot(I_b_inv * rbxn);
        }

        return eff;
    }

    void apply_impulse(float impulse_magnitude) override
    {
        if (!body_a_ || !cache_valid_)
            return;

        Vec3f impulse = cached_dir_ * impulse_magnitude;

        // Linear impulse
        body_a_->apply_velocity_impulse(-impulse * body_a_->get_inverse_mass());

        // Angular impulse: torque = r x F
        body_a_->apply_angular_impulse(-cached_r_a_.cross(impulse));

        if (body_b_)
        {
            body_b_->apply_velocity_impulse(impulse * body_b_->get_inverse_mass());
            body_b_->apply_angular_impulse(cached_r_b_.cross(impulse));
        }

        accumulated_impulse_ += impulse_magnitude;
    }

    // ========================================================================
    // Warm-Start
    // ========================================================================

    float get_accumulated_impulse() const override
    {
        return accumulated_impulse_;
    }
    void set_warm_start_impulse(float impulse) override
    {
        accumulated_impulse_ = impulse;
    }

protected:
    /// Override to add constraint-specific error (e.g., rotation error for WeldJoint).
    /// Default returns positional error only.
    virtual float compute_pivot_error() const
    {
        return cached_error_len_;
    }

    Body *body_a_;
    Body *body_b_;
    Vec3f anchor_a_local_;
    Vec3f anchor_b_local_;
    float accumulated_impulse_;

    // Cached per-iteration state (mutable for const compute methods)
    mutable Mat3f cached_R_a_;
    mutable Mat3f cached_R_b_;
    mutable Vec3f cached_r_a_;
    mutable Vec3f cached_r_b_;
    mutable Vec3f cached_anchor_a_world_;
    mutable Vec3f cached_anchor_b_world_;
    mutable Vec3f cached_error_vec_;
    mutable float cached_error_len_ = 0.0f;
    mutable Vec3f cached_dir_;
    mutable bool cache_valid_ = false;
};

} // namespace phynity::physics::constraints

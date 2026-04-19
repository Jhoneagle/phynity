#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/constraints/body.hpp>
#include <core/physics/constraints/constraint.hpp>

#include <cmath>

namespace phynity::physics::constraints
{

using phynity::math::vectors::Vec3f;

/// Distance joint: maintains a fixed distance between two bodies.
/// Works with any Body type (particles, rigid bodies, or mixed).
class DistanceJoint : public Constraint
{
public:
    DistanceJoint(Body &body_a, Body &body_b, const Vec3f &offset = Vec3f(0.0f))
        : body_a_(body_a),
          body_b_(body_b),
          rest_offset_(offset.length() > 1e-5f ? offset : (body_b.get_position() - body_a.get_position())),
          accumulated_impulse_(0.0f)
    {
        rest_distance_ = rest_offset_.length();
    }

    // ========================================================================
    // Constraint Interface
    // ========================================================================

    float compute_error() const override
    {
        const float current_distance = get_current_distance();
        return std::abs(current_distance - rest_distance_);
    }

    /// J * v for a distance constraint: relative velocity along the constraint direction
    /// J = [-d, d] where d is the unit direction from A to B
    /// J * v = d . (v_b - v_a)
    float compute_jv() const override
    {
        if (!is_active())
            return 0.0f;

        const Vec3f direction = get_direction();
        if (direction.squaredLength() < 1e-12f)
            return 0.0f;

        Vec3f v_a = body_a_.get_velocity();
        Vec3f v_b = body_b_.get_velocity();
        return direction.dot(v_b - v_a);
    }

    /// J * M^-1 * J^T = inv_mass_a + inv_mass_b (direction is unit length)
    float compute_effective_mass() const override
    {
        if (!is_active())
            return 0.0f;

        return body_a_.get_inverse_mass() + body_b_.get_inverse_mass();
    }

    void apply_impulse(float impulse_magnitude) override
    {
        if (!is_active())
            return;

        const Vec3f direction = get_direction();
        if (direction.squaredLength() < 1e-12f)
            return;

        const Vec3f impulse_vector = direction * impulse_magnitude;

        if (body_a_.get_inverse_mass() > 0.0f)
        {
            body_a_.apply_velocity_impulse(-impulse_vector * body_a_.get_inverse_mass());
        }

        if (body_b_.get_inverse_mass() > 0.0f)
        {
            body_b_.apply_velocity_impulse(impulse_vector * body_b_.get_inverse_mass());
        }

        accumulated_impulse_ += impulse_magnitude;
    }

    // ========================================================================
    // Warm-Start
    // ========================================================================

    void set_warm_start_impulse(float impulse) override
    {
        if (impulse > 0.0f && get_current_distance() > 1e-6f)
        {
            apply_impulse(impulse);
        }
    }

    float get_accumulated_impulse() const override
    {
        return std::abs(accumulated_impulse_);
    }

    // ========================================================================
    // Status & Accessors
    // ========================================================================

    bool is_active() const override
    {
        return body_a_.is_alive() && body_b_.is_alive();
    }

    float get_rest_distance() const { return rest_distance_; }
    const Vec3f &get_rest_offset() const { return rest_offset_; }

    float get_current_distance() const
    {
        return (body_b_.get_position() - body_a_.get_position()).length();
    }

private:
    /// Get unit direction from A to B (returns zero if degenerate)
    Vec3f get_direction() const
    {
        const Vec3f offset = body_b_.get_position() - body_a_.get_position();
        const float distance = offset.length();
        if (distance < 1e-6f)
            return Vec3f(0.0f);
        return offset * (1.0f / distance);
    }

    Body &body_a_;
    Body &body_b_;
    Vec3f rest_offset_;
    float rest_distance_;
    float accumulated_impulse_;
};

} // namespace phynity::physics::constraints

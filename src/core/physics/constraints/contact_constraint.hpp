#pragma once

#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/constraints/body.hpp>
#include <core/physics/constraints/constraint.hpp>

#include <algorithm>
#include <cstdint>

namespace phynity::physics::constraints
{

using phynity::math::matrices::Mat3f;
using phynity::math::vectors::Vec3f;
using phynity::physics::collision::ContactManifold;

/// Contact constraint: represents a collision between two bodies.
/// Wraps a ContactManifold and computes solver quantities directly from body state.
/// Includes angular coupling for rigid bodies via the contact point lever arm.
class ContactConstraint : public Constraint
{
public:
    enum class ContactType
    {
        Normal,
        Tangent
    };

    ContactConstraint(const ContactManifold &manifold,
                      Body &body_a,
                      Body &body_b,
                      ContactType contact_type = ContactType::Normal)
        : manifold_(manifold), body_a_(body_a), body_b_(body_b), contact_type_(contact_type), accumulated_impulse_(0.0f)
    {
        if (!manifold_.is_valid())
        {
            active_ = false;
            return;
        }

        // Precompute lever arms from body centers to contact point
        r_a_ = manifold_.contact.position - body_a_.get_position();
        r_b_ = manifold_.contact.position - body_b_.get_position();
    }

    // ========================================================================
    // Constraint Interface
    // ========================================================================

    float compute_error() const override
    {
        return std::max(0.0f, manifold_.contact.penetration);
    }

    float compute_jv() const override
    {
        if (!active_)
            return 0.0f;

        const Vec3f &n = manifold_.contact.normal;

        // Velocity at contact point = v + omega x r
        Vec3f v_a = body_a_.get_velocity() + body_a_.get_angular_velocity().cross(r_a_);
        Vec3f v_b = body_b_.get_velocity() + body_b_.get_angular_velocity().cross(r_b_);

        return n.dot(v_b - v_a);
    }

    float compute_effective_mass() const override
    {
        if (!active_)
            return 0.0f;

        const Vec3f &n = manifold_.contact.normal;

        // J * M^-1 * J^T including angular terms:
        // eff = inv_mass_a + inv_mass_b
        //     + (r_a x n)^T * I_a_inv * (r_a x n)
        //     + (r_b x n)^T * I_b_inv * (r_b x n)
        float eff = body_a_.get_inverse_mass() + body_b_.get_inverse_mass();

        Vec3f raxn = r_a_.cross(n);
        Mat3f I_a_inv = body_a_.get_inverse_inertia_world();
        eff += raxn.dot(I_a_inv * raxn);

        Vec3f rbxn = r_b_.cross(n);
        Mat3f I_b_inv = body_b_.get_inverse_inertia_world();
        eff += rbxn.dot(I_b_inv * rbxn);

        return eff;
    }

    void apply_impulse(float impulse_magnitude) override
    {
        if (!active_)
            return;

        accumulated_impulse_ += impulse_magnitude;

        const Vec3f &normal = manifold_.contact.normal;
        const Vec3f impulse_vector = normal * impulse_magnitude;

        // Linear impulse
        if (body_a_.get_inverse_mass() > 0.0f)
        {
            body_a_.apply_velocity_impulse(-impulse_vector * body_a_.get_inverse_mass());
        }
        if (body_b_.get_inverse_mass() > 0.0f)
        {
            body_b_.apply_velocity_impulse(impulse_vector * body_b_.get_inverse_mass());
        }

        // Angular impulse: torque = r x impulse
        body_a_.apply_angular_impulse(-r_a_.cross(impulse_vector));
        body_b_.apply_angular_impulse(r_b_.cross(impulse_vector));
    }

    // ========================================================================
    // Warm-Start & Status
    // ========================================================================

    void set_warm_start_impulse(float impulse) override
    {
        (void) impulse;
    }
    float get_accumulated_impulse() const override
    {
        return accumulated_impulse_;
    }

    bool is_active() const override
    {
        return active_ && manifold_.is_valid();
    }
    bool is_unilateral() const override
    {
        return true;
    }

    float get_restitution() const override
    {
        return std::min(body_a_.get_restitution(), body_b_.get_restitution());
    }

    float get_initial_approach_velocity() const override
    {
        return manifold_.contact.relative_velocity_along_normal;
    }

    const ContactManifold &get_manifold() const
    {
        return manifold_;
    }
    ContactType get_contact_type() const
    {
        return contact_type_;
    }
    uint64_t get_contact_id() const
    {
        return manifold_.contact_id;
    }

private:
    const ContactManifold &manifold_;
    Body &body_a_;
    Body &body_b_;
    Vec3f r_a_; ///< Lever arm: contact point relative to body A center
    Vec3f r_b_; ///< Lever arm: contact point relative to body B center
    ContactType contact_type_;
    float accumulated_impulse_;
    bool active_ = true;
};

} // namespace phynity::physics::constraints

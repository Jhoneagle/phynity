#pragma once

#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/constraints/body.hpp>
#include <core/physics/constraints/constraint.hpp>

#include <algorithm>
#include <cstdint>

namespace phynity::physics::constraints
{

using phynity::math::vectors::Vec3f;
using phynity::physics::collision::ContactManifold;

/// Contact constraint: represents a collision between two bodies.
/// Wraps a ContactManifold and computes solver quantities directly from body state.
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
        : manifold_(manifold),
          body_a_(body_a),
          body_b_(body_b),
          contact_type_(contact_type),
          accumulated_impulse_(0.0f)
    {
        if (!manifold_.is_valid())
        {
            active_ = false;
        }
    }

    // ========================================================================
    // Constraint Interface
    // ========================================================================

    float compute_error() const override
    {
        return std::max(0.0f, manifold_.contact.penetration);
    }

    /// J * v for a contact: relative velocity along normal = (v_b - v_a) . normal
    float compute_jv() const override
    {
        if (!active_)
            return 0.0f;

        const Vec3f &n = manifold_.contact.normal;
        Vec3f v_a = body_a_.get_velocity();
        Vec3f v_b = body_b_.get_velocity();

        // J = [-n, n], so J*v = -n.v_a + n.v_b = n.(v_b - v_a)
        return n.dot(v_b - v_a);
    }

    /// J * M^-1 * J^T for a contact: inv_mass_a + inv_mass_b (for linear-only)
    float compute_effective_mass() const override
    {
        if (!active_)
            return 0.0f;

        // For J = [-n, n] and diagonal M^-1:
        // J * M^-1 * J^T = |n|^2 * (inv_mass_a + inv_mass_b) = inv_mass_a + inv_mass_b
        // (since n is unit length)
        return body_a_.get_inverse_mass() + body_b_.get_inverse_mass();
    }

    void apply_impulse(float impulse_magnitude) override
    {
        if (!active_)
            return;

        accumulated_impulse_ += impulse_magnitude;

        const Vec3f &normal = manifold_.contact.normal;
        const Vec3f impulse_vector = normal * impulse_magnitude;

        if (body_a_.get_inverse_mass() > 0.0f)
        {
            body_a_.apply_velocity_impulse(-impulse_vector * body_a_.get_inverse_mass());
        }

        if (body_b_.get_inverse_mass() > 0.0f)
        {
            body_b_.apply_velocity_impulse(impulse_vector * body_b_.get_inverse_mass());
        }
    }

    // ========================================================================
    // Warm-Start
    // ========================================================================

    void set_warm_start_impulse(float impulse) override { (void)impulse; }
    float get_accumulated_impulse() const override { return accumulated_impulse_; }

    // ========================================================================
    // Status & Accessors
    // ========================================================================

    bool is_active() const override { return active_ && manifold_.is_valid(); }
    bool is_unilateral() const override { return true; }

    float get_restitution() const override
    {
        return std::min(body_a_.get_restitution(), body_b_.get_restitution());
    }

    float get_initial_approach_velocity() const override
    {
        return manifold_.contact.relative_velocity_along_normal;
    }

    const ContactManifold &get_manifold() const { return manifold_; }
    ContactType get_contact_type() const { return contact_type_; }
    uint64_t get_contact_id() const { return manifold_.contact_id; }

private:
    const ContactManifold &manifold_;
    Body &body_a_;
    Body &body_b_;
    ContactType contact_type_;
    float accumulated_impulse_;
    bool active_ = true;
};

} // namespace phynity::physics::constraints

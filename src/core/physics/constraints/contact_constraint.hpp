#pragma once

#include <core/physics/constraints/constraint.hpp>
#include <core/physics/collision/contact_manifold.hpp>
#include <core/physics/particle.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <vector>

namespace phynity::physics::constraints {

using phynity::physics::collision::ContactManifold;
using phynity::math::vectors::Vec3f;

/// Contact constraint: represents a collision between two particles.
/// This constraint wraps a ContactManifold and provides the interface
/// needed for solving contact constraints alongside other rigid constraints.
class ContactConstraint : public Constraint {
public:
    /// Contact constraint types
    enum class ContactType {
        Normal,      ///< Normal force (perpendicular to contact surface)
        Tangent      ///< Friction force (parallel to contact surface)
    };

    // ========================================================================
    // Construction
    // ========================================================================

    /// Create a contact constraint from a manifold and particle references.
    /// @param manifold ContactManifold from collision detection
    /// @param body_a First particle involved in contact
    /// @param body_b Second particle involved in contact
    /// @param contact_type Whether this constraint is normal or tangent (friction)
    ContactConstraint(
        const ContactManifold& manifold,
        Particle& body_a,
        Particle& body_b,
        ContactType contact_type = ContactType::Normal
    )
        : manifold_(manifold)
        , body_a_(body_a)
        , body_b_(body_b)
        , contact_type_(contact_type)
        , accumulated_impulse_(0.0f)
        , warm_start_impulse_(manifold.previous_impulse.x)  // For normal contact
    {
        // Verify valid manifold
        if (!manifold_.is_valid()) {
            active_ = false;
        }
    }

    // ========================================================================
    // Constraint Interface Implementation
    // ========================================================================

    /// Compute the constraint error (penetration depth).
    /// @return Positive value indicates constraint violation (penetration)
    float compute_error() const override {
        return std::max(0.0f, manifold_.contact.penetration);
    }

    /// Compute the Jacobian for this contact constraint.
    /// For a simple particle contact:
    ///   J = [-normal, normal]  (linear only, no angular components)
    /// This relates relative velocity along the normal to the constraint violation rate.
    MatDynamic<float> compute_jacobian() const override {
        MatDynamic<float> jacobian(1, 6);  // 1 row, 6 columns (3 DOF for body A, 3 for body B)

        if (!active_) {
            return jacobian;
        }

        // Contact normal direction (from A to B)
        const Vec3f& normal = manifold_.contact.normal;

        // For two particles with linear velocity only:
        // J = [-normal, normal]  (normal points from A to B)
        jacobian(0, 0) = -normal.x;
        jacobian(0, 1) = -normal.y;
        jacobian(0, 2) = -normal.z;
        jacobian(0, 3) = normal.x;
        jacobian(0, 4) = normal.y;
        jacobian(0, 5) = normal.z;

        return jacobian;
    }

    /// Apply an impulse along the contact normal.
    /// This modifies velocities of both particles.
    /// @param impulse_magnitude The impulse to apply (clamped to non-negative for normal contact)
    void apply_impulse(float impulse_magnitude) override {
        if (!active_) {
            return;
        }

        // Clamp impulse to non-negative for normal contact (no pulling)
        const float clamped_impulse = std::max(0.0f, impulse_magnitude);

        // Accumulate for warm-starting
        accumulated_impulse_ += clamped_impulse;

        // Compute impulse vector: impulse_magnitude * contact_normal
        const Vec3f& normal = manifold_.contact.normal;
        const Vec3f impulse_vector = normal * clamped_impulse;

        // Apply impulse to bodies (F = m * a, so dv = impulse / m)
        // Body A: receives negative impulse (stays in place)
        // Body B: receives positive impulse (pushed away)
        if (body_a_.inverse_mass() > 0.0f) {
            body_a_.velocity -= impulse_vector * body_a_.inverse_mass();
        }

        if (body_b_.inverse_mass() > 0.0f) {
            body_b_.velocity += impulse_vector * body_b_.inverse_mass();
        }
    }

    /// Get the body IDs for this contact.
    std::vector<size_t> get_body_ids() const override {
        return { manifold_.object_a_id, manifold_.object_b_id };
    }

    // ========================================================================
    // Warm-Start Support
    // ========================================================================

    /// Set warm-start impulse from previous frame.
    void set_warm_start_impulse(float impulse) override {
        warm_start_impulse_ = impulse;
    }

    /// Get the current accumulated impulse for caching.
    float get_accumulated_impulse() const override {
        return accumulated_impulse_;
    }

    // ========================================================================
    // Status Queries
    // ========================================================================

    /// Check if contact is still active.
    bool is_active() const override {
        return active_ && manifold_.is_valid();
    }

    /// Contact constraints are unilateral (can only push, not pull).
    bool is_unilateral() const override {
        return true;
    }

    /// Get the contact manifold data.
    const ContactManifold& get_manifold() const {
        return manifold_;
    }

    /// Get the contact type (normal or tangent).
    ContactType get_contact_type() const {
        return contact_type_;
    }

    /// Get the contact ID for cache tracking.
    uint64_t get_contact_id() const {
        return manifold_.contact_id;
    }

    /// Get the combined coefficient of restitution for this contact.
    /// Uses the minimum of the two bodies' restitution values.
    float get_restitution() const override {
        return std::min(body_a_.material.restitution, body_b_.material.restitution);
    }

    /// Get the initial approach velocity (for restitution calculation).
    /// This is the relative velocity along the normal at first contact.
    float get_initial_approach_velocity() const {
        return manifold_.contact.relative_velocity_along_normal;
    }

private:
    const ContactManifold& manifold_;   ///< Reference to contact data
    Particle& body_a_;                  ///< First particle
    Particle& body_b_;                  ///< Second particle
    ContactType contact_type_;          ///< Normal or tangent/friction
    float accumulated_impulse_;         ///< Total impulse applied (for warm-start caching)
    float warm_start_impulse_;          ///< Impulse from previous frame (for warm-start)
    bool active_ = true;                ///< Whether this contact is active
};

}  // namespace phynity::physics::constraints

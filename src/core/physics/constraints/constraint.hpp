#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/constraints/body.hpp>

#include <cstdint>

namespace phynity::physics::constraints
{

using phynity::math::vectors::Vec3f;

/// Abstract base class for all constraints.
/// A constraint enforces a relationship between one or more bodies.
/// Each constraint stores its own body references and computes solver
/// quantities (Jv, effective mass, impulse application) directly —
/// the solver never needs to look up bodies externally.
class Constraint
{
public:
    virtual ~Constraint() = default;

    // ========================================================================
    // Core Constraint Interface
    // ========================================================================

    /// Compute the constraint error (violation) for the current state.
    /// @return Constraint error (positive = violated, 0 = satisfied)
    virtual float compute_error() const = 0;

    /// Compute the Jacobian-velocity product (J * v) for this constraint.
    /// This represents the rate of constraint violation given current body velocities.
    /// @return Dot product of constraint Jacobian with body velocity vector
    virtual float compute_jv() const = 0;

    /// Compute the effective mass denominator (J * M^-1 * J^T) for this constraint.
    /// This represents the resistance to impulse application.
    /// @return Effective mass (positive = solvable, near-zero = singular)
    virtual float compute_effective_mass() const = 0;

    /// Apply an impulse to the constrained bodies.
    /// Modifies body velocities according to the impulse magnitude and constraint direction.
    /// @param impulse_magnitude The magnitude of the impulse to apply
    virtual void apply_impulse(float impulse_magnitude) = 0;

    // ========================================================================
    // Warm-Start Support (Optional)
    // ========================================================================

    /// Set warm-start impulse from previous frame (for faster convergence).
    virtual void set_warm_start_impulse(float impulse)
    {
        (void) impulse;
    }

    /// Get the current accumulated impulse (for caching to next frame).
    virtual float get_accumulated_impulse() const
    {
        return 0.0f;
    }

    // ========================================================================
    // Constraint Type Information
    // ========================================================================

    /// Get the number of independent constraint equations.
    virtual int num_constraint_rows() const
    {
        return 1;
    }

    /// Check if this constraint is active (should be solved).
    virtual bool is_active() const
    {
        return true;
    }

    /// Check if impulses should be clamped to non-negative (contact = unilateral).
    virtual bool is_unilateral() const
    {
        return false;
    }

    /// Get the coefficient of restitution for this constraint.
    virtual float get_restitution() const
    {
        return 0.0f;
    }

    /// Get the initial approach velocity for contact constraints.
    virtual float get_initial_approach_velocity() const
    {
        return 0.0f;
    }
};

} // namespace phynity::physics::constraints

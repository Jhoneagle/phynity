#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/math/matrices/mat_dynamic.hpp>
#include <core/physics/particle.hpp>
#include <vector>
#include <cstdint>

namespace phynity::physics::constraints {

using phynity::math::vectors::Vec3f;
using phynity::math::matrices::MatDynamic;

/// Abstract base class for all constraints.
/// A constraint enforces a relationship between one or more bodies.
/// Examples: contact constraints (collision), fixed joints, hinge joints, etc.
class Constraint {
public:
    virtual ~Constraint() = default;

    // ========================================================================
    // Core Constraint Interface
    // ========================================================================

    /// Compute the constraint error (violation) for the current state.
    /// For contact constraints, this is penetration depth.
    /// For joint constraints, this is deviation from desired configuration.
    /// @return Constraint error (positive = violated, 0 = satisfied)
    virtual float compute_error() const = 0;

    /// Compute the Jacobian matrix for this constraint.
    /// The Jacobian relates body velocities to constraint violation rate.
    /// Each row corresponds to one constraint equation (e.g., normal contact direction).
    /// For a bilateral constraint on 2 bodies:
    ///   Column format: [v_a_x, v_a_y, v_a_z, v_b_x, v_b_y, v_b_z]
    ///   (6 columns for 2 particles, 3 DOF each; angular DOF omitted)
    /// For particles (no angular velocity), only 3 columns per body.
    /// @return MatDynamic with one row per constraint equation
    virtual MatDynamic<float> compute_jacobian() const = 0;

    /// Apply an impulse to the constrained bodies.
    /// This modifies the bodies' velocities according to the impulse magnitude and direction.
    /// @param impulse_magnitude The magnitude of the impulse to apply (may be clamped by constraint type)
    virtual void apply_impulse(float impulse_magnitude) = 0;

    /// Get the associated body IDs for this constraint.
    /// @return Vector of particle/body indices involved in the constraint
    virtual std::vector<size_t> get_body_ids() const = 0;

    // ========================================================================
    // Warm-Start Support (Optional)
    // ========================================================================

    /// Set warm-start impulse from previous frame (for faster convergence).
    /// @param impulse The impulse applied in the previous frame
    /// @note Default implementation does nothing; override if your constraint supports warm-start
    virtual void set_warm_start_impulse(float impulse) {
        (void)impulse;  // Unused by default
    }

    /// Get the current accumulated impulse (for caching to next frame).
    /// @return Current impulse magnitude for warm-starting
    virtual float get_accumulated_impulse() const {
        return 0.0f;
    }

    // ========================================================================
    // Constraints Type Information
    // ========================================================================

    /// Get the number of independent constraint equations.
    /// For single-contact normal: 1
    /// For contact with friction (normal + tangent): 2
    /// For fixed joint (position + rotation): 6
    /// @return Number of constraint rows
    virtual int num_constraint_rows() const {
        return 1;  // Default: single constraint equation
    }

    /// Check if this constraint is active (should be solved).
    /// Deactivate constraints that are satisfied or have aged out.
    /// @return True if constraint should be processed by solver, false if inactive
    virtual bool is_active() const {
        return true;  // Default: all constraints are active
    }

    /// Check if impulses for this constraint should be clamped to non-negative.
    /// Contact constraints are unilateral (can only push, not pull).
    /// Fixed/joint constraints are bilateral (can push or pull).
    /// @return True if impulse should be clamped to >= 0, false for bilateral constraints
    virtual bool is_unilateral() const {
        return false;  // Default: bilateral (allow negative impulses)
    }
};

}  // namespace phynity::physics::constraints

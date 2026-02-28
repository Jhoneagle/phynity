#pragma once

#include <core/physics/constraints/solver/constraint.hpp>
#include <core/physics/micro/particle.hpp>
#include <core/math/vectors/vec3.hpp>
#include <cmath>
#include <vector>

namespace phynity::physics::constraints {

using phynity::math::vectors::Vec3f;

/// Fixed constraint: rigidly attaches two particles at a fixed offset.
/// The particles move together, maintaining their relative position and orientation.
/// This is equivalent to a welded joint with 6 DOF locked (in 3D) or 3 DOF locked (in 2D).
class FixedConstraint : public Constraint {
public:
    // ========================================================================
    // Construction
    // ========================================================================

    /// Create a fixed constraint between two particles.
    /// The constraint locks the particles at their current relative offset.
    /// @param body_a First particle
    /// @param body_b Second particle
    /// @param offset Optional relative position offset (defaults to current offset)
    FixedConstraint(
        Particle& body_a,
        Particle& body_b,
        const Vec3f& offset = Vec3f(0.0f)
    )
        : body_a_(body_a)
        , body_b_(body_b)
        , rest_offset_(offset.length() > 1e-5f ? offset : (body_b.position - body_a.position))
        , accumulated_impulse_(Vec3f(0.0f))
        , max_iterations_(10)
    {
        // Pre-compute rest distance for stability
        rest_distance_ = rest_offset_.length();
    }

    // ========================================================================
    // Constraint Interface Implementation
    // ========================================================================

    /// Compute the constraint error (distance deviation from rest state).
    /// Error is the difference between current distance and rest distance.
    /// @return Positive value indicates constraint violation (distance mismatch)
    float compute_error() const override {
        const Vec3f current_offset = body_b_.position - body_a_.position;
        const float current_distance = current_offset.length();
        const float error = current_distance - rest_distance_;
        return std::abs(error);  // Penalize both compression and extension
    }

    /// Compute the Jacobian for this fixed constraint.
    /// For a distance constraint: J = [direction_normalized, -direction_normalized]
    /// where direction is the vector from A to B.
    MatDynamic<float> compute_jacobian() const override {
        MatDynamic<float> jacobian(1, 6);  // 1 row, 6 columns (3 DOF for body A, 3 for body B)

        if (!is_active()) {
            return jacobian;
        }

        const Vec3f offset = body_b_.position - body_a_.position;
        const float distance = offset.length();

        if (distance < 1e-6f) {
            // Degenerate case: particles are at same position
            return jacobian;
        }

        // Normalized direction from A to B
        const Vec3f direction = offset * (1.0f / distance);

        // Jacobian row: direction for A, -direction for B
        jacobian(0, 0) = -direction.x;
        jacobian(0, 1) = -direction.y;
        jacobian(0, 2) = -direction.z;
        jacobian(0, 3) = direction.x;
        jacobian(0, 4) = direction.y;
        jacobian(0, 5) = direction.z;

        return jacobian;
    }

    /// Apply an impulse along the distance constraint direction.
    /// This corrects the relative position by modifying velocities.
    /// @param impulse_magnitude The impulse to apply
    void apply_impulse(float impulse_magnitude) override {
        if (!is_active()) {
            return;
        }

        const Vec3f offset = body_b_.position - body_a_.position;
        const float distance = offset.length();

        if (distance < 1e-6f) {
            return;  // Degenerate case
        }

        // Direction from A to B
        const Vec3f direction = offset * (1.0f / distance);

        // Create impulse vector
        const Vec3f impulse_vector = direction * impulse_magnitude;

        // Apply impulse to bodies
        if (body_a_.inverse_mass() > 0.0f) {
            body_a_.velocity -= impulse_vector * body_a_.inverse_mass();
        }

        if (body_b_.inverse_mass() > 0.0f) {
            body_b_.velocity += impulse_vector * body_b_.inverse_mass();
        }

        // Accumulate for warm-starting
        accumulated_impulse_ += impulse_vector * impulse_magnitude;
    }

    /// Get the body IDs for this constraint.
    std::vector<size_t> get_body_ids() const override {
        // Note: In full implementation, would need to track particle indices
        // For now, return a placeholder
        return { 0, 1 };  // Would need actual IDs from particles
    }

    // ========================================================================
    // Warm-Start Support
    // ========================================================================

    /// Set warm-start impulse from previous frame.
    void set_warm_start_impulse(float impulse) override {
        // For fixed constraints, apply the entire warm-start impulse immediately
        // This provides faster convergence to the rest configuration
        if (impulse > 0.0f) {
            const Vec3f offset = body_b_.position - body_a_.position;
            const float distance = offset.length();
            if (distance > 1e-6f) {
                apply_impulse(impulse);
            }
        }
    }

    /// Get the current accumulated impulse.
    float get_accumulated_impulse() const override {
        return accumulated_impulse_.length();
    }

    // ========================================================================
    // Status Queries
    // ========================================================================

    /// Check if constraint is active.
    /// A fixed constraint is inactive if either body is dead.
    bool is_active() const override {
        return body_a_.is_alive() && body_b_.is_alive();
    }

    /// Get the rest distance between particles.
    float get_rest_distance() const {
        return rest_distance_;
    }

    /// Get the rest offset (relative position).
    const Vec3f& get_rest_offset() const {
        return rest_offset_;
    }

    /// Get the current distance between particles.
    float get_current_distance() const {
        const Vec3f offset = body_b_.position - body_a_.position;
        return offset.length();
    }

    /// Get the constraint iteration count for diagnostic purposes.
    int num_constraint_rows() const override {
        return 1;  // Distance constraint is a single equation
    }

private:
    Particle& body_a_;              ///< First particle
    Particle& body_b_;              ///< Second particle
    Vec3f rest_offset_;             ///< Desired relative position (direction and magnitude)
    float rest_distance_;           ///< Distance to maintain between particles
    Vec3f accumulated_impulse_;     ///< Accumulated impulse vector (for warm-start)
    int max_iterations_;            ///< Max iterations to converge (for reference)
};

}  // namespace phynity::physics::constraints

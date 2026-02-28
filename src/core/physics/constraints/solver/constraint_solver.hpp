#pragma once

#include <core/physics/constraints/solver/constraint.hpp>
#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/micro/particle.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <vector>
#include <memory>
#include <algorithm>

namespace phynity::physics::constraints {

using phynity::physics::collision::ContactManifold;
using phynity::math::vectors::Vec3f;

/// Solver configuration for constraint solving
struct ConstraintSolverConfig {
    int iterations = 4;                    ///< Number of solver iterations (higher = more stable but slower)
    float convergence_threshold = 1e-5f;   ///< Convergence criterion (stop if impulse change < threshold)
    float baumgarte_beta = 0.2f;           ///< Baumgarte penetration correction parameter (0.0-0.5)
    bool use_warm_start = true;            ///< Whether to use warm-start from previous impulses
    bool enable_adaptive_iterations = true; ///< Increase iterations for large contact sets
};

/// Unified constraint solver using Projected Gauss-Seidel (PGS).
/// This solver handles all constraints uniformly:
/// - Contact constraints from collisions
/// - Rigid constraints (fixed joints, hinges, etc.)
///
/// Algorithm:
/// 1. If warm-start enabled: apply cached impulses from previous frame
/// 2. For each iteration:
///    a. For each constraint:
///       - Compute constraint error
///       - Compute Jacobian
///       - Solve for impulse magnitude
///       - Clamp impulse (for contact: non-negative)
///       - Apply impulse, update body velocities
///    b. Check convergence: if impulse changes < threshold, early exit
/// 3. Cache final impulses for next frame's warm-start
///
/// Benefits over sequential impulse resolution:
/// - Faster convergence for multi-contact scenarios
/// - Better stacking stability with iterative refinement
/// - Unified framework for collision + constraint solving
class ConstraintSolver {
public:
    // ========================================================================
    // Construction & Configuration
    // ========================================================================

    ConstraintSolver() = default;

    /// Set solver configuration
    void set_config(const ConstraintSolverConfig& config) {
        config_ = config;
    }

    /// Get current solver configuration
    const ConstraintSolverConfig& config() const {
        return config_;
    }

    // ========================================================================
    // Main Solver Interface
    // ========================================================================

    /// Solve all constraints in the system.
    /// This is the main entry point for unified constraint solving.
    /// @param constraints Vector of constraints to solve (contacts + rigid constraints)
    /// @param particles   Reference to all particles for constraint access
    void solve(std::vector<std::unique_ptr<Constraint>>& constraints, std::vector<Particle>& particles) {
        PROFILE_FUNCTION();

        // Filter out inactive constraints
        std::vector<Constraint*> active_constraints;
        for (auto& constraint : constraints) {
            if (constraint && constraint->is_active()) {
                active_constraints.push_back(constraint.get());
            }
        }

        if (active_constraints.empty()) {
            return;  // No constraints to solve
        }

        // Determine iteration count
        int iterations = config_.iterations;
        if (config_.enable_adaptive_iterations) {
            // Increase iterations for large constraint sets
            const size_t constraint_count = active_constraints.size();
            if (constraint_count > 20) {
                iterations = std::min(iterations * 2, 16);  // Cap at 16 iterations
            }
        }

        // Phase 1: Apply warm-start impulses (if enabled)
        if (config_.use_warm_start) {
            PROFILE_SCOPE("warm_start");
            for (auto constraint : active_constraints) {
                float warm_start = constraint->get_accumulated_impulse();
                if (warm_start > 1e-6f) {
                    constraint->apply_impulse(warm_start);
                }
            }
        }

        // Phase 2: Main solve loop with iterative refinement
        float max_impulse_change = 0.0f;

        for (int iter = 0; iter < iterations; ++iter) {
            PROFILE_SCOPE("iteration");

            max_impulse_change = 0.0f;

            for (auto constraint : active_constraints) {
                // Get constraint info
                float error = constraint->compute_error();
                const auto jacobian = constraint->compute_jacobian();

                if (jacobian.isEmpty() || jacobian.numRows() == 0) {
                    continue;
                }

                // Solve the first constraint equation (row 0)
                // Future: extend to handle multi-row constraints
                size_t row = 0;

                // Compute Jacobian-velocity product (relative velocity along constraint)
                // J * v = dot(jacobian[row], [v_a, v_b])
                float jv = compute_jacobian_velocity(jacobian, row, constraint->get_body_ids(), particles);

                // Compute impulse magnitude
                // Using Baumgarte stabilization: add error-correcting term
                // For contact constraints with restitution: add velocity bounce target
                const float error_correction = config_.baumgarte_beta * error;
                
                // Add restitution target velocity for contact constraints
                // Restitution is based on INITIAL approach velocity, not current velocity
                // This ensures restitution is only applied once, not re-computed each iteration
                float restitution_term = 0.0f;
                if (constraint->is_unilateral() && iter == 0) {  // Contact constraint, first iteration only
                    float e = constraint->get_restitution();
                    float v_approach = constraint->get_initial_approach_velocity();
                    if (v_approach < -1e-3f) {  // Significant approach velocity
                        restitution_term = e * v_approach;  // Note: v_approach is negative
                    }
                }
                
                const float rhs = -(jv + error_correction + restitution_term);

                // Compute inverse mass sum along constraint direction
                const float denominator = compute_jacobian_inverse_mass(jacobian, row, constraint->get_body_ids(), particles);
                if (denominator < 1e-6f) {
                    continue;  // Singular constraint
                }

                // Compute unclamped impulse
                const float impulse_unclamped = rhs / denominator;

                // Store old impulse for convergence check
                float old_accumulated = constraint->get_accumulated_impulse();

                // Clamp impulse based on constraint type:
                // - Unilateral constraints (contact): clamp to >= 0 (can only push)
                // - Bilateral constraints (fixed/joint): allow negative impulses
                const float impulse_clamped = constraint->is_unilateral() ?
                    (std::max(0.0f, old_accumulated + impulse_unclamped) - old_accumulated) :
                    impulse_unclamped;

                // Track max change for convergence
                max_impulse_change = std::max(max_impulse_change, std::abs(impulse_clamped));

                // Apply clamped impulse
                if (std::abs(impulse_clamped) > 1e-9f) {
                    constraint->apply_impulse(impulse_clamped);
                }
            }

            // Early exit if converged
            if (max_impulse_change < config_.convergence_threshold) {
                break;
            }
        }
    }

    // ========================================================================
    // Diagnostics & Statistics
    // ========================================================================

    /// Get last frame's average impulse magnitude (for debugging).
    float get_last_avg_impulse() const {
        return last_avg_impulse_;
    }

    /// Get last frame's iteration count.
    int get_last_iteration_count() const {
        return last_iteration_count_;
    }

private:
    // ========================================================================
    // Helper Methods
    // ========================================================================

    /// Compute J * v (Jacobian-velocity product) for a single constraint equation.
    /// This represents the rate of constraint violation.
    /// @param jacobian_row Jacobian row [jv_a.x, jv_a.y, jv_a.z, jv_b.x, jv_b.y, jv_b.z]
    /// @param body_ids Particle indices involved in constraint
    /// @param particles Reference to all particles
    /// @return -dot(jacobian_row, [v_a, v_b, ...])  (negative because we solve in positive direction)
    float compute_jacobian_velocity(
        const MatDynamic<float>& jacobian,
        size_t row_index,
        const std::vector<size_t>& body_ids,
        const std::vector<Particle>& particles
    ) const {
        float jv = 0.0f;
        size_t col_index = 0;

        for (size_t body_id : body_ids) {
            if (body_id >= particles.size()) {
                continue;
            }

            const Particle& p = particles[body_id];
            const Vec3f& v = p.velocity;

            // Dot product: jacobian[row, col:col+3] · v
            if (col_index < jacobian.numCols()) {
                jv += jacobian(row_index, col_index) * v.x;
            }
            if (col_index + 1 < jacobian.numCols()) {
                jv += jacobian(row_index, col_index + 1) * v.y;
            }
            if (col_index + 2 < jacobian.numCols()) {
                jv += jacobian(row_index, col_index + 2) * v.z;
            }

            col_index += 3;  // Move to next body (assumes 3 linear DOF per body)
        }

        return jv;
    }

    /// Compute J * M^-1 * J^T (effective mass denominator).
    /// This represents the resistance to impulse application.
    /// @param jacobian_row Jacobian row
    /// @param body_ids Particle indices
    /// @param particles Reference to all particles
    /// @return Effective mass (positive = solvable, zero = singular)
    float compute_jacobian_inverse_mass(
        const MatDynamic<float>& jacobian,
        size_t row_index,
        const std::vector<size_t>& body_ids,
        const std::vector<Particle>& particles
    ) const {
        float denominator = 0.0f;
        size_t col_index = 0;

        for (size_t body_id : body_ids) {
            if (body_id >= particles.size()) {
                continue;
            }

            const Particle& p = particles[body_id];
            float inv_mass = p.inverse_mass();

            // J_i * M^-1 * J_i^T = (J_i · J_i) * inv_mass
            if (col_index < jacobian.numCols()) {
                float jx = jacobian(row_index, col_index);
                float jy = jacobian(row_index, col_index + 1);
                float jz = jacobian(row_index, col_index + 2);
                float j_squared = jx * jx + jy * jy + jz * jz;
                denominator += j_squared * inv_mass;
            }

            col_index += 3;
        }

        return denominator;
    }

    // ========================================================================
    // Member Variables
    // ========================================================================

    ConstraintSolverConfig config_;
    float last_avg_impulse_ = 0.0f;
    int last_iteration_count_ = 0;
};

}  // namespace phynity::physics::constraints

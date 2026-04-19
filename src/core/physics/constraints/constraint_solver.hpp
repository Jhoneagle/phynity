#pragma once

#include <core/diagnostics/profiling_macros.hpp>
#include <core/physics/constraints/constraint.hpp>
#include <platform/allocation_tracker.hpp>

#include <algorithm>
#include <memory>
#include <vector>

namespace phynity::physics::constraints
{

/// Solver configuration for constraint solving
struct ConstraintSolverConfig
{
    int iterations = 4; ///< Number of solver iterations (higher = more stable but slower)
    float convergence_threshold = 1e-5f; ///< Convergence criterion (stop if impulse change < threshold)
    float baumgarte_beta = 0.2f; ///< Baumgarte penetration correction parameter (0.0-0.5)
    bool use_warm_start = true; ///< Whether to use warm-start from previous impulses
    bool enable_adaptive_iterations = true; ///< Increase iterations for large contact sets
};

/// Unified constraint solver using Projected Gauss-Seidel (PGS).
/// Each constraint carries its own body references and computes Jv / effective mass
/// internally — the solver orchestrates iteration, clamping, and convergence.
class ConstraintSolver
{
public:
    ConstraintSolver() = default;

    void set_config(const ConstraintSolverConfig &config)
    {
        config_ = config;
    }

    const ConstraintSolverConfig &config() const
    {
        return config_;
    }

    /// Solve all constraints in the system.
    /// @param constraints Vector of constraints to solve (contacts + joints)
    void solve(std::vector<std::unique_ptr<Constraint>> &constraints)
    {
        PROFILE_FUNCTION();

        // Filter out inactive constraints
        std::vector<Constraint *> active_constraints;
        for (auto &constraint : constraints)
        {
            if (constraint && constraint->is_active())
            {
                active_constraints.push_back(constraint.get());
            }
        }

        if (active_constraints.empty())
        {
            return;
        }

        // Determine iteration count
        int iterations = config_.iterations;
        if (config_.enable_adaptive_iterations)
        {
            const size_t constraint_count = active_constraints.size();
            if (constraint_count > 20)
            {
                iterations = std::min(iterations * 2, 16);
            }
        }

        // Phase 1: Apply warm-start impulses
        if (config_.use_warm_start)
        {
            PROFILE_SCOPE("warm_start");
            for (auto constraint : active_constraints)
            {
                float warm_start = constraint->get_accumulated_impulse();
                if (warm_start > 1e-6f)
                {
                    constraint->apply_impulse(warm_start);
                }
            }
        }

        // Phase 2: Iterative PGS solve
        float max_impulse_change = 0.0f;

        for (int iter = 0; iter < iterations; ++iter)
        {
            PROFILE_SCOPE("iteration");

            max_impulse_change = 0.0f;

            for (auto constraint : active_constraints)
            {
                float error = constraint->compute_error();
                float jv = constraint->compute_jv();
                float effective_mass = constraint->compute_effective_mass();

                if (effective_mass < 1e-6f)
                {
                    continue; // Singular constraint
                }

                // Baumgarte stabilization
                const float error_correction = config_.baumgarte_beta * error;

                // Restitution (first iteration only, for contacts)
                float restitution_term = 0.0f;
                if (constraint->is_unilateral() && iter == 0)
                {
                    float e = constraint->get_restitution();
                    float v_approach = constraint->get_initial_approach_velocity();
                    if (v_approach < -1e-3f)
                    {
                        restitution_term = e * v_approach;
                    }
                }

                const float rhs = -(jv + error_correction + restitution_term);
                const float impulse_unclamped = rhs / effective_mass;

                float old_accumulated = constraint->get_accumulated_impulse();

                // Clamp: unilateral (contact) can only push, bilateral can push or pull
                const float impulse_clamped =
                    constraint->is_unilateral()
                        ? (std::max(0.0f, old_accumulated + impulse_unclamped) - old_accumulated)
                        : impulse_unclamped;

                max_impulse_change = std::max(max_impulse_change, std::abs(impulse_clamped));

                if (std::abs(impulse_clamped) > 1e-9f)
                {
                    constraint->apply_impulse(impulse_clamped);
                }
            }

            if (max_impulse_change < config_.convergence_threshold)
            {
                break;
            }
        }
    }

    float get_last_avg_impulse() const
    {
        return last_avg_impulse_;
    }

    int get_last_iteration_count() const
    {
        return last_iteration_count_;
    }

private:
    ConstraintSolverConfig config_;
    float last_avg_impulse_ = 0.0f;
    int last_iteration_count_ = 0;
};

} // namespace phynity::physics::constraints

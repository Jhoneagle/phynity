#pragma once

#include <core/diagnostics/energy_monitor.hpp>
#include <core/diagnostics/momentum_monitor.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <core/jobs/task_executor.hpp>
#include <core/jobs/task_graph.hpp>
#include <core/jobs/task_graph_builder.hpp>
#include <core/jobs/task_scheduler.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/physics/config/ccd_config.hpp>
#include <core/physics/constraints/constraint.hpp>
#include <core/physics/dynamics/force_field.hpp>
#include <core/physics/dynamics/inertia.hpp>
#include <core/physics/rigid_bodies/rigid_body.hpp>
#include <core/physics/rigid_bodies/rigid_body_collision_resolver.hpp>
#include <platform/allocation_tracker.hpp>

#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

namespace phynity::physics
{

using phynity::physics::constraints::Constraint;

/// Manages a collection of rigid bodies, force fields, and constraints.
/// Handles 6-DOF rigid body dynamics with rotational integration,
/// collision detection, and constraint solving.
class RigidBodySystem
{
public:
    // ========================================================================
    // Configuration
    // ========================================================================

    struct Config
    {
        float gravity_acceleration = 9.81f; ///< Gravitational acceleration (m/s²)
        int broadphase_grid_resolution = 20; ///< Spatial grid cell count
        float broadphase_grid_size = 100.0f; ///< Total grid size
        float default_collision_radius = 1.0f; ///< Default broadphase radius for new bodies
        bool enable_linear_ccd = true; ///< Enable linear CCD for fast movers
        CCDConfig ccd_config = ccd_presets::balanced(); ///< CCD thresholds and parameters
        int constraint_iterations = 8; ///< PGS solver iterations for constraints
        float baumgarte_beta = 0.2f; ///< Baumgarte penetration/error correction factor (0.0-0.5)
        float max_constraint_impulse = 100.0f; ///< Maximum impulse per constraint per iteration

        constexpr Config() = default;
    };

    /// Constructor with default configuration
    explicit RigidBodySystem() : RigidBodySystem(Config{})
    {
    }

    /// Constructor with custom configuration
    explicit RigidBodySystem(const Config &config)
        : config_(config),
          collision_resolver_(config.broadphase_grid_size / static_cast<float>(config.broadphase_grid_resolution)),
          next_body_id_(0)
    {
    }

    ~RigidBodySystem()
    {
        phynity::platform::track_vector_capacity_release(bodies_);
        phynity::platform::track_vector_capacity_release(force_fields_);
        phynity::platform::track_vector_capacity_release(constraints_);
    }

    RigidBodySystem(RigidBodySystem &&other) noexcept = default;
    RigidBodySystem &operator=(RigidBodySystem &&other) noexcept = default;

    // ========================================================================
    // Body Management
    // ========================================================================

    /// Spawn a new rigid body
    /// @param position Starting position in world space
    /// @param orientation Starting orientation as quaternion
    /// @param shape Collision shape (sphere, box, capsule)
    /// @param mass Mass in kilograms
    /// @param material Physical properties (restitution, friction)
    /// @return ID of the created body
    RigidBodyID spawn_body(const Vec3f &position,
                           const Quatf &orientation,
                           std::shared_ptr<Shape> shape,
                           float mass = 1.0f,
                           const Material &material = Material{})
    {
        PROFILE_SCOPE("RigidBodySystem::spawn_body");

        const size_t previous_capacity = bodies_.capacity();
        bodies_.emplace_back();
        phynity::platform::track_vector_capacity_change(bodies_, previous_capacity);
        RigidBody &rb = bodies_.back();

        rb.position = position;
        rb.orientation = orientation;
        rb.shape = shape;
        rb.material = material;
        rb.id = next_body_id_;
        rb.collision_radius = (shape) ? shape->get_bounding_radius() : config_.default_collision_radius;

        rb.set_mass(mass);

        body_index_[next_body_id_] = bodies_.size() - 1;
        return next_body_id_++;
    }

    /// Get a rigid body by ID (O(1) lookup via index map)
    RigidBody *get_body(RigidBodyID id)
    {
        auto it = body_index_.find(id);
        if (it == body_index_.end() || it->second >= bodies_.size())
        {
            return nullptr;
        }
        return &bodies_[it->second];
    }

    /// Get total number of active bodies
    size_t body_count() const
    {
        return bodies_.size();
    }

    /// Direct body storage access for serialization/helpers.
    std::vector<RigidBody> &bodies()
    {
        return bodies_;
    }

    const std::vector<RigidBody> &bodies() const
    {
        return bodies_;
    }

    /// Remove all rigid bodies from the system.
    void clear_bodies()
    {
        bodies_.clear();
        body_index_.clear();
        next_body_id_ = 0;
    }

    /// Adjust next ID after external restoration logic sets explicit IDs.
    void set_next_body_id(RigidBodyID next_id)
    {
        next_body_id_ = next_id;
    }

    // ========================================================================
    // Force Fields
    // ========================================================================

    /// Add a force field (gravity, drag, wind, etc.)
    void add_force_field(std::unique_ptr<ForceField> field)
    {
        const size_t previous_capacity = force_fields_.capacity();
        force_fields_.push_back(std::move(field));
        phynity::platform::track_vector_capacity_change(force_fields_, previous_capacity);
    }

    /// Remove all force fields
    void clear_force_fields()
    {
        force_fields_.clear();
    }

    // ========================================================================
    // Constraints
    // ========================================================================

    /// Register a constraint (fixed joint, hinge, etc.)
    void add_constraint(std::unique_ptr<Constraint> constraint)
    {
        const size_t previous_capacity = constraints_.capacity();
        constraints_.push_back(std::move(constraint));
        phynity::platform::track_vector_capacity_change(constraints_, previous_capacity);
    }

    /// Get all constraints
    const std::vector<std::unique_ptr<Constraint>> &get_constraints() const
    {
        return constraints_;
    }

    // ========================================================================
    // Job System Integration
    // ========================================================================

    /// Provide a task executor for structured parallel update using task graphs.
    void set_task_executor(phynity::jobs::TaskExecutor *executor)
    {
        task_executor_ = executor;
    }

    // ========================================================================
    // Simulation Step
    // ========================================================================

    /// Advance simulation by dt seconds
    void update(float dt)
    {
        PROFILE_SCOPE("RigidBodySystem::update");

        if (bodies_.empty())
        {
            return;
        }

        // Task-graph path: structured wavefront dispatch with cache affinity
        if (task_executor_)
        {
            update_with_task_graph(dt);
            return;
        }

        std::vector<Vec3f> frame_start_positions;
        frame_start_positions.reserve(bodies_.size());
        for (const auto &body : bodies_)
        {
            frame_start_positions.push_back(body.position);
        }

        // Phase 1: Clear forces and torques
        {
            PROFILE_SCOPE("RigidBodySystem::clear_forces");
            for (auto &rb : bodies_)
            {
                rb.clear_forces_and_torques();
            }
        }

        // Phase 2: Apply force fields (gravity, drag, wind, etc.)
        {
            PROFILE_SCOPE("RigidBodySystem::apply_forces");
            for (auto &field : force_fields_)
            {
                for (auto &rb : bodies_)
                {
                    Vec3f force = field->apply(rb.position, rb.velocity, rb.get_mass());
                    rb.force_accumulator += force;
                }
            }
        }

        // Phase 3: Linear integration (semi-implicit Euler)
        {
            PROFILE_SCOPE("RigidBodySystem::linear_integration");
            for (auto &rb : bodies_)
            {
                if (rb.is_static())
                    continue;

                // v_new = v + (F/m) * dt
                Vec3f acceleration = rb.force_accumulator * rb.inv_mass;
                rb.velocity += acceleration * dt;

                // Apply linear damping
                rb.velocity *= (1.0f - rb.material.linear_damping * dt);

                // p_new = p + v_new * dt
                rb.position += rb.velocity * dt;
            }
        }

        // Phase 4: Angular integration (quaternion + omega)
        {
            PROFILE_SCOPE("RigidBodySystem::angular_integration");
            for (auto &rb : bodies_)
            {
                if (rb.is_static())
                    continue;

                // ω_new = ω + (I^-1 * τ) * dt
                Vec3f angular_accel = rb.inertia_tensor_inv * rb.torque_accumulator;
                rb.angular_velocity += angular_accel * dt;

                // Apply angular damping
                rb.angular_velocity *= (1.0f - rb.material.angular_damping * dt);

                // q_new = q + 0.5 * dt * ω_quat * q
                rb.orientation =
                    phynity::physics::inertia::integrate_quaternion(rb.orientation, rb.angular_velocity, dt);
                rb.orientation.normalize();
            }
        }

        // Phase 5: Collision detection
        {
            PROFILE_SCOPE("RigidBodySystem::collisions");
            RigidBodyCollisionConfig collision_config;
            collision_config.default_collision_radius = config_.default_collision_radius;
            collision_config.enable_linear_ccd = config_.enable_linear_ccd;
            collision_config.ccd_config = config_.ccd_config;
            collision_resolver_.resolve(bodies_, frame_start_positions, dt, collision_config);
        }

        // Phase 6: Constraint solving (PGS-style iterative solver for rigid body constraints)
        if (!constraints_.empty())
        {
            PROFILE_SCOPE("RigidBodySystem::constraint_solving");

            // Collect active constraints (reuse member to avoid per-frame allocation)
            active_constraints_cache_.clear();
            for (auto &c : constraints_)
            {
                if (c && c->is_active())
                {
                    active_constraints_cache_.push_back(c.get());
                }
            }

            if (!active_constraints_cache_.empty())
            {
                const int iterations = config_.constraint_iterations;
                const float beta = config_.baumgarte_beta;
                const float convergence_threshold = 1e-5f;

                for (int iter = 0; iter < iterations; ++iter)
                {
                    float max_impulse = 0.0f;

                    for (auto *constraint : active_constraints_cache_)
                    {
                        float error = constraint->compute_error();
                        if (error < convergence_threshold)
                        {
                            continue;
                        }

                        // Scale impulse by dt to convert positional error into
                        // a velocity correction appropriate for one timestep
                        float impulse = std::min(beta * error * dt, config_.max_constraint_impulse);

                        constraint->apply_impulse(impulse);
                        max_impulse = std::max(max_impulse, std::abs(impulse));
                    }

                    if (max_impulse < convergence_threshold)
                    {
                        break;
                    }
                }
            }
        }

        // Phase 7: Update lifetimes and cleanup
        {
            PROFILE_SCOPE("RigidBodySystem::cleanup");
            for (auto &rb : bodies_)
            {
                rb.update_lifetime(dt);
            }
            const size_t size_before = bodies_.size();
            bodies_.erase(
                std::remove_if(bodies_.begin(), bodies_.end(), [](const RigidBody &rb) { return !rb.active; }),
                bodies_.end());

            // Rebuild index map only if removal shifted elements
            if (bodies_.size() != size_before)
            {
                body_index_.clear();
                for (size_t idx = 0; idx < bodies_.size(); ++idx)
                {
                    body_index_[bodies_[idx].id] = idx;
                }
            }
        }

        // Phase 8: Update diagnostics
        {
            PROFILE_SCOPE("RigidBodySystem::diagnostics");
            compute_diagnostics();
        }
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Diagnostic statistics about the system
    struct Diagnostics
    {
        float total_kinetic_energy = 0.0f; ///< Linear + angular KE
        float total_linear_ke = 0.0f;
        float total_angular_ke = 0.0f;
        Vec3f total_momentum = Vec3f(0.0f); ///< Linear momentum (m*v)
        Vec3f total_angular_momentum = Vec3f(0.0f); ///< Angular momentum (I*ω)
        size_t body_count = 0;
    };

    /// Get current diagnostics
    const Diagnostics &get_diagnostics() const
    {
        return diagnostics_;
    }

private:
    // Collision detection and resolution is delegated to RigidBodyCollisionResolver.

    /// Recompute system-wide diagnostics
    void compute_diagnostics()
    {
        diagnostics_ = Diagnostics{};
        diagnostics_.body_count = bodies_.size();

        for (const auto &rb : bodies_)
        {
            // Linear kinetic energy
            float linear_ke = 0.5f * rb.get_mass() * rb.velocity.squaredLength();
            diagnostics_.total_linear_ke += linear_ke;

            // Angular kinetic energy
            float angular_ke =
                phynity::physics::inertia::compute_angular_kinetic_energy(rb.inertia_tensor, rb.angular_velocity);
            diagnostics_.total_angular_ke += angular_ke;

            // Total kinetic energy
            diagnostics_.total_kinetic_energy += linear_ke + angular_ke;

            // Linear momentum
            diagnostics_.total_momentum += rb.momentum();

            // Angular momentum
            diagnostics_.total_angular_momentum += rb.angular_momentum();
        }
    }

    // ========================================================================
    // Member Variables
    // ========================================================================

    void update_with_task_graph(float dt)
    {
        using namespace phynity::jobs;

        const uint32_t count = static_cast<uint32_t>(bodies_.size());
        const uint32_t partitions = std::min(count, 4u);
        const bool has_constraints = !constraints_.empty();

        // Invalidate cached schedule when topology changes
        if (partitions != cached_partition_count_ || has_constraints != cached_has_constraints_)
        {
            cached_schedule_.reset();
        }

        // Save start positions for CCD (reused member vector avoids per-frame allocation)
        frame_start_positions_.clear();
        frame_start_positions_.reserve(count);
        for (const auto &body : bodies_)
            frame_start_positions_.push_back(body.position);

        TaskGraph graph;

        auto clear_ids = add_partitioned_tier(
            graph,
            count,
            partitions,
            {},
            [this](uint32_t s, uint32_t e) -> JobFn
            {
                return [this, s, e]
                {
                    for (uint32_t i = s; i < e; ++i)
                        bodies_[i].clear_forces_and_torques();
                };
            },
            "rb_clear");

        auto forces_ids = add_partitioned_tier(
            graph,
            count,
            partitions,
            clear_ids,
            [this](uint32_t s, uint32_t e) -> JobFn
            {
                return [this, s, e]
                {
                    for (auto &field : force_fields_)
                        for (uint32_t i = s; i < e; ++i)
                        {
                            Vec3f force = field->apply(bodies_[i].position, bodies_[i].velocity, bodies_[i].get_mass());
                            bodies_[i].force_accumulator += force;
                        }
                };
            },
            "rb_forces");

        auto linear_ids = add_partitioned_tier(
            graph,
            count,
            partitions,
            forces_ids,
            [this, dt](uint32_t s, uint32_t e) -> JobFn
            {
                return [this, s, e, dt]
                {
                    for (uint32_t i = s; i < e; ++i)
                    {
                        auto &rb = bodies_[i];
                        if (rb.is_static())
                            continue;
                        rb.velocity += (rb.force_accumulator * rb.inv_mass) * dt;
                        rb.velocity *= (1.0f - rb.material.linear_damping * dt);
                        rb.position += rb.velocity * dt;
                    }
                };
            },
            "rb_linear");

        auto angular_ids = add_partitioned_tier(
            graph,
            count,
            partitions,
            linear_ids,
            [this, dt](uint32_t s, uint32_t e) -> JobFn
            {
                return [this, s, e, dt]
                {
                    for (uint32_t i = s; i < e; ++i)
                    {
                        auto &rb = bodies_[i];
                        if (rb.is_static())
                            continue;
                        rb.angular_velocity += (rb.inertia_tensor_inv * rb.torque_accumulator) * dt;
                        rb.angular_velocity *= (1.0f - rb.material.angular_damping * dt);
                        rb.orientation =
                            phynity::physics::inertia::integrate_quaternion(rb.orientation, rb.angular_velocity, dt);
                        rb.orientation.normalize();
                    }
                };
            },
            "rb_angular");

        auto collision_id = add_serial_task_after(
            graph,
            angular_ids,
            [this, dt]
            {
                RigidBodyCollisionConfig cc;
                cc.default_collision_radius = config_.default_collision_radius;
                cc.enable_linear_ccd = config_.enable_linear_ccd;
                cc.ccd_config = config_.ccd_config;
                collision_resolver_.resolve(bodies_, frame_start_positions_, dt, cc);
            },
            "rb_collisions");

        if (!constraints_.empty())
        {
            auto constraint_id =
                graph.add_task({.fn =
                                    [this, dt]
                                {
                                    active_constraints_cache_.clear();
                                    for (auto &c : constraints_)
                                        if (c && c->is_active())
                                            active_constraints_cache_.push_back(c.get());

                                    if (active_constraints_cache_.empty())
                                        return;

                                    const int iterations = config_.constraint_iterations;
                                    const float beta = config_.baumgarte_beta;
                                    const float threshold = 1e-5f;

                                    for (int iter = 0; iter < iterations; ++iter)
                                    {
                                        float max_impulse = 0.0f;
                                        for (auto *constraint : active_constraints_cache_)
                                        {
                                            float error = constraint->compute_error();
                                            if (error < threshold)
                                                continue;
                                            float impulse = std::min(beta * error * dt, config_.max_constraint_impulse);
                                            constraint->apply_impulse(impulse);
                                            max_impulse = std::max(max_impulse, std::abs(impulse));
                                        }
                                        if (max_impulse < threshold)
                                            break;
                                    }
                                },
                                .debug_name = "rb_constraints"});
            graph.add_dependency(collision_id, constraint_id);
        }

        // Use cached schedule when topology is stable (skip validate + Kahn's)
        if (!cached_schedule_)
        {
            cached_schedule_ = TaskScheduler::build_schedule(graph);
            cached_partition_count_ = partitions;
            cached_has_constraints_ = has_constraints;
        }

        task_executor_->execute(*cached_schedule_, graph);

        // Post-graph serial: cleanup and diagnostics
        for (auto &rb : bodies_)
            rb.update_lifetime(dt);

        const size_t size_before = bodies_.size();
        bodies_.erase(std::remove_if(bodies_.begin(), bodies_.end(), [](const RigidBody &rb) { return !rb.active; }),
                      bodies_.end());

        if (bodies_.size() != size_before)
        {
            body_index_.clear();
            for (size_t idx = 0; idx < bodies_.size(); ++idx)
                body_index_[bodies_[idx].id] = idx;
        }

        compute_diagnostics();
    }

    Config config_;
    std::vector<RigidBody> bodies_;
    std::unordered_map<RigidBodyID, size_t> body_index_;
    std::vector<std::unique_ptr<ForceField>> force_fields_;
    std::vector<std::unique_ptr<Constraint>> constraints_;
    std::vector<Constraint *> active_constraints_cache_;
    RigidBodyCollisionResolver collision_resolver_;
    std::vector<Vec3f> frame_start_positions_; // reused per frame to avoid allocation
    RigidBodyID next_body_id_;
    Diagnostics diagnostics_;
    phynity::jobs::TaskExecutor *task_executor_ = nullptr;

    // Cached task schedule (topology-stable across frames)
    std::optional<phynity::jobs::TaskSchedule> cached_schedule_;
    uint32_t cached_partition_count_ = 0;
    bool cached_has_constraints_ = false;
};

} // namespace phynity::physics

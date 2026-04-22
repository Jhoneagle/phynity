#pragma once

#include <core/diagnostics/physics_diagnostics_hub.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <core/jobs/job_system.hpp>
#include <core/jobs/task_executor.hpp>
#include <core/jobs/task_graph.hpp>
#include <core/jobs/task_graph_builder.hpp>
#include <core/jobs/task_scheduler.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/physics/config/ccd_config.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/constraints/constraint.hpp>
#include <core/physics/constraints/constraint_solver.hpp>
#include <core/physics/constraints/fixed_joint.hpp>
#include <core/physics/dynamics/force_field.hpp>
#include <core/physics/particles/particle.hpp>
#include <core/physics/particles/particle_collision_resolver.hpp>
#include <platform/allocation_tracker.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

namespace phynity::physics
{

/// Manages a collection of particles and force fields, providing simulation stepping.
/// Integrates Material system, ForceField system, and provides energy/momentum diagnostics.
class ParticleSystem
{
public:
    /// Diagnostic information about the particle system
    struct Diagnostics
    {
        float total_kinetic_energy = 0.0f; ///< Sum of all particle kinetic energies
        Vec3f total_momentum = Vec3f(0.0f); ///< Sum of all particle momenta (mass * velocity)
        size_t particle_count = 0; ///< Number of active particles
    };

    ParticleSystem() = default;
    ~ParticleSystem()
    {
        phynity::platform::track_vector_capacity_release(particles_);
        phynity::platform::track_vector_capacity_release(force_fields_);
        phynity::platform::track_vector_capacity_release(constraints_);
    }


    // Move semantics
    ParticleSystem(ParticleSystem &&other) noexcept = default;
    ParticleSystem &operator=(ParticleSystem &&other) noexcept = default;

    // ========================================================================
    // Particle Management
    // ========================================================================

    /// Spawn a new particle at position with initial velocity.
    /// @param position Starting position
    /// @param velocity Starting velocity
    /// @param mass Particle mass (default: 1.0)
    /// @param lifetime Particle lifetime (-1 = infinite, > 0 = finite)
    void
    spawn(const Vec3f &position, const Vec3f &velocity, float mass = 1.0f, float lifetime = -1.0f, float radius = -1.0f)
    {
        const size_t previous_capacity = particles_.capacity();
        particles_.emplace_back();
        phynity::platform::track_vector_capacity_change(particles_, previous_capacity);
        Particle &p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material.mass = mass;
        p.lifetime = lifetime;
        p.radius = (radius > 0.0f) ? radius : default_collision_radius_;
        p.id = static_cast<int>(particles_.size() - 1);
    }

    /// Spawn a new particle with full material specification.
    /// @param position Starting position
    /// @param velocity Starting velocity
    /// @param material Complete material definition
    /// @param lifetime Particle lifetime (-1 = infinite, > 0 = finite)
    void spawn(const Vec3f &position,
               const Vec3f &velocity,
               const Material &material,
               float lifetime = -1.0f,
               float radius = -1.0f)
    {
        const size_t previous_capacity = particles_.capacity();
        particles_.emplace_back();
        phynity::platform::track_vector_capacity_change(particles_, previous_capacity);
        Particle &p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material = material;
        p.lifetime = lifetime;
        p.radius = (radius > 0.0f) ? radius : default_collision_radius_;
        p.id = static_cast<int>(particles_.size() - 1);
    }

    /// Clear all particles.
    void clear()
    {
        particles_.clear();
    }

    /// Remove dead particles from the system.
    /// WARNING: This invalidates any outstanding iterators or references to particles.
    /// Do not keep references to particles obtained via particles() before calling this method.
    /// The internal particle storage may be reordered or resized during removal.
    void remove_dead_particles()
    {
        particles_.erase(
            std::remove_if(particles_.begin(), particles_.end(), [](const Particle &p) { return !p.is_alive(); }),
            particles_.end());
    }

    // ========================================================================
    // Force Field Management
    // ========================================================================

    /// Add a force field to the system.
    /// The system takes ownership of the field.
    /// @param field Unique pointer to force field
    void add_force_field(std::unique_ptr<ForceField> field)
    {
        const size_t previous_capacity = force_fields_.capacity();
        force_fields_.push_back(std::move(field));
        phynity::platform::track_vector_capacity_change(force_fields_, previous_capacity);
    }

    /// Remove all force fields from the system.
    void clear_force_fields()
    {
        force_fields_.clear();
    }

    /// Get the number of active force fields.
    size_t force_field_count() const
    {
        return force_fields_.size();
    }

    // ========================================================================
    // Collision Management
    // ========================================================================

    /// Enable or disable simple sphere-sphere collision handling.
    void enable_collisions(bool enabled)
    {
        collisions_enabled_ = enabled;
    }

    /// Check whether collisions are enabled.
    bool collisions_enabled() const
    {
        return collisions_enabled_;
    }

    /// Set default collision radius used by spawn() when radius is not specified.
    void set_default_collision_radius(float radius)
    {
        if (radius > 0.0f)
        {
            default_collision_radius_ = radius;
        }
    }

    /// Get the current default collision radius.
    float default_collision_radius() const
    {
        return default_collision_radius_;
    }

    /// Set the broadphase grid cell size (in world units).
    /// Larger cells = fewer cells, faster insertion but more candidates per query.
    /// Smaller cells = more cells, slower insertion but fewer candidates.
    /// Recommended: 2x to 4x the average particle diameter.
    void set_broadphase_cell_size(float cell_size)
    {
        collision_resolver_.set_cell_size(cell_size);
    }

    /// Get the current broadphase grid cell size.
    float broadphase_cell_size() const
    {
        return collision_resolver_.cell_size();
    }

    // ========================================================================
    // Constraint Management (Phase 5: Constraint Framework)
    // ========================================================================

    /// Add a rigid constraint to the system (e.g., fixed joint, hinge).
    /// The system takes ownership of the constraint.
    /// @param constraint Unique pointer to constraint
    void add_constraint(std::unique_ptr<constraints::Constraint> constraint)
    {
        if (constraint)
        {
            const size_t previous_capacity = constraints_.capacity();
            constraints_.push_back(std::move(constraint));
            phynity::platform::track_vector_capacity_change(constraints_, previous_capacity);
        }
    }

    /// Create and add a fixed constraint between two particles.
    /// @param particle_a_index Index of first particle
    /// @param particle_b_index Index of second particle
    /// @return Pointer to the created constraint (for reference/modification)
    constraints::DistanceJoint *add_fixed_constraint(size_t particle_a_index, size_t particle_b_index)
    {
        if (particle_a_index >= particles_.size() || particle_b_index >= particles_.size())
        {
            return nullptr;
        }

        auto constraint =
            std::make_unique<constraints::DistanceJoint>(particles_[particle_a_index], particles_[particle_b_index]);
        auto *ptr = constraint.get();
        const size_t previous_capacity = constraints_.capacity();
        constraints_.push_back(std::move(constraint));
        phynity::platform::track_vector_capacity_change(constraints_, previous_capacity);
        return ptr;
    }

    /// Remove all constraints from the system.
    void clear_constraints()
    {
        constraints_.clear();
    }

    /// Get the number of active constraints.
    size_t constraint_count() const
    {
        return constraints_.size();
    }

    /// Enable or disable constraint solving.
    /// When disabled, only collision constraints are used.
    void enable_constraints(bool enabled)
    {
        constraints_enabled_ = enabled;
    }

    /// Check whether constraint solving is enabled.
    bool constraints_enabled() const
    {
        return constraints_enabled_;
    }

    /// Set constraint solver configuration.
    void set_constraint_solver_config(const constraints::ConstraintSolverConfig &config)
    {
        constraint_solver_.set_config(config);
    }

    /// Get constraint solver configuration.
    const constraints::ConstraintSolverConfig &constraint_solver_config() const
    {
        return constraint_solver_.config();
    }

    // ========================================================================
    // Continuous Collision Detection (CCD) Configuration
    // ========================================================================

    /// Set continuous collision detection configuration.
    /// Controls when CCD is triggered and how many substeps are used.
    /// @param config CCD configuration struct
    void set_ccd_config(const CCDConfig &config)
    {
        ccd_config_ = config;
    }

    /// Get current continuous collision detection configuration.
    const CCDConfig &ccd_config() const
    {
        return ccd_config_;
    }

    /// Enable or disable CCD globally.
    /// @param enabled True to enable CCD, false to disable
    void set_ccd_enabled(bool enabled)
    {
        ccd_config_.enabled = enabled;
    }

    /// Check if CCD is currently enabled.
    bool is_ccd_enabled() const
    {
        return ccd_config_.enabled;
    }

    /// Set the velocity threshold for CCD triggering.
    /// Objects moving faster than this will use CCD.
    /// @param threshold Minimum speed in m/s (0.0 = always use CCD)
    void set_ccd_velocity_threshold(float threshold)
    {
        ccd_config_.velocity_threshold = threshold;
    }

    // ========================================================================
    // Simulation Update
    // ========================================================================

    /// Update the simulation by one timestep.
    /// Order of operations:
    /// 1. Clear all particle forces
    /// 2. Apply each force field to each particle
    /// 3. Update particle accelerations from accumulated forces
    /// 4. Integrate particle positions/velocities
    /// 5. Remove dead particles
    /// @param dt Time step in seconds
    void update(float dt)
    {
        PROFILE_FUNCTION();

        // Task-graph path: structured wavefront dispatch with cache affinity
        if (task_executor_ && !particles_.empty())
        {
            update_with_task_graph(dt);
            return;
        }

        auto for_each_alive = [this](auto &&fn)
        {
            const size_t count = particles_.size();
            if (job_system_ && job_system_->is_running())
            {
                job_system_->parallel_for(0,
                                          static_cast<uint32_t>(count),
                                          1,
                                          [&](uint32_t i)
                                          {
                                              Particle &p = particles_[i];
                                              if (p.is_alive())
                                              {
                                                  fn(p);
                                              }
                                          });
            }
            else
            {
                for (auto &p : particles_)
                {
                    if (p.is_alive())
                    {
                        fn(p);
                    }
                }
            }
        };

        // Step 1: Clear forces from previous frame
        {
            PROFILE_SCOPE("clear_forces");
            for_each_alive([](Particle &p) { p.clear_forces(); });
        }

        // Step 2: Apply all force fields to all particles
        {
            PROFILE_SCOPE("apply_force_fields");
            for (const auto &field : force_fields_)
            {
                if (job_system_ && job_system_->is_running())
                {
                    const size_t count = particles_.size();
                    job_system_->parallel_for(0,
                                              static_cast<uint32_t>(count),
                                              1,
                                              [&](uint32_t i)
                                              {
                                                  Particle &p = particles_[i];
                                                  if (!p.is_alive())
                                                  {
                                                      return;
                                                  }
                                                  Vec3f force = field->apply(p.position, p.velocity, p.material.mass);
                                                  p.apply_force(force);
                                              });
                }
                else
                {
                    for (auto &p : particles_)
                    {
                        if (p.is_alive())
                        {
                            Vec3f force = field->apply(p.position, p.velocity, p.material.mass);
                            p.apply_force(force);
                        }
                    }
                }
            }
        }

        // Step 3: Update accelerations from accumulated forces
        {
            PROFILE_SCOPE("update_accelerations");
            for_each_alive([](Particle &p) { p.update_acceleration(); });
        }

        // Step 4: Integrate particle state
        {
            PROFILE_SCOPE("integration");
            for_each_alive([dt](Particle &p) { p.integrate(dt); });
        }

        // Step 5: Resolve collisions (optional)
        if (collisions_enabled_)
        {
            PROFILE_SCOPE("collision_resolution");
            ParticleCollisionContext ctx{constraint_solver_, constraints_, constraints_enabled_};
            collision_resolver_.resolve(particles_, dt, ccd_config_, ctx);

            if (diagnostics_hub_.has_active_collision_monitor())
            {
                const auto &stats = collision_resolver_.last_stats();
                diagnostics_hub_.report_collisions(
                    stats.broadphase_candidates, stats.narrowphase_tests, stats.actual_collisions);
            }
        }

        // Step 6: Monitor physics (energy, momentum)
        if (diagnostics_hub_.has_active_physics_monitors())
        {
            PROFILE_SCOPE("physics_monitoring");
            const Diagnostics diag = compute_diagnostics();
            diagnostics_hub_.report_physics(diag.total_kinetic_energy, diag.total_momentum);
        }

        // Step 7: Remove dead particles
        {
            PROFILE_SCOPE("remove_dead_particles");
            remove_dead_particles();
        }
    }

    /// Legacy step method for backwards compatibility.
    /// @deprecated Use update() instead
    [[deprecated("Use update() instead")]] void step(float dt)
    {
        update(dt);
    }

    /// Apply gravity to all particles (legacy method).
    /// @deprecated Add a GravityField instead
    [[deprecated("Add a GravityField instead")]] void applyGravity(const Vec3f &gravity)
    {
        for (auto &p : particles_)
        {
            if (p.is_alive())
            {
                p.apply_force(gravity * p.material.mass);
            }
        }
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Compute current system diagnostics (energy, momentum).
    /// @return Diagnostics struct with current values
    Diagnostics compute_diagnostics() const
    {
        Diagnostics diag;
        diag.particle_count = particles_.size();
        diag.total_kinetic_energy = 0.0f;
        diag.total_momentum = Vec3f(0.0f);

        for (const auto &p : particles_)
        {
            if (p.is_alive())
            {
                diag.total_kinetic_energy += p.kinetic_energy();
                diag.total_momentum += p.velocity * p.material.mass;
            }
        }

        return diag;
    }

    // ========================================================================
    // Physics Monitoring (Optional)
    // ========================================================================

    void enable_energy_monitor(std::shared_ptr<diagnostics::EnergyMonitor> monitor)
    {
        diagnostics_hub_.enable_energy_monitor(monitor);
    }

    void disable_energy_monitor()
    {
        diagnostics_hub_.disable_energy_monitor();
    }

    void enable_momentum_monitor(std::shared_ptr<diagnostics::MomentumMonitor> monitor)
    {
        diagnostics_hub_.enable_momentum_monitor(monitor);
    }

    void disable_momentum_monitor()
    {
        diagnostics_hub_.disable_momentum_monitor();
    }

    void enable_collision_monitor(std::shared_ptr<diagnostics::CollisionMonitor> monitor)
    {
        diagnostics_hub_.enable_collision_monitor(monitor);
    }

    void disable_collision_monitor()
    {
        diagnostics_hub_.disable_collision_monitor();
    }

    // ========================================================================
    // Job System Integration
    // ========================================================================

    /// Provide a job system for optional parallel update passes.
    /// The system is not owned by ParticleSystem.
    void set_job_system(phynity::jobs::JobSystem *job_system)
    {
        job_system_ = job_system;
    }

    /// Provide a task executor for structured parallel update using task graphs.
    /// When set, update() uses wavefront-based dispatch instead of ad-hoc parallel_for.
    /// The executor is not owned by ParticleSystem.
    void set_task_executor(phynity::jobs::TaskExecutor *executor)
    {
        task_executor_ = executor;
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Get particle count.
    size_t particleCount() const
    {
        return particles_.size();
    }

    /// Get all particles (const access).
    const std::vector<Particle> &particles() const
    {
        return particles_;
    }

    /// Get all particles (mutable access).
    std::vector<Particle> &particles()
    {
        return particles_;
    }

private:
    void update_with_task_graph(float dt)
    {
        using namespace phynity::jobs;

        const uint32_t count = static_cast<uint32_t>(particles_.size());
        const uint32_t partitions = std::min(count, 4u);

        // Invalidate cached schedule when topology changes
        if (partitions != cached_partition_count_ || collisions_enabled_ != cached_collisions_enabled_)
        {
            cached_schedule_.reset();
        }

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
                        if (particles_[i].is_alive())
                            particles_[i].clear_forces();
                };
            },
            "clear_forces");

        auto forces_ids = add_partitioned_tier(
            graph,
            count,
            partitions,
            clear_ids,
            [this](uint32_t s, uint32_t e) -> JobFn
            {
                return [this, s, e]
                {
                    for (const auto &field : force_fields_)
                        for (uint32_t i = s; i < e; ++i)
                        {
                            Particle &p = particles_[i];
                            if (!p.is_alive())
                                continue;
                            p.apply_force(field->apply(p.position, p.velocity, p.material.mass));
                        }
                };
            },
            "apply_forces");

        auto accel_ids = add_partitioned_tier(
            graph,
            count,
            partitions,
            forces_ids,
            [this](uint32_t s, uint32_t e) -> JobFn
            {
                return [this, s, e]
                {
                    for (uint32_t i = s; i < e; ++i)
                        if (particles_[i].is_alive())
                            particles_[i].update_acceleration();
                };
            },
            "update_accel");

        auto integrate_ids = add_partitioned_tier(
            graph,
            count,
            partitions,
            accel_ids,
            [this, dt](uint32_t s, uint32_t e) -> JobFn
            {
                return [this, s, e, dt]
                {
                    for (uint32_t i = s; i < e; ++i)
                        if (particles_[i].is_alive())
                            particles_[i].integrate(dt);
                };
            },
            "integrate");

        if (collisions_enabled_)
        {
            add_serial_task_after(
                graph,
                integrate_ids,
                [this, dt]
                {
                    ParticleCollisionContext ctx{constraint_solver_, constraints_, constraints_enabled_};
                    collision_resolver_.resolve(particles_, dt, ccd_config_, ctx);
                    if (diagnostics_hub_.has_active_collision_monitor())
                    {
                        const auto &stats = collision_resolver_.last_stats();
                        diagnostics_hub_.report_collisions(
                            stats.broadphase_candidates, stats.narrowphase_tests, stats.actual_collisions);
                    }
                },
                "collisions");
        }

        // Use cached schedule when topology is stable (skip validate + Kahn's)
        if (!cached_schedule_)
        {
            cached_schedule_ = TaskScheduler::build_schedule(graph);
            cached_partition_count_ = partitions;
            cached_collisions_enabled_ = collisions_enabled_;
        }

        task_executor_->execute(*cached_schedule_, graph);

        if (diagnostics_hub_.has_active_physics_monitors())
        {
            const Diagnostics diag = compute_diagnostics();
            diagnostics_hub_.report_physics(diag.total_kinetic_energy, diag.total_momentum);
        }
        remove_dead_particles();
    }

    std::vector<Particle> particles_;
    std::vector<std::unique_ptr<ForceField>> force_fields_;
    phynity::jobs::JobSystem *job_system_ = nullptr;
    phynity::jobs::TaskExecutor *task_executor_ = nullptr;
    ParticleCollisionResolver collision_resolver_{2.0f};
    bool collisions_enabled_ = false;
    float default_collision_radius_ = 0.5f;

    // Continuous Collision Detection (CCD) configuration
    CCDConfig ccd_config_;

    // Constraint framework (Phase 5: Unified Constraint Solving)
    std::vector<std::unique_ptr<constraints::Constraint>> constraints_;
    constraints::ConstraintSolver constraint_solver_;
    bool constraints_enabled_ = true;

    // Diagnostics monitoring (energy, momentum, collision stats)
    diagnostics::PhysicsDiagnosticsHub diagnostics_hub_;

    // Cached task schedule (topology-stable across frames)
    std::optional<phynity::jobs::TaskSchedule> cached_schedule_;
    uint32_t cached_partition_count_ = 0;
    bool cached_collisions_enabled_ = false;
};

} // namespace phynity::physics

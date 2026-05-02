#pragma once

#include <core/jobs/job_system.hpp>
#include <core/jobs/schedule_recorder.hpp>
#include <core/jobs/schedule_replayer.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/config/timestep_controller.hpp>
#include <core/physics/dynamics/force_field.hpp>
#include <core/physics/dynamics/material.hpp>
#include <core/physics/particles/particle_system.hpp>
#include <core/physics/rigid_bodies/rigid_body_system.hpp>
#include <core/serialization/simulation_timeline.hpp>
#include <core/serialization/snapshot_helpers.hpp>

#include <memory>

namespace phynity::app
{

using phynity::jobs::JobSystem;
using phynity::jobs::JobSystemConfig;
using phynity::jobs::SchedulingMode;
using phynity::math::vectors::Vec3f;
using phynity::physics::DragField;
using phynity::physics::ForceField;
using phynity::physics::GravityField;
using phynity::physics::Material;
using phynity::physics::ParticleSystem;
using phynity::physics::RigidBodySystem;
using phynity::physics::TimestepController;
using phynity::physics::constants::EARTH_GRAVITY;

/// Application-level physics context manager.
/// Handles lifecycle management of the particle system, timestep controller,
/// and provides convenient methods for scenario setup and diagnostics.
class PhysicsContext
{
public:
    /// Configuration for physics simulation
    struct Config
    {
        float target_fps = 60.0f; ///< Target frames per second
        float max_timestep = 1.0f / 30.0f; ///< Maximum time per physics step
        bool use_determinism = true; ///< Enable deterministic mode
        bool enable_jobs = true; ///< Enable job system parallelization
        uint32_t job_workers = 0; ///< Worker count (0 = auto)
        Vec3f gravity = Vec3f(0.0f, -EARTH_GRAVITY, 0.0f); ///< Gravitational acceleration
        float air_drag = 0.0f; ///< Air drag coefficient
        bool record_schedule = false; ///< Record task execution schedule
        std::string record_schedule_path; ///< Path to save recorded schedule
        std::string replay_schedule_path; ///< If non-empty, replay from this file
        RigidBodySystem::Config rigid_body_config{}; ///< Rigid body system configuration

        /// Default constructor initializes all fields to defaults above
        Config() = default;
    };

    /// Unified diagnostics aggregating both particle and rigid body systems
    struct Diagnostics
    {
        size_t particle_count = 0;
        size_t body_count = 0;
        size_t constraint_count = 0;
        float total_kinetic_energy = 0.0f;
        float total_linear_ke = 0.0f;
        float total_angular_ke = 0.0f;
        Vec3f total_momentum = Vec3f(0.0f);
        Vec3f total_angular_momentum = Vec3f(0.0f);
    };

    /// Constructor with default or custom configuration
    explicit PhysicsContext(const Config &config);

    /// Constructor with default configuration
    PhysicsContext();

    /// Destructor ensures job system shutdown
    ~PhysicsContext();

    // Non-copyable, non-movable
    PhysicsContext(const PhysicsContext &) = delete;
    PhysicsContext &operator=(const PhysicsContext &) = delete;

    // ========================================================================
    // Simulation Control
    // ========================================================================

    /// Update the simulation by delta_time seconds
    /// Accumulates time and performs physics steps as needed
    /// When paused, this is a no-op. When speed != 1.0, dt is scaled.
    /// @param delta_time Time elapsed since last frame (seconds)
    void update(float delta_time);

    /// Manually advance one physics step with fixed timestep
    /// Used for deterministic testing or scripted scenarios
    void step_deterministic();

    /// Reset the timestep accumulator without performing physics steps
    void reset_accumulator();

    // ========================================================================
    // Timeline Control (pause / step / rewind)
    // ========================================================================

    /// Pause the simulation
    void pause();

    /// Resume the simulation
    void resume();

    /// Check if simulation is paused
    bool is_paused() const
    {
        return paused_;
    }

    /// Advance exactly one fixed timestep (works when paused)
    void step_forward();

    /// Restore previous snapshot from timeline (works when paused)
    void step_backward();

    /// Jump to a specific frame in the timeline
    void seek_to_frame(size_t index);

    /// Set simulation speed multiplier (0.25 to 4.0)
    void set_speed(float multiplier);

    /// Get current speed multiplier
    float speed() const
    {
        return speed_multiplier_;
    }

    /// Get current position in timeline (index of most recent frame)
    size_t current_frame_index() const
    {
        return timeline_.size() > 0 ? timeline_.size() - 1 : 0;
    }

    /// Get total frames stored in timeline
    size_t timeline_size() const
    {
        return timeline_.size();
    }

    // ========================================================================
    // Particle Management (delegates to ParticleSystem)
    // ========================================================================

    /// Spawn a new particle with mass
    void spawn_particle(const Vec3f &position, const Vec3f &velocity, float mass = 1.0f, float radius = -1.0f);

    /// Spawn a new particle with full material specification
    void spawn_particle(const Vec3f &position, const Vec3f &velocity, const Material &material, float radius = -1.0f);

    /// Clear all particles from the system
    void clear_particles();

    /// Get the number of active particles
    size_t particle_count() const;

    /// Get direct access to the particle system
    ParticleSystem &particle_system()
    {
        return particle_system_;
    }
    const ParticleSystem &particle_system() const
    {
        return particle_system_;
    }

    // ========================================================================
    // Rigid Body Management (delegates to RigidBodySystem)
    // ========================================================================

    /// Spawn a new rigid body
    phynity::physics::RigidBodyID spawn_body(const Vec3f &position,
                                             const phynity::math::quaternions::Quatf &orientation,
                                             std::shared_ptr<phynity::physics::shapes::Shape> shape,
                                             float mass = 1.0f,
                                             const Material &material = Material{});

    /// Clear all rigid bodies
    void clear_bodies();

    /// Get the number of active rigid bodies
    size_t body_count() const;

    /// Get direct access to the rigid body system
    RigidBodySystem &rigid_body_system()
    {
        return rigid_body_system_;
    }
    const RigidBodySystem &rigid_body_system() const
    {
        return rigid_body_system_;
    }

    // ========================================================================
    // Force Field Management
    // ========================================================================

    /// Set gravity acceleration (applies to both particle and rigid body systems)
    void set_gravity(const Vec3f &gravity);

    /// Set air drag coefficient (particle system only)
    void set_drag(float drag_coefficient);

    /// Clear all force fields (both systems)
    void clear_force_fields();

    /// Get the number of active force fields (particle system)
    size_t force_field_count() const;

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Get unified diagnostics aggregating both particle and rigid body systems
    Diagnostics diagnostics() const;

    /// Get timestep controller statistics
    const TimestepController::Statistics &timestep_statistics() const;

    /// Print diagnostic information to standard output
    void print_diagnostics() const;

    // ========================================================================
    // Configuration Access
    // ========================================================================

    /// Get current configuration
    const Config &config() const
    {
        return config_;
    }

    /// Get target timestep (1 / target_fps)
    float target_timestep() const;

private:
    Config config_;
    ParticleSystem particle_system_;
    RigidBodySystem rigid_body_system_;
    TimestepController timestep_controller_;
    JobSystem job_system_;
    std::unique_ptr<phynity::jobs::ScheduleRecorder> schedule_recorder_;
    std::unique_ptr<phynity::jobs::ScheduleReplayer> schedule_replayer_;

    // Timeline state
    serialization::SimulationTimeline timeline_{600};
    bool paused_ = false;
    float speed_multiplier_ = 1.0f;
    uint64_t frame_counter_ = 0;

    /// Initialize force fields based on configuration (both systems)
    void initialize_force_fields();

    /// Save recorded schedule to disk (called by destructor if recording)
    void save_schedule();

    /// Capture current state into timeline
    void capture_timeline_frame();

    /// Restore state from a timeline snapshot
    void restore_from_snapshot(const serialization::PhysicsSnapshot &snapshot);
};

} // namespace phynity::app

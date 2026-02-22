#pragma once

#include <core/physics/particle_system.hpp>
#include <core/physics/timestep_controller.hpp>
#include <core/physics/material.hpp>
#include <core/physics/force_field.hpp>
#include <core/math/vectors/vec3.hpp>
#include <memory>

namespace phynity::app {

using phynity::physics::ParticleSystem;
using phynity::physics::TimestepController;
using phynity::physics::Material;
using phynity::physics::ForceField;
using phynity::physics::GravityField;
using phynity::physics::DragField;
using phynity::math::vectors::Vec3f;

/// Application-level physics context manager.
/// Handles lifecycle management of the particle system, timestep controller,
/// and provides convenient methods for scenario setup and diagnostics.
class PhysicsContext {
public:
    /// Configuration for physics simulation
    struct Config {
        float target_fps = 60.0f;           ///< Target frames per second
        float max_timestep = 1.0f / 30.0f;  ///< Maximum time per physics step
        bool use_determinism = true;        ///< Enable deterministic mode
        Vec3f gravity = Vec3f(0.0f, -9.81f, 0.0f);  ///< Gravitational acceleration
        float air_drag = 0.0f;              ///< Air drag coefficient
        
        /// Default constructor initializes all fields to defaults above
        Config() = default;
    };

    /// Constructor with default or custom configuration
    explicit PhysicsContext(const Config& config);
    
    /// Constructor with default configuration
    PhysicsContext();

    // Non-copyable, non-movable
    PhysicsContext(const PhysicsContext&) = delete;
    PhysicsContext& operator=(const PhysicsContext&) = delete;

    // ========================================================================
    // Simulation Control
    // ========================================================================

    /// Update the simulation by delta_time seconds
    /// Accumulates time and performs physics steps as needed
    /// @param delta_time Time elapsed since last frame (seconds)
    void update(float delta_time);

    /// Manually advance one physics step with fixed timestep
    /// Used for deterministic testing or scripted scenarios
    void step_deterministic();

    /// Reset the timestep accumulator without performing physics steps
    void reset_accumulator();

    // ========================================================================
    // Particle Management (delegates to ParticleSystem)
    // ========================================================================

    /// Spawn a new particle with mass
    void spawn_particle(
        const Vec3f& position,
        const Vec3f& velocity,
        float mass = 1.0f,
        float radius = -1.0f
    );

    /// Spawn a new particle with full material specification
    void spawn_particle(
        const Vec3f& position,
        const Vec3f& velocity,
        const Material& material,
        float radius = -1.0f
    );

    /// Clear all particles from the system
    void clear_particles();

    /// Get the number of active particles
    size_t particle_count() const;

    /// Get direct access to the particle system
    ParticleSystem& particle_system() { return particle_system_; }
    const ParticleSystem& particle_system() const { return particle_system_; }

    // ========================================================================
    // Force Field Management
    // ========================================================================

    /// Set gravity acceleration
    void set_gravity(const Vec3f& gravity);

    /// Set air drag coefficient
    void set_drag(float drag_coefficient);

    /// Clear all force fields
    void clear_force_fields();

    /// Get the number of active force fields
    size_t force_field_count() const;

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Get current system diagnostics (energy, momentum, particle count)
    ParticleSystem::Diagnostics diagnostics() const;

    /// Get timestep controller statistics
    const TimestepController::Statistics& timestep_statistics() const;

    /// Print diagnostic information to standard output
    void print_diagnostics() const;

    // ========================================================================
    // Configuration Access
    // ========================================================================

    /// Get current configuration
    const Config& config() const { return config_; }

    /// Get target timestep (1 / target_fps)
    float target_timestep() const;

private:
    Config config_;
    ParticleSystem particle_system_;
    TimestepController timestep_controller_;

    /// Initialize force fields based on configuration
    void initialize_force_fields();
};

}  // namespace phynity::app

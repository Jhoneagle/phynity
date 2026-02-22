#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/material.hpp>

namespace phynity::physics {

using phynity::math::vectors::Vec3f;

/// Represents a single particle in the simulation.
/// Each particle has mass, kinematic state (position, velocity), material properties,
/// and lifecycle management (lifetime, active flags for pooling).
class Particle {
public:
    // ========================================================================
    // State Variables
    // ========================================================================
    
    Vec3f position = Vec3f(0.0f);        ///< Current position in world space
    Vec3f velocity = Vec3f(0.0f);        ///< Current velocity
    Vec3f acceleration = Vec3f(0.0f);    ///< Current acceleration (computed each frame)
    Vec3f force_accumulator = Vec3f(0.0f); ///< Accumulated forces this frame
    float radius = 0.5f;                 ///< Collision radius for sphere collisions

    // ========================================================================
    // Material and Lifecycle
    // ========================================================================
    
    Material material{};           ///< Material properties (mass, restitution, etc.)
    float lifetime = -1.0f;        ///< Remaining lifetime (< 0 = infinite, 0 = dead, > 0 = finite)
    bool active = true;            ///< Active flag for pooling/recycling

    // ========================================================================
    // Constructors
    // ========================================================================

    /// Default constructor - creates particle at origin with default material
    Particle() = default;

    /// Full constructor with all parameters
    Particle(
        const Vec3f& pos,
        const Vec3f& vel = Vec3f(0.0f),
        const Material& mat = Material{}
    )
        : position(pos), velocity(vel), material(mat)
    {
    }

    // ========================================================================
    // Force Application
    // ========================================================================

    /// Accumulate a force on this particle (does not directly modify acceleration)
    /// @param force Force vector in Newtons
    void apply_force(const Vec3f& force) {
        force_accumulator += force;
    }

    /// Clear the force accumulator (should be called after integration)
    void clear_forces() {
        force_accumulator = Vec3f(0.0f);
    }

    /// Compute acceleration from accumulated forces and material mass
    /// Should be called after all forces are applied and before integration
    void update_acceleration() {
        if (material.mass > 0.0f) {
            acceleration = force_accumulator * (1.0f / material.mass);
        } else {
            acceleration = Vec3f(0.0f);
        }
    }

    // ========================================================================
    // Integration
    // ========================================================================

    /// Integrates particle state using semi-implicit Euler integration.
    /// Order: update velocity -> apply damping -> update position
    /// @param dt Time step in seconds
    void integrate(float dt) {
        // Update velocity from acceleration
        velocity += acceleration * dt;

        // Apply velocity damping (material effect)
        float damping_factor = std::max(0.0f, 1.0f - material.linear_damping * dt);
        velocity = velocity * damping_factor;

        // Update position from velocity
        position += velocity * dt;

        // Update lifetime if finite
        if (lifetime > 0.0f) {
            lifetime -= dt;
            // Clamp to 0 when expired to distinguish from infinite lifetime
            if (lifetime < 0.0f) lifetime = 0.0f;
        }

        // Clear forces for next frame (will be re-applied by force fields)
        clear_forces();
        acceleration = Vec3f(0.0f);
    }

    // ========================================================================
    // Lifecycle Management
    // ========================================================================

    /// Returns true if particle is still alive
    /// Particles with lifetime < 0 are infinite (always alive)
    /// Particles with lifetime > 0 are alive and counting down
    /// Particles with lifetime <= 0 (for positive lifetimes) are dead
    bool is_alive() const {
        return lifetime < 0.0f || lifetime > 0.0f;
    }

    /// Set finite lifetime for this particle
    /// @param lifetime_sec Remaining time in seconds (> 0)
    void set_lifetime(float lifetime_sec) {
        lifetime = lifetime_sec;
    }

    /// Set infinite lifetime for this particle
    void set_infinite_lifetime() {
        lifetime = -1.0f;
    }

    /// Reset particle to initial state (for pooling)
    /// @param pos New position
    /// @param vel New velocity
    /// @param mat Material (optional)
    void reset(
        const Vec3f& pos,
        const Vec3f& vel = Vec3f(0.0f),
        const Material& mat = Material{}
    ) {
        position = pos;
        velocity = vel;
        acceleration = Vec3f(0.0f);
        force_accumulator = Vec3f(0.0f);
        radius = 0.5f;
        material = mat;
        lifetime = -1.0f;
        active = true;
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Get inverse mass (for efficient force calculations)
    /// Returns 0 if mass is zero (kinematic particle)
    float inverse_mass() const {
        return material.mass > 0.0f ? 1.0f / material.mass : 0.0f;
    }

    /// Get speed (magnitude of velocity)
    float speed() const {
        return velocity.length();
    }

    /// Get kinetic energy
    float kinetic_energy() const {
        return 0.5f * material.mass * velocity.squaredLength();
    }

    // ========================================================================
    // Deprecated Methods (for backwards compatibility)
    // ========================================================================

    /// @deprecated Use apply_force() instead
    void applyForce(const Vec3f& force) {
        apply_force(force);
    }

    /// @deprecated Use is_alive() instead
    bool isAlive() const {
        return is_alive();
    }

    /// @deprecated Use integrate() with material damping instead
    void integrate_legacy(float dt) {
        velocity += acceleration * dt;
        position += velocity * dt;

        if (lifetime > 0.0f) {
            lifetime -= dt;
        }

        clear_forces();
        acceleration = Vec3f(0.0f);
    }
};

}  // namespace phynity::physics

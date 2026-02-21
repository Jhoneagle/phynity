#pragma once

#include <core/physics/particle.hpp>
#include <core/physics/force_field.hpp>
#include <algorithm>
#include <memory>
#include <vector>

namespace phynity::physics {

/// Manages a collection of particles and force fields, providing simulation stepping.
/// Integrates Material system, ForceField system, and provides energy/momentum diagnostics.
class ParticleSystem {
public:
    /// Diagnostic information about the particle system
    struct Diagnostics {
        float total_kinetic_energy = 0.0f;  ///< Sum of all particle kinetic energies
        Vec3f total_momentum = Vec3f(0.0f); ///< Sum of all particle momenta (mass * velocity)
        size_t particle_count = 0;          ///< Number of active particles
    };

    ParticleSystem() = default;

    // ========================================================================
    // Particle Management
    // ========================================================================

    /// Spawn a new particle at position with initial velocity.
    /// @param position Starting position
    /// @param velocity Starting velocity
    /// @param mass Particle mass (default: 1.0)
    /// @param lifetime Particle lifetime (-1 = infinite, > 0 = finite)
    void spawn(const Vec3f& position, const Vec3f& velocity, float mass = 1.0f, float lifetime = -1.0f) {
        particles_.emplace_back();
        Particle& p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material.mass = mass;
        p.lifetime = lifetime;
    }

    /// Spawn a new particle with full material specification.
    /// @param position Starting position
    /// @param velocity Starting velocity
    /// @param material Complete material definition
    /// @param lifetime Particle lifetime (-1 = infinite, > 0 = finite)
    void spawn(const Vec3f& position, const Vec3f& velocity, const Material& material, float lifetime = -1.0f) {
        particles_.emplace_back();
        Particle& p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material = material;
        p.lifetime = lifetime;
    }

    /// Clear all particles.
    void clear() { 
        particles_.clear(); 
    }

    /// Remove dead particles from the system.
    void remove_dead_particles() {
        particles_.erase(
            std::remove_if(particles_.begin(), particles_.end(),
                          [](const Particle& p) { return !p.is_alive(); }),
            particles_.end()
        );
    }

    // ========================================================================
    // Force Field Management
    // ========================================================================

    /// Add a force field to the system.
    /// The system takes ownership of the field.
    /// @param field Unique pointer to force field
    void add_force_field(std::unique_ptr<ForceField> field) {
        force_fields_.push_back(std::move(field));
    }

    /// Remove all force fields from the system.
    void clear_force_fields() {
        force_fields_.clear();
    }

    /// Get the number of active force fields.
    size_t force_field_count() const {
        return force_fields_.size();
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
    void update(float dt) {
        // Step 1: Clear forces from previous frame
        for (auto& p : particles_) {
            if (p.is_alive()) {
                p.clear_forces();
            }
        }

        // Step 2: Apply all force fields to all particles
        for (const auto& field : force_fields_) {
            for (auto& p : particles_) {
                if (p.is_alive()) {
                    Vec3f force = field->apply(p.position, p.velocity, p.material.mass);
                    p.apply_force(force);
                }
            }
        }

        // Step 3: Update accelerations from accumulated forces
        for (auto& p : particles_) {
            if (p.is_alive()) {
                p.update_acceleration();
            }
        }

        // Step 4: Integrate particle state
        for (auto& p : particles_) {
            if (p.is_alive()) {
                p.integrate(dt);
            }
        }

        // Step 5: Remove dead particles
        remove_dead_particles();
    }

    /// Legacy step method for backwards compatibility.
    /// @deprecated Use update() instead
    void step(float dt) {
        update(dt);
    }

    /// Apply gravity to all particles (legacy method).
    /// @deprecated Add a GravityField instead
    void applyGravity(const Vec3f& gravity) {
        for (auto& p : particles_) {
            if (p.is_alive()) {
                p.apply_force(gravity * p.material.mass);
            }
        }
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Compute current system diagnostics (energy, momentum).
    /// @return Diagnostics struct with current values
    Diagnostics compute_diagnostics() const {
        Diagnostics diag;
        diag.particle_count = particles_.size();
        diag.total_kinetic_energy = 0.0f;
        diag.total_momentum = Vec3f(0.0f);

        for (const auto& p : particles_) {
            if (p.is_alive()) {
                diag.total_kinetic_energy += p.kinetic_energy();
                diag.total_momentum += p.velocity * p.material.mass;
            }
        }

        return diag;
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Get particle count.
    size_t particleCount() const { return particles_.size(); }

    /// Get all particles (const access).
    const std::vector<Particle>& particles() const { return particles_; }

    /// Get all particles (mutable access).
    std::vector<Particle>& particles() { return particles_; }

private:
    std::vector<Particle> particles_;
    std::vector<std::unique_ptr<ForceField>> force_fields_;
};

}  // namespace phynity::physics

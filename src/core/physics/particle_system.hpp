#pragma once

#include <core/physics/particle.hpp>
#include <core/physics/force_field.hpp>
#include <core/physics/physics_constants.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <algorithm>
#include <memory>
#include <vector>

namespace phynity::physics {

using namespace phynity::physics::constants;

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
    
    // Move semantics
    ParticleSystem(ParticleSystem&& other) noexcept = default;
    ParticleSystem& operator=(ParticleSystem&& other) noexcept = default;

    // ========================================================================
    // Particle Management
    // ========================================================================

    /// Spawn a new particle at position with initial velocity.
    /// @param position Starting position
    /// @param velocity Starting velocity
    /// @param mass Particle mass (default: 1.0)
    /// @param lifetime Particle lifetime (-1 = infinite, > 0 = finite)
    void spawn(
        const Vec3f& position,
        const Vec3f& velocity,
        float mass = 1.0f,
        float lifetime = -1.0f,
        float radius = -1.0f
    ) {
        particles_.emplace_back();
        Particle& p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material.mass = mass;
        p.lifetime = lifetime;
        p.radius = (radius > 0.0f) ? radius : default_collision_radius_;
    }

    /// Spawn a new particle with full material specification.
    /// @param position Starting position
    /// @param velocity Starting velocity
    /// @param material Complete material definition
    /// @param lifetime Particle lifetime (-1 = infinite, > 0 = finite)
    void spawn(
        const Vec3f& position,
        const Vec3f& velocity,
        const Material& material,
        float lifetime = -1.0f,
        float radius = -1.0f
    ) {
        particles_.emplace_back();
        Particle& p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material = material;
        p.lifetime = lifetime;
        p.radius = (radius > 0.0f) ? radius : default_collision_radius_;
    }

    /// Clear all particles.
    void clear() { 
        particles_.clear(); 
    }

    /// Remove dead particles from the system.
    /// WARNING: This invalidates any outstanding iterators or references to particles.
    /// Do not keep references to particles obtained via particles() before calling this method.
    /// The internal particle storage may be reordered or resized during removal.
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
    // Collision Management
    // ========================================================================

    /// Enable or disable simple sphere-sphere collision handling.
    void enable_collisions(bool enabled) {
        collisions_enabled_ = enabled;
    }

    /// Check whether collisions are enabled.
    bool collisions_enabled() const {
        return collisions_enabled_;
    }

    /// Set default collision radius used by spawn() when radius is not specified.
    void set_default_collision_radius(float radius) {
        if (radius > 0.0f) {
            default_collision_radius_ = radius;
        }
    }

    /// Get the current default collision radius.
    float default_collision_radius() const {
        return default_collision_radius_;
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

        // Step 5: Resolve collisions (optional)
        if (collisions_enabled_) {
            resolve_collisions();
        }

        // Step 6: Remove dead particles
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
    bool collisions_enabled_ = false;
    float default_collision_radius_ = 0.5f;

    void resolve_collisions() {
        const size_t count = particles_.size();
        for (size_t i = 0; i < count; ++i) {
            Particle& a = particles_[i];
            if (!a.is_alive()) {
                continue;
            }

            for (size_t j = i + 1; j < count; ++j) {
                Particle& b = particles_[j];
                if (!b.is_alive()) {
                    continue;
                }

                Vec3f delta = b.position - a.position;
                float dist = delta.length();
                float min_dist = a.radius + b.radius;

                using phynity::math::utilities::is_zero;
                if (is_zero(dist, COLLISION_EPSILON) || dist > min_dist) {
                    continue;
                }

                Vec3f normal = delta / dist;
                Vec3f relative_velocity = b.velocity - a.velocity;
                float rel_normal = relative_velocity.dot(normal);

                if (rel_normal >= 0.0f) {
                    continue;
                }

                float inv_m1 = a.inverse_mass();
                float inv_m2 = b.inverse_mass();
                float inv_sum = inv_m1 + inv_m2;
                if (inv_sum <= 0.0f) {
                    continue;
                }

                float restitution = std::min(a.material.restitution, b.material.restitution);
                float impulse_mag = -(1.0f + restitution) * rel_normal / inv_sum;
                Vec3f impulse = normal * impulse_mag;

                a.velocity -= impulse * inv_m1;
                b.velocity += impulse * inv_m2;

                float penetration = min_dist - dist;
                if (penetration > 0.0f) {
                    Vec3f correction = normal * (penetration / inv_sum);
                    a.position -= correction * inv_m1;
                    b.position += correction * inv_m2;
                }
            }
        }
    }
};

}  // namespace phynity::physics

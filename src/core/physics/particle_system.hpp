#pragma once

#include <core/physics/particle.hpp>
#include <algorithm>
#include <vector>

namespace phynity::physics {

/// Simple particle system that manages a pool of particles.
class ParticleSystem {
public:
    ParticleSystem() = default;

    /// Spawn a new particle at position with initial velocity.
    void spawn(const Vec3f& position, const Vec3f& velocity, float mass = 1.0f, float lifetime = -1.0f) {
        particles_.emplace_back();
        Particle& p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material.mass = mass;
        p.lifetime = lifetime;
    }

    /// Apply gravity to all particles.
    void applyGravity(const Vec3f& gravity) {
        for (auto& p : particles_) {
            if (p.is_alive()) {
                p.apply_force(gravity * p.material.mass);
            }
        }
    }

    /// Step the simulation by dt seconds.
    void step(float dt) {
        for (auto& p : particles_) {
            if (p.is_alive()) {
                p.integrate(dt);
            }
        }

        // Remove dead particles
        particles_.erase(
            std::remove_if(particles_.begin(), particles_.end(),
                          [](const Particle& p) { return !p.is_alive(); }),
            particles_.end()
        );
    }

    /// Get particle count.
    size_t particleCount() const { return particles_.size(); }

    /// Get all particles (const access).
    const std::vector<Particle>& particles() const { return particles_; }

    /// Clear all particles.
    void clear() { particles_.clear(); }

private:
    std::vector<Particle> particles_;
};

}  // namespace phynity::physics

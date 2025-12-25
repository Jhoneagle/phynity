#pragma once

#include <core/math/vec3.hpp>

namespace phynity::physics {

using namespace phynity::math;

/// Represents a single particle in the simulation.
struct Particle {
    Vec3 position;      ///< Current position
    Vec3 velocity;      ///< Current velocity
    Vec3 acceleration;  ///< Current acceleration (computed each step)
    float mass = 1.0f;  ///< Mass (inverse used for forces)
    float lifetime = 0.0f;  ///< Remaining lifetime (0 = infinite)

    /// Integrates position and velocity using basic Euler method.
    /// @param dt Time step in seconds
    void integrate(float dt) {
        velocity += acceleration * dt;
        position += velocity * dt;
        
        if (lifetime > 0.0f) {
            lifetime -= dt;
        }
        
        // Reset acceleration for next frame (forces re-applied per step)
        acceleration = Vec3(0.0f);
    }

    /// Applies a force scaled by inverse mass.
    void applyForce(const Vec3& force) {
        acceleration += force * (1.0f / mass);
    }

    /// Returns true if particle is still alive.
    bool isAlive() const {
        return lifetime < 0.0f || lifetime > 0.0f;
    }
};

}  // namespace phynity::physics

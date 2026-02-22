#pragma once

#include "physics_context.hpp"
#include <string>

namespace phynity::app::scenarios {

using phynity::math::vectors::Vec3f;
using phynity::physics::Material;

/// Base class for demo scenarios
class Scenario {
public:
    virtual ~Scenario() = default;
    
    /// Return the name of this scenario
    virtual const char* name() const = 0;
    
    /// Return a brief description
    virtual const char* description() const = 0;
    
    /// Setup the scenario (spawn particles, configure forces, etc.)
    virtual void setup(PhysicsContext& context) = 0;
    
    /// Optional: Called for each physics step to apply external forces or events
    virtual void step_callback(PhysicsContext& /*context*/, float /*dt*/) {}
};

// ============================================================================
// Scenario Implementations
// ============================================================================

/// Simple gravity well: particles fall under constant gravity
class GravityWell : public Scenario {
public:
    const char* name() const override {
        return "Gravity Well";
    }
    
    const char* description() const override {
        return "Particles falling under Earth gravity (EARTH_GRAVITY m/s²)";
    }
    
    void setup(PhysicsContext& context) override;
};

/// Horizontal particle spread with gravity
class ParticleSpread : public Scenario {
public:
    const char* name() const override {
        return "Particle Spread";
    }
    
    const char* description() const override {
        return "Multiple particles with different initial velocities under gravity";
    }
    
    void setup(PhysicsContext& context) override;
};

/// Particles subject to air drag
class DragInteraction : public Scenario {
public:
    const char* name() const override {
        return "Drag Interaction";
    }
    
    const char* description() const override {
        return "Particles slowing down due to air resistance";
    }
    
    void setup(PhysicsContext& context) override;
};

/// Simple projectile motion scenario
class ProjectileMotion : public Scenario {
public:
    const char* name() const override {
        return "Projectile Motion";
    }
    
    const char* description() const override {
        return "Classic projectile trajectory under gravity";
    }
    
    void setup(PhysicsContext& context) override;
};

/// Orbit stability using a spring field toward origin
class OrbitStability : public Scenario {
public:
    const char* name() const override {
        return "Orbit Stability";
    }

    const char* description() const override {
        return "Spring-centered orbit with near-constant radius";
    }

    void setup(PhysicsContext& context) override;
};

/// Multi-particle collision scenario (simple sphere-sphere response)
class MultiParticleCollision : public Scenario {
public:
    const char* name() const override {
        return "Multi-Particle Collision";
    }

    const char* description() const override {
        return "Two particles collide elastically and exchange momentum";
    }

    void setup(PhysicsContext& context) override;

private:
    float radius_ = 0.5f;
    float restitution_ = 1.0f;
};

/// Reduced gravity environment (e.g., Moon)
class LowGravity : public Scenario {
public:
    const char* name() const override {
        return "Low Gravity";
    }
    
    const char* description() const override {
        return "Particles in low gravity environment (lunar conditions)";
    }
    
    void setup(PhysicsContext& context) override;
};

/// Zero gravity (no forces)
class ZeroGravity : public Scenario {
public:
    const char* name() const override {
        return "Zero Gravity";
    }
    
    const char* description() const override {
        return "Particles in zero-gravity environment (constant velocity)";
    }
    
    void setup(PhysicsContext& context) override;
};

/// High air resistance scenario
class HighDrag : public Scenario {
public:
    const char* name() const override {
        return "High Drag";
    }
    
    const char* description() const override {
        return "Particles in viscous medium with high drag coefficient";
    }
    
    void setup(PhysicsContext& context) override;
};

}  // namespace phynity::app::scenarios

#pragma once

namespace phynity::physics {

/// Represents physical material properties for particles.
/// Defines mass, restitution, friction, damping, and drag characteristics.
struct Material {
    /// Mass in kilograms
    float mass = 1.0f;
    
    /// Restitution coefficient (bounce factor), typically [0, 1]
    /// 0 = no bounce (perfectly inelastic), 1 = perfect elastic bounce
    float restitution = 0.8f;
    
    /// Kinetic friction coefficient, typically [0, 1]
    float friction = 0.3f;
    
    /// Linear velocity damping coefficient, typically [0, 1]
    /// Applied per frame: velocity *= (1.0 - linear_damping * dt)
    float linear_damping = 0.01f;
    
    /// Angular velocity damping coefficient (reserved for future use)
    float angular_damping = 0.01f;
    
    /// Drag coefficient for velocity-dependent drag force
    /// F_drag = -drag_coefficient * velocity
    float drag_coefficient = 0.0f;

    /// Default constructor creates standard particle material
    constexpr Material() = default;

    /// Constructor with explicit values
    constexpr Material(
        float m,
        float rest = 0.8f,
        float fric = 0.3f,
        float lin_damp = 0.01f,
        float ang_damp = 0.01f,
        float drag = 0.0f
    )
        : mass(m),
          restitution(rest),
          friction(fric),
          linear_damping(lin_damp),
          angular_damping(ang_damp),
          drag_coefficient(drag)
    {
    }
};

// ============================================================================
// Preset Material Factories
// ============================================================================

/// High-density, hard material with minimal bounce and damping
/// Used for rigid, heavy objects like steel balls or metal cubes
inline constexpr Material steel() {
    return Material(
        /*mass=*/7850.0f,             // ~7.85 kg/cm³ typical density
        /*restitution=*/0.3f,         // Low bounce
        /*friction=*/0.6f,            // High friction
        /*linear_damping=*/0.02f,     // Slight damping
        /*angular_damping=*/0.02f,
        /*drag_coefficient=*/0.01f    // Very small air resistance
    );
}

/// Elastic material with good bounce and moderate friction
/// Used for rubber balls, flexible materials, and general-purpose objects
inline constexpr Material rubber() {
    return Material(
        /*mass=*/1200.0f,             // ~1.2 kg/cm³ typical density
        /*restitution=*/0.8f,         // High bounce
        /*friction=*/0.4f,            // Moderate friction
        /*linear_damping=*/0.01f,     // Low damping
        /*angular_damping=*/0.01f,
        /*drag_coefficient=*/0.05f    // Moderate air resistance
    );
}

/// Light, porous material with high friction
/// Used for wood blocks, cork, and general structural materials
inline constexpr Material wood() {
    return Material(
        /*mass=*/600.0f,              // ~0.6 kg/cm³ typical density
        /*restitution=*/0.4f,         // Moderate bounce
        /*friction=*/0.5f,            // Moderate-high friction
        /*linear_damping=*/0.02f,     // Slight damping
        /*angular_damping=*/0.02f,
        /*drag_coefficient=*/0.03f    // Low-moderate air resistance
    );
}

/// Light particle used for fluid simulations (SPH, etc.)
/// Low mass with significant drag for fluid-like behavior
inline constexpr Material fluid_particle() {
    return Material(
        /*mass=*/0.1f,                // Very light particle
        /*restitution=*/0.1f,         // No bounce (inelastic)
        /*friction=*/0.2f,            // Low friction
        /*linear_damping=*/0.05f,     // Moderate damping (viscous)
        /*angular_damping=*/0.05f,
        /*drag_coefficient=*/0.5f     // High drag (fluid-like)
    );
}

/// Extremely light particle for smoke, fire, or dust simulations
/// Minimal mass with very high damping and drag
inline constexpr Material dust() {
    return Material(
        /*mass=*/0.01f,               // Extremely light
        /*restitution=*/0.0f,         // No bounce
        /*friction=*/0.1f,            // Very low friction
        /*linear_damping=*/0.1f,      // High damping
        /*angular_damping=*/0.1f,
        /*drag_coefficient=*/1.0f     // Very high drag
    );
}

/// Dense, hard material similar to stone
/// Good for rocks, concrete, and massive static objects
inline constexpr Material stone() {
    return Material(
        /*mass=*/2500.0f,             // ~2.5 kg/cm³ typical density
        /*restitution=*/0.15f,        // Very low bounce
        /*friction=*/0.7f,            // High friction
        /*linear_damping=*/0.01f,     // Minimal damping
        /*angular_damping=*/0.01f,
        /*drag_coefficient=*/0.02f    // Low air resistance
    );
}

}  // namespace phynity::physics

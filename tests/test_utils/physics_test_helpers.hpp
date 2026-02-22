#pragma once

#include <core/physics/material.hpp>
#include <core/math/utilities/constants.hpp>

namespace phynity::test::helpers {

using phynity::physics::Material;
using phynity::math::utilities::physicsf;

// ============================================================================
// Named Tolerance Constants for Self-Documenting Tests
// ============================================================================

namespace tolerance {
    /// Strict tolerance for deterministic/reference-based checks
    constexpr float STRICT = 1e-6f;
    
    /// Velocity comparison tolerance (accumulated integration error)
    constexpr float VELOCITY = 1e-5f;
    
    /// Position comparison tolerance (more integration accumulation)
    constexpr float POSITION = 1e-4f;
    
    /// Relative tolerance for energy conservation (1%)
    constexpr float ENERGY_REL = 0.01f;
    
    /// Momentum conservation tolerance
    constexpr float MOMENTUM = 1e-5f;

    // ========================================================================
    // Relative Tolerances for Physics Validation (percentage-based)
    // ========================================================================
    
    /// Relative tolerance for analytical solution comparison (1%)
    constexpr float ANALYTICAL_REL_1PCT = 0.01f;
    
    /// Relative tolerance for velocity (2%)
    constexpr float ANALYTICAL_REL_2PCT = 0.02f;
    
    /// Relative tolerance for projectile motion/ballistics (5%)
    constexpr float ANALYTICAL_REL_5PCT = 0.05f;
    
    /// Absolute tolerance for reference-based simulation (precision: 1e-3)
    constexpr float REFERENCE_ABS = 1e-3f;
}

// ============================================================================
// Material Factory Helpers
// ============================================================================

/// Create material with zero damping for physics validation tests
/// @param mass Particle mass (kg)
/// @param restitution Coefficient of restitution (default: 0.8)
/// @return Material with specified mass and restitution, no damping
inline Material make_no_damping_material(float mass, float restitution = 0.8f) {
    return Material(
        mass,           // mass
        restitution,    // restitution
        0.3f,          // friction
        0.0f,          // linear_damping (disabled)
        0.0f,          // angular_damping (disabled)
        0.0f           // drag_coefficient (disabled)
    );
}

/// Create material with low restitution (minimal bounce)
/// @param mass Particle mass (kg)
/// @return Material with low restitution, no damping
inline Material make_low_restitution_material(float mass) {
    return Material(
        mass,    // mass
        0.1f,    // restitution (minimal bounce)
        0.3f,    // friction
        0.0f,    // linear_damping (disabled)
        0.0f,    // angular_damping (disabled)
        0.0f     // drag_coefficient (disabled)
    );
}

// ============================================================================
// Physics Constants for Tests
// ============================================================================

namespace constants {
    /// Standard Earth gravity acceleration (m/s²)
    constexpr float EARTH_GRAVITY = physicsf::gravity_earth;
    
    /// Moon gravity acceleration (m/s²)
    constexpr float MOON_GRAVITY = 1.62f;
}

}  // namespace phynity::test::helpers

#pragma once

#include <core/math/utilities/constants.hpp>
#include <core/math/vectors/vec3.hpp>

namespace phynity::physics::constants {

using phynity::math::utilities::physicsf;
using phynity::math::utilities::mathf;
using phynity::math::vectors::Vec3f;

// ============================================================================
// Gravity Constants
// ============================================================================

/// Standard Earth gravity acceleration (m/s²) - SI precise value
constexpr float EARTH_GRAVITY = physicsf::gravity_earth;  // 9.80665

/// Moon gravity acceleration (m/s²)
constexpr float MOON_GRAVITY = 1.62f;

// Common gravity vectors (using precise SI value)
constexpr Vec3f EARTH_GRAVITY_VECTOR = Vec3f(0.0f, -EARTH_GRAVITY, 0.0f);
constexpr Vec3f MOON_GRAVITY_VECTOR = Vec3f(0.0f, -MOON_GRAVITY, 0.0f);
constexpr Vec3f ZERO_GRAVITY_VECTOR = Vec3f(0.0f, 0.0f, 0.0f);

// ============================================================================
// Collision and Simulation Tolerances
// ============================================================================

/// Epsilon for collision detection (geometric tolerance)
constexpr float COLLISION_EPSILON = physicsf::linear_slop;  // 1e-5

/// Epsilon for near-zero velocity checks
constexpr float VELOCITY_EPSILON = 1e-6f;

/// Epsilon for timestep comparisons
constexpr float TIMESTEP_EPSILON = 1e-6f;

// ============================================================================
// Mathematical Constants
// ============================================================================

/// Pi constant with full precision
constexpr float PI = mathf::pi;

/// Two pi (2π)
constexpr float TWO_PI = mathf::two_pi;

/// Half pi (π/2)
constexpr float HALF_PI = mathf::half_pi;

/// Degrees to radians conversion factor
constexpr float DEG_TO_RAD = mathf::deg_to_rad;

/// Radians to degrees conversion factor
constexpr float RAD_TO_DEG = mathf::rad_to_deg;

}  // namespace phynity::physics::constants

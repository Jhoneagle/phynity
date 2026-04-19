#pragma once

#include <core/math/vectors/vec3.hpp>

namespace phynity::physics::collision
{

using phynity::math::vectors::Vec3f;

/// Lightweight data proxy for sphere-based collision detection.
/// Not a shape — extracts the minimal properties needed by narrowphase
/// and impulse resolution from any body type.
struct CollisionProxy
{
    Vec3f position = Vec3f(0.0f); ///< World position
    Vec3f velocity = Vec3f(0.0f); ///< Linear velocity
    float radius = 0.5f; ///< Collision radius
    float inverse_mass = 1.0f; ///< Inverse mass (0 for static/infinite mass objects)
    float restitution = 0.0f; ///< Coefficient of restitution
};

} // namespace phynity::physics::collision

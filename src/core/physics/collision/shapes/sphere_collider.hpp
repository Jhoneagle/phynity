#pragma once

#include <core/math/vectors/vec3.hpp>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Minimal interface for objects that can participate in sphere-based collision detection
/// Any object with these properties can be used with SphereSpherNarrowphase and ImpulseResolver
struct SphereCollider {
    Vec3f position = Vec3f(0.0f);        ///< World position
    Vec3f velocity = Vec3f(0.0f);        ///< Linear velocity
    float radius = 0.5f;                 ///< Collision radius
    float inverse_mass = 1.0f;           ///< Inverse mass (0 for static/infinite mass objects)
    float restitution = 0.0f;            ///< Coefficient of restitution
};

}  // namespace phynity::physics::collision

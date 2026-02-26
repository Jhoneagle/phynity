#pragma once

#include <core/math/vectors/vec3.hpp>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Represents a single contact point between two objects
struct ContactPoint {
    Vec3f position = Vec3f(0.0f);        ///< Contact point position in world space
    Vec3f normal = Vec3f(0.0f, 1.0f, 0.0f); ///< Contact normal (from A to B)
    float penetration = 0.0f;            ///< Penetration depth (positive = penetrating)
    float relative_velocity_along_normal = 0.0f; ///< Relative velocity along contact normal
};

/// Manages contact information between two objects
/// For simple sphere-sphere collisions, typically contains one contact point
struct ContactManifold {
    size_t object_a_id = static_cast<size_t>(-1);                ///< ID or index of object A
    size_t object_b_id = static_cast<size_t>(-1);                ///< ID or index of object B
    
    ContactPoint contact;                ///< Contact point information
    
    bool is_valid() const {
        return object_a_id != static_cast<size_t>(-1) && object_b_id != static_cast<size_t>(-1);
    }
};

}  // namespace phynity::physics::collision

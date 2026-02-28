#pragma once

#include <core/math/vectors/vec3.hpp>
#include <cstdint>
#include <algorithm>

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
    size_t object_a_id = static_cast<size_t>(-1);   ///< ID or index of object A
    size_t object_b_id = static_cast<size_t>(-1);   ///< ID or index of object B
    
    ContactPoint contact;                ///< Contact point information
    
    // Contact caching fields (Phase 3)
    uint64_t contact_id = 0;             ///< Unique identifier for this contact (for matching across frames)
    int age = 0;                         ///< Number of frames this contact has been active
    Vec3f previous_impulse = Vec3f(0.0f); ///< Cached impulse from previous frame (for warm-start)
    
    bool is_valid() const {
        return object_a_id != static_cast<size_t>(-1) && object_b_id != static_cast<size_t>(-1);
    }
    
    /// Generate a contact ID based on the object pair
    /// Uses a deterministic hash that is order-independent (same ID for A-B and B-A)
    static uint64_t generate_contact_id(size_t obj_a, size_t obj_b) {
        // Ensure consistent ordering: smaller ID first
        size_t id1 = std::min(obj_a, obj_b);
        size_t id2 = std::max(obj_a, obj_b);
        
        // Simple hash combining the two IDs
        // Using a symmetric combination to ensure A-B == B-A
        uint64_t hash = static_cast<uint64_t>(id1);
        hash = (hash << 32) | static_cast<uint64_t>(id2);
        return hash;
    }
    
    /// Update the contact ID based on current object IDs
    void update_contact_id() {
        contact_id = generate_contact_id(object_a_id, object_b_id);
    }
};

}  // namespace phynity::physics::collision

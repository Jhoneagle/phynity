#pragma once

#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/collision/shapes/sphere_collider.hpp>
#include <core/physics/common/physics_constants.hpp>
#include <core/math/utilities/float_comparison.hpp>

namespace phynity::physics::collision {

using phynity::physics::constants::COLLISION_EPSILON;
using phynity::math::vectors::Vec3f;

/// Narrowphase collision detection for sphere-sphere contacts
/// Generates contact manifolds from geometric overlap.
/// Works with any objects that provide SphereCollider properties.
class SphereSpherNarrowphase {
public:
    /// Detect collision between two spheres
    /// @param collider_a First colliding object
    /// @param collider_b Second colliding object
    /// @param index_a Index or ID of first object
    /// @param index_b Index or ID of second object
    /// @return Contact manifold if collision detected, otherwise invalid manifold
    static ContactManifold detect(
        const SphereCollider& collider_a,
        const SphereCollider& collider_b,
        size_t index_a,
        size_t index_b
    ) {
        ContactManifold manifold;
        manifold.object_a_id = index_a;
        manifold.object_b_id = index_b;

        Vec3f delta = collider_b.position - collider_a.position;
        float dist = delta.length();
        float min_dist = collider_a.radius + collider_b.radius;

        // Check for zero distance (touching at same point)
        using phynity::math::utilities::is_zero;
        if (is_zero(dist, COLLISION_EPSILON) || dist > min_dist) {
            // No collision - return invalid manifold
            manifold.object_a_id = static_cast<size_t>(-1);
            manifold.object_b_id = static_cast<size_t>(-1);
            return manifold;
        }

        // Compute contact normal (from A to B)
        Vec3f normal = delta / dist;
        
        // Compute relative velocity
        Vec3f relative_velocity = collider_b.velocity - collider_a.velocity;
        float rel_normal = relative_velocity.dot(normal);

        // Skip if objects are separating
        if (rel_normal >= 0.0f) {
            manifold.object_a_id = static_cast<size_t>(-1);
            manifold.object_b_id = static_cast<size_t>(-1);
            return manifold;
        }

        // Populate contact information
        manifold.contact.position = collider_a.position + normal * collider_a.radius;
        manifold.contact.normal = normal;
        manifold.contact.penetration = min_dist - dist;
        manifold.contact.relative_velocity_along_normal = rel_normal;

        return manifold;
    }
};

}  // namespace phynity::physics::collision

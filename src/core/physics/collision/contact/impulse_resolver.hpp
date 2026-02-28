#pragma once

#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/collision/shapes/sphere_collider.hpp>
#include <algorithm>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Resolves collisions using impulse-based response
/// Handles velocity impulses and penetration correction.
/// Works with any objects that provide the SphereCollider interface.
class ImpulseResolver {
public:
    /// Resolve a contact by applying impulse and penetration correction
    /// @param manifold Contact manifold describing the collision
    /// @param collider_a First colliding object (modified in place)
    /// @param collider_b Second colliding object (modified in place)
    /// @return The impulse vector that was applied (useful for warm-start caching)
    static Vec3f resolve(
        const ContactManifold& manifold,
        SphereCollider& collider_a,
        SphereCollider& collider_b
    ) {
        if (!manifold.is_valid()) {
            return Vec3f(0.0f);
        }

        const ContactPoint& contact = manifold.contact;

        // Compute inverse masses
        float inv_m1 = collider_a.inverse_mass;
        float inv_m2 = collider_b.inverse_mass;
        float inv_sum = inv_m1 + inv_m2;

        // Skip if both objects are static (infinite mass)
        if (inv_sum <= 0.0f) {
            return Vec3f(0.0f);
        }

        // Apply impulse to resolve velocity
        float restitution = std::min(collider_a.restitution, collider_b.restitution);
        float impulse_mag = -(1.0f + restitution) * contact.relative_velocity_along_normal / inv_sum;
        Vec3f impulse = contact.normal * impulse_mag;

        collider_a.velocity -= impulse * inv_m1;
        collider_b.velocity += impulse * inv_m2;

        // Apply positional correction to resolve penetration
        if (contact.penetration > 0.0f) {
            Vec3f correction = contact.normal * (contact.penetration / inv_sum);
            collider_a.position -= correction * inv_m1;
            collider_b.position += correction * inv_m2;
        }

        // Return the impulse for caching
        return impulse;
    }
};

}  // namespace phynity::physics::collision

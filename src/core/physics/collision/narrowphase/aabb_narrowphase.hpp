#pragma once

#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/collision/shapes/aabb.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <cmath>
#include <algorithm>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Narrowphase collision detection for AABB-AABB contacts
/// Generates contact manifolds from axis-aligned box overlaps
class AABBNarrowphase {
public:
    /// Detect collision between two AABBs
    /// Computes contact normal and penetration depth along the axis of least penetration
    /// @param aabb_a First AABB
    /// @param aabb_b Second AABB
    /// @param center_a Center position of first object (for contact point calculation)
    /// @param center_b Center position of second object (for contact point calculation)
    /// @param velocity_a Velocity of first object (for relative velocity calculation)
    /// @param velocity_b Velocity of second object (for relative velocity calculation)
    /// @param index_a Index or ID of first object
    /// @param index_b Index or ID of second object
    /// @return Contact manifold if collision detected, otherwise invalid manifold
    static ContactManifold detect(
        const AABB& aabb_a,
        const AABB& aabb_b,
        const Vec3f& center_a,
        const Vec3f& center_b,
        const Vec3f& velocity_a,
        const Vec3f& velocity_b,
        size_t index_a,
        size_t index_b
    ) {
        ContactManifold manifold;
        manifold.object_a_id = index_a;
        manifold.object_b_id = index_b;

        // Quick rejection test - no overlap means no collision
        if (!aabb_a.overlaps(aabb_b)) {
            manifold.object_a_id = static_cast<size_t>(-1);
            manifold.object_b_id = static_cast<size_t>(-1);
            return manifold;
        }

        // Find the axis of least penetration (separation axis with minimum overlap)
        Vec3f overlap;
        overlap.x = std::min(aabb_a.max.x - aabb_b.min.x, aabb_b.max.x - aabb_a.min.x);
        overlap.y = std::min(aabb_a.max.y - aabb_b.min.y, aabb_b.max.y - aabb_a.min.y);
        overlap.z = std::min(aabb_a.max.z - aabb_b.min.z, aabb_b.max.z - aabb_a.min.z);

        // Find minimum overlap axis
        float min_overlap = overlap.x;
        int axis = 0;  // 0=x, 1=y, 2=z

        if (overlap.y < min_overlap) {
            min_overlap = overlap.y;
            axis = 1;
        }
        if (overlap.z < min_overlap) {
            min_overlap = overlap.z;
            axis = 2;
        }

        // Build contact normal pointing from A to B
        Vec3f normal(0.0f, 0.0f, 0.0f);
        if (axis == 0) {
            normal.x = (center_b.x > center_a.x) ? 1.0f : -1.0f;
        } else if (axis == 1) {
            normal.y = (center_b.y > center_a.y) ? 1.0f : -1.0f;
        } else {
            normal.z = (center_b.z > center_a.z) ? 1.0f : -1.0f;
        }

        // Calculate relative velocity along contact normal
        Vec3f relative_velocity = velocity_b - velocity_a;
        float rel_normal = relative_velocity.dot(normal);

        // Skip if objects are separating
        if (rel_normal >= 0.0f) {
            manifold.object_a_id = static_cast<size_t>(-1);
            manifold.object_b_id = static_cast<size_t>(-1);
            return manifold;
        }

        // Compute contact point (midpoint on the collision plane)
        Vec3f contact_pos = (center_a + center_b) * 0.5f;

        // Populate contact information
        manifold.contact.position = contact_pos;
        manifold.contact.normal = normal;
        manifold.contact.penetration = min_overlap;
        manifold.contact.relative_velocity_along_normal = rel_normal;

        return manifold;
    }
};

}  // namespace phynity::physics::collision

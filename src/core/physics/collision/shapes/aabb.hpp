#pragma once

#include <core/math/vectors/vec3.hpp>
#include <algorithm>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Axis-Aligned Bounding Box for collision detection broadphase
/// Represents a box aligned with world axes, defined by min/max corners
struct AABB {
    Vec3f min = Vec3f(0.0f);  ///< Minimum corner (lower-left-back)
    Vec3f max = Vec3f(0.0f);  ///< Maximum corner (upper-right-front)

    /// Default constructor - zero-initialized AABB at origin
    AABB() = default;

    /// Construct AABB from min and max corners
    AABB(const Vec3f& min_corner, const Vec3f& max_corner)
        : min(min_corner), max(max_corner)
    {}

    /// Check if this AABB overlaps with another AABB
    /// Uses separating axis theorem for AABB-AABB overlap (3D)
    /// @param other The other AABB to test against
    /// @return true if AABBs overlap, false if separated or just touching
    bool overlaps(const AABB& other) const {
        if (max.x <= other.min.x || min.x >= other.max.x) return false;
        if (max.y <= other.min.y || min.y >= other.max.y) return false;
        if (max.z <= other.min.z || min.z >= other.max.z) return false;
        
        return true;
    }

    /// Check if this 2D AABB overlaps with another 2D AABB (ignores z-axis)
    /// For use in 2D collision detection where shapes are flat on z=0 plane
    /// @param other The other AABB to test against
    /// @return true if AABBs overlap, false if separated or just touching
    bool overlaps_2d(const AABB& other) const {
        if (max.x <= other.min.x || min.x >= other.max.x) return false;
        if (max.y <= other.min.y || min.y >= other.max.y) return false;
        
        return true;
    }

    /// Check if a point is contained within this AABB
    /// @param point The point to test
    /// @return true if point is inside or on boundary, false otherwise
    bool contains_point(const Vec3f& point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    /// Expand this AABB to include another AABB
    /// Modifies this AABB to be the bounding box of both
    /// @param other The AABB to merge with this one
    void expand(const AABB& other) {
        min.x = std::min(min.x, other.min.x);
        min.y = std::min(min.y, other.min.y);
        min.z = std::min(min.z, other.min.z);

        max.x = std::max(max.x, other.max.x);
        max.y = std::max(max.y, other.max.y);
        max.z = std::max(max.z, other.max.z);
    }

    /// Expand this AABB to include a point
    /// @param point The point to include
    void expand(const Vec3f& point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);

        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }

    /// Get the center point of this AABB
    /// @return Center position in world space
    Vec3f get_center() const {
        return (min + max) * 0.5f;
    }

    /// Get the half-extents (half-widths) of this AABB
    /// @return Vector containing half the width, height, and depth
    Vec3f get_half_extents() const {
        return (max - min) * 0.5f;
    }

    /// Get the full extents (dimensions) of this AABB
    /// @return Vector containing width, height, and depth
    Vec3f get_extents() const {
        return max - min;
    }

    /// Get the surface area of this AABB
    /// Useful for spatial partitioning heuristics
    /// @return Surface area
    float surface_area() const {
        Vec3f extents = get_extents();
        return 2.0f * (extents.x * extents.y + extents.y * extents.z + extents.z * extents.x);
    }

    /// Get the volume of this AABB
    /// @return Volume
    float volume() const {
        Vec3f extents = get_extents();
        return extents.x * extents.y * extents.z;
    }

    /// Create an AABB from a sphere
    /// @param center Sphere center position
    /// @param radius Sphere radius
    /// @return AABB that tightly bounds the sphere
    static AABB from_sphere(const Vec3f& center, float radius) {
        Vec3f offset(radius, radius, radius);
        return AABB(center - offset, center + offset);
    }

    /// Create an AABB that bounds multiple AABBs
    /// @param aabbs Vector of AABBs to merge
    /// @return Merged AABB
    static AABB merge(const std::vector<AABB>& aabbs) {
        if (aabbs.empty()) {
            return AABB();
        }

        AABB result = aabbs[0];
        for (size_t i = 1; i < aabbs.size(); ++i) {
            result.expand(aabbs[i]);
        }
        return result;
    }

    /// Check if this AABB is valid (min <= max on all axes)
    /// @return true if valid, false otherwise
    bool is_valid() const {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }
};

}  // namespace phynity::physics::collision

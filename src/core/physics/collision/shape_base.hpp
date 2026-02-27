#pragma once

#include <core/math/vectors/vec2.hpp>
#include <core/math/vectors/vec3.hpp>
#include <memory>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec2f;
using phynity::math::vectors::Vec3f;

// Forward declarations
struct AABB;

/// Virtual base class for collision shapes
/// All shapes must implement these methods for collision detection
class Shape {
public:
    virtual ~Shape() = default;

    /// Get the bounding box (AABB) of this shape in world space
    /// @return An AABB enclosing the shape
    virtual AABB get_aabb() const = 0;

    /// Get the closest point on the shape's surface along a given direction
    /// Used by GJK and other distance algorithms
    /// @param direction The search direction (should be normalized)
    /// @return The point on the shape's boundary furthest along the direction
    virtual Vec3f support_point(const Vec3f& direction) const = 0;

    /// Get the radius of the shape (for sphere-based approximations)
    /// @return The radius, or 0 if not applicable
    virtual float get_radius() const = 0;
};

/// Specialization for 2D shapes (lies in XY plane, Z=0)
class Shape2D : public Shape {
public:
    virtual ~Shape2D() = default;

    /// Get the support point in 2D (result has z=0)
    /// @param direction_2d The search direction in 2D
    /// @return The point on the shape's boundary in 2D (with z=0)
    virtual Vec2f support_point_2d(const Vec2f& direction) const = 0;

    /// Override 3D support_point to delegate to 2D version
    Vec3f support_point(const Vec3f& direction) const final {
        Vec2f dir_2d(direction.x, direction.y);
        Vec2f result_2d = support_point_2d(dir_2d);
        return Vec3f(result_2d.x, result_2d.y, 0.0f);
    }
};

/// Specialization for 3D shapes
class Shape3D : public Shape {
public:
    virtual ~Shape3D() = default;
};

}  // namespace phynity::physics::collision

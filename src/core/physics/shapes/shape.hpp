#pragma once

#include <core/math/vectors/vec2.hpp>
#include <core/math/vectors/vec3.hpp>

#include <memory>

namespace phynity::physics::shapes
{

using phynity::math::vectors::Vec2f;
using phynity::math::vectors::Vec3f;

// Forward declaration
struct AABB;

/// Enumeration for supported shape types
enum class ShapeType
{
    Sphere,
    Box,
    Capsule,
    ConvexHull2D,
    ConvexHull3D
};

/// Abstract base class for all collision shapes.
/// Shapes are pure geometry — they define support functions, bounding volumes,
/// and cloning, but not physics properties like inertia (which is computed externally).
class Shape
{
public:
    virtual ~Shape() = default;

    /// Identify the shape type for dispatch
    virtual ShapeType get_type() const = 0;

    /// Get the support point: the point on the shape's boundary furthest along the given direction.
    /// Used by GJK and other distance algorithms.
    /// @param direction The search direction (should be normalized)
    /// @return The point on the shape's boundary furthest along the direction
    virtual Vec3f support_point(const Vec3f &direction) const = 0;

    /// Get the bounding box (AABB) of this shape
    /// @return An AABB enclosing the shape
    virtual AABB get_aabb() const = 0;

    /// Get bounding sphere radius (for broadphase)
    virtual float get_bounding_radius() const = 0;

    /// Clone this shape into a unique_ptr
    virtual std::unique_ptr<Shape> clone() const = 0;
};

/// Specialization for 2D shapes (lies in XY plane, Z=0)
class Shape2D : public Shape
{
public:
    ~Shape2D() override = default;

    /// Get the support point in 2D (result has z=0)
    /// @param direction The search direction in 2D
    /// @return The point on the shape's boundary in 2D (with z=0)
    virtual Vec2f support_point_2d(const Vec2f &direction) const = 0;

    /// Override 3D support_point to delegate to 2D version
    Vec3f support_point(const Vec3f &direction) const final
    {
        Vec2f dir_2d(direction.x, direction.y);
        Vec2f result_2d = support_point_2d(dir_2d);
        return Vec3f(result_2d.x, result_2d.y, 0.0f);
    }
};

/// Specialization for 3D shapes
class Shape3D : public Shape
{
public:
    ~Shape3D() override = default;
};

} // namespace phynity::physics::shapes

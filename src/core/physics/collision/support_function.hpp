#pragma once

#include <core/physics/collision/shape_base.hpp>
#include <core/math/vectors/vec3.hpp>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Support function interface: returns the point on a shape's boundary furthest along a direction
class SupportFunction {
public:
    virtual ~SupportFunction() = default;

    /// Get the support point: the point on the shape's boundary furthest along the given direction
    /// @param direction Unit direction vector (should be normalized for best results)
    /// @return Point on the shape's boundary in that direction
    virtual Vec3f get_support_point(const Vec3f& direction) const = 0;

    /// Get the origin of the shape (for translating support points)
    /// @return The center/origin position
    virtual Vec3f get_origin() const = 0;
};

/// Support function wrapper for Shape objects
class ShapeSupportFunction : public SupportFunction {
private:
    const Shape* shape;
    Vec3f origin;

public:
    ShapeSupportFunction(const Shape* s, const Vec3f& org)
        : shape(s), origin(org) {}

    Vec3f get_support_point(const Vec3f& direction) const override {
        if (!shape) return origin;
        // Shape returns support point in local coordinates, add origin offset
        return shape->support_point(direction) + origin;
    }

    Vec3f get_origin() const override {
        return origin;
    }
};

/// Support function for the Minkowski difference (A - B)
/// This is used by GJK to find the closest point between two shapes
class MinkowskiDifferenceSupportFunction : public SupportFunction {
private:
    const SupportFunction& shape_a;
    const SupportFunction& shape_b;

public:
    MinkowskiDifferenceSupportFunction(const SupportFunction& a, const SupportFunction& b)
        : shape_a(a), shape_b(b) {}

    /// Get support point of (A - B) in direction d
    /// This is: support_a(d) - support_b(-d)
    Vec3f get_support_point(const Vec3f& direction) const override {
        Vec3f support_a = shape_a.get_support_point(direction);
        Vec3f support_b = shape_b.get_support_point(-direction);
        return support_a - support_b;
    }

    Vec3f get_origin() const override {
        return shape_a.get_origin() - shape_b.get_origin();
    }
};

}  // namespace phynity::physics::collision

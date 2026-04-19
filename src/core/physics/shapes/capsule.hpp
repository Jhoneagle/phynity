#pragma once

#include <core/physics/shapes/aabb.hpp>
#include <core/physics/shapes/shape.hpp>

#include <cmath>
#include <memory>

namespace phynity::physics::shapes
{

/// Represents a capsule shape (cylinder with hemispherical ends)
class CapsuleShape : public Shape
{
public:
    float radius = 0.25f;      ///< Radius of the capsule
    float half_height = 0.5f;  ///< Half-length of the cylinder
    Vec3f local_center;        ///< Center offset in body-local coordinates

    explicit CapsuleShape(float r = 0.25f, float h = 0.5f, const Vec3f &center = Vec3f(0.0f))
        : radius(r), half_height(h), local_center(center)
    {
    }

    Vec3f get_local_center() const override { return local_center; }

    ShapeType get_type() const override
    {
        return ShapeType::Capsule;
    }

    Vec3f support_point(const Vec3f &direction) const override
    {
        Vec3f normalized = direction.normalized();
        Vec3f result = local_center;
        result += normalized * radius;

        float axis_support = (normalized.y >= 0) ? half_height : -half_height;
        result.y += axis_support;

        return result;
    }

    AABB get_aabb() const override
    {
        float total_half_height = half_height + radius;
        Vec3f min_corner = local_center - Vec3f(radius, total_half_height, radius);
        Vec3f max_corner = local_center + Vec3f(radius, total_half_height, radius);
        return AABB(min_corner, max_corner);
    }

    float get_bounding_radius() const override
    {
        return std::sqrt(radius * radius + half_height * half_height + radius * radius);
    }

    std::unique_ptr<Shape> clone() const override
    {
        return std::make_unique<CapsuleShape>(*this);
    }
};

} // namespace phynity::physics::shapes

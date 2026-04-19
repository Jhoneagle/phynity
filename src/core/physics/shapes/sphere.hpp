#pragma once

#include <core/physics/shapes/aabb.hpp>
#include <core/physics/shapes/shape.hpp>

#include <memory>

namespace phynity::physics::shapes
{

/// Represents a spherical shape
class SphereShape : public Shape
{
public:
    float radius = 0.5f;
    Vec3f local_center; ///< Center offset in body-local coordinates

    explicit SphereShape(float r = 0.5f, const Vec3f &center = Vec3f(0.0f)) : radius(r), local_center(center)
    {
    }

    ShapeType get_type() const override
    {
        return ShapeType::Sphere;
    }

    Vec3f support_point(const Vec3f &direction) const override
    {
        Vec3f normalized_dir = direction.normalized();
        return local_center + normalized_dir * radius;
    }

    AABB get_aabb() const override
    {
        return AABB::from_sphere(local_center, radius);
    }

    float get_bounding_radius() const override
    {
        return radius;
    }

    std::unique_ptr<Shape> clone() const override
    {
        return std::make_unique<SphereShape>(*this);
    }
};

} // namespace phynity::physics::shapes

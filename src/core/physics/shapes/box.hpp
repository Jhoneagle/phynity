#pragma once

#include <core/physics/shapes/aabb.hpp>
#include <core/physics/shapes/shape.hpp>

#include <memory>
#include <vector>

namespace phynity::physics::shapes
{

/// Represents an axis-aligned box shape (for MVP; OBB extends later)
class BoxShape : public Shape
{
public:
    Vec3f half_extents; ///< Half-size in each dimension (x, y, z)
    Vec3f local_center; ///< Center offset in body-local coordinates

    explicit BoxShape(const Vec3f &half_ext = Vec3f(0.5f), const Vec3f &center = Vec3f(0.0f))
        : half_extents(half_ext), local_center(center)
    {
    }

    Vec3f get_local_center() const override { return local_center; }

    ShapeType get_type() const override
    {
        return ShapeType::Box;
    }

    Vec3f support_point(const Vec3f &direction) const override
    {
        Vec3f result = local_center;
        result.x += (direction.x >= 0.0f) ? half_extents.x : -half_extents.x;
        result.y += (direction.y >= 0.0f) ? half_extents.y : -half_extents.y;
        result.z += (direction.z >= 0.0f) ? half_extents.z : -half_extents.z;
        return result;
    }

    AABB get_aabb() const override
    {
        return AABB(local_center - half_extents, local_center + half_extents);
    }

    float get_bounding_radius() const override
    {
        return half_extents.length();
    }

    /// Get the 8 corner vertices of the box
    std::vector<Vec3f> get_vertices() const
    {
        std::vector<Vec3f> verts;
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                for (int k = 0; k < 2; ++k)
                {
                    Vec3f v = local_center;
                    v.x += (i == 0) ? -half_extents.x : half_extents.x;
                    v.y += (j == 0) ? -half_extents.y : half_extents.y;
                    v.z += (k == 0) ? -half_extents.z : half_extents.z;
                    verts.push_back(v);
                }
            }
        }
        return verts;
    }

    std::unique_ptr<Shape> clone() const override
    {
        return std::make_unique<BoxShape>(*this);
    }
};

} // namespace phynity::physics::shapes

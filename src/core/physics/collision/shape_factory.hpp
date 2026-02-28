#pragma once

#include <core/physics/collision/convex_hull.hpp>
#include <core/math/vectors/vec2.hpp>
#include <core/math/vectors/vec3.hpp>
#include <cmath>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec2f;
using phynity::math::vectors::Vec3f;

/// Factory functions for creating common convex shapes
class ShapeFactory {
public:
    /// Create a 2D axis-aligned box (AABB as convex hull)
    /// @param half_width Half width in X direction
    /// @param half_height Half height in Y direction
    /// @param center Center position
    /// @return ConvexHull2D representing a rectangle
    static ConvexHull2D create_box_2d(
        float half_width = 1.0f,
        float half_height = 1.0f,
        const Vec2f& center = Vec2f(0.0f)
    ) {
        std::vector<Vec2f> vertices;
        
        // Vertices in counter-clockwise order starting from bottom-left
        // Store in LOCAL space (relative to center)
        vertices.push_back(Vec2f(-half_width, -half_height));  // Bottom-left
        vertices.push_back(Vec2f(half_width, -half_height));   // Bottom-right
        vertices.push_back(Vec2f(half_width, half_height));    // Top-right
        vertices.push_back(Vec2f(-half_width, half_height));   // Top-left

        // Center position stored separately for world space
        return ConvexHull2D(vertices, center);
    }

    /// Create a 2D regular polygon
    /// @param sides Number of sides (3+ for valid polygon)
    /// @param radius Radius from center to vertices
    /// @param rotation_degrees Rotation in degrees
    /// @param center Center position
    /// @return ConvexHull2D representing a regular polygon
    static ConvexHull2D create_regular_polygon_2d(
        size_t sides = 6,
        float radius = 1.0f,
        float rotation_degrees = 0.0f,
        const Vec2f& center = Vec2f(0.0f)
    ) {
        if (sides < 3) {
            assert(false && "Polygon must have at least 3 sides");
            return ConvexHull2D();
        }

        std::vector<Vec2f> vertices;
        float angle_step = 2.0f * 3.14159265358979f / static_cast<float>(sides);
        float rotation_rad = rotation_degrees * 3.14159265358979f / 180.0f;

        for (size_t i = 0; i < sides; ++i) {
            float angle = static_cast<float>(i) * angle_step + rotation_rad;
            float x = radius * std::cos(angle);
            float y = radius * std::sin(angle);
            vertices.push_back(Vec2f(x, y));  // Store in local space
        }

        return ConvexHull2D(vertices, center);
    }

    /// Create a 2D triangle
    /// @param p0 First vertex
    /// @param p1 Second vertex
    /// @param p2 Third vertex
    /// @param center Center position (optional, auto-computed if not provided)
    /// @return ConvexHull2D representing a triangle
    static ConvexHull2D create_triangle_2d(
        const Vec2f& p0,
        const Vec2f& p1,
        const Vec2f& p2,
        const Vec2f& /*center*/ = Vec2f(0.0f)
    ) {
        std::vector<Vec2f> vertices = {p0, p1, p2};
        
        // Check if vertices are in CCW order
        float signed_area = (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
        if (signed_area < 0.0f) {
            // Clockwise, reverse to CCW
            std::reverse(vertices.begin(), vertices.end());
        }

        // Compute centroid
        Vec2f computed_center = (p0 + p1 + p2) / 3.0f;
        return ConvexHull2D(vertices, computed_center);
    }

    /// Create a 3D axis-aligned box
    /// @param half_width Half width in X direction
    /// @param half_height Half height in Y direction
    /// @param half_depth Half depth in Z direction
    /// @param center Center position
    /// @return ConvexHull3D representing a box
    static ConvexHull3D create_box_3d(
        float half_width = 1.0f,
        float half_height = 1.0f,
        float half_depth = 1.0f,
        const Vec3f& center = Vec3f(0.0f)
    ) {
        std::vector<Vec3f> vertices;

        // 8 vertices of the box in counter-clockwise order (when viewed from outside)
        float x = half_width;
        float y = half_height;
        float z = half_depth;

        // Bottom face (z = -half_depth)
        vertices.push_back(Vec3f(-x, -y, -z));  // 0: back-left-bottom
        vertices.push_back(Vec3f(x, -y, -z));   // 1: back-right-bottom
        vertices.push_back(Vec3f(x, y, -z));    // 2: front-right-bottom
        vertices.push_back(Vec3f(-x, y, -z));   // 3: front-left-bottom

        // Top face (z = +half_depth)
        vertices.push_back(Vec3f(-x, -y, z));   // 4: back-left-top
        vertices.push_back(Vec3f(x, -y, z));    // 5: back-right-top
        vertices.push_back(Vec3f(x, y, z));     // 6: front-right-top
        vertices.push_back(Vec3f(-x, y, z));    // 7: front-left-top

        ConvexHull3D hull(vertices, center);
        // Populate face normals for SAT collision detection
        // Box has 6 faces with normals in ±x, ±y, ±z directions
        hull.face_normals = {
            Vec3f(1.0f, 0.0f, 0.0f),   // +X face normal
            Vec3f(-1.0f, 0.0f, 0.0f),  // -X face normal
            Vec3f(0.0f, 1.0f, 0.0f),   // +Y face normal
            Vec3f(0.0f, -1.0f, 0.0f),  // -Y face normal
            Vec3f(0.0f, 0.0f, 1.0f),   // +Z face normal
            Vec3f(0.0f, 0.0f, -1.0f)   // -Z face normal
        };
        return hull;
    }

    /// Create a 3D regular tetrahedron
    /// @param edge_length Length of each edge
    /// @param center Center position
    /// @return ConvexHull3D representing a tetrahedron
    static ConvexHull3D create_tetrahedron_3d(
        float edge_length = 1.0f,
        const Vec3f& center = Vec3f(0.0f)
    ) {
        // Regular tetrahedron vertices
        std::vector<Vec3f> vertices;
        float a = edge_length / std::sqrt(8.0f / 3.0f);  // Scale factor
        
        vertices.push_back(Vec3f(a, a, a));
        vertices.push_back(Vec3f(a, -a, -a));
        vertices.push_back(Vec3f(-a, a, -a));
        vertices.push_back(Vec3f(-a, -a, a));

        return ConvexHull3D(vertices, center);
    }

    /// Create a 3D regular octahedron
    /// @param radius Distance from center to vertices
    /// @param center Center position
    /// @return ConvexHull3D representing an octahedron
    static ConvexHull3D create_octahedron_3d(
        float radius = 1.0f,
        const Vec3f& center = Vec3f(0.0f)
    ) {
        std::vector<Vec3f> vertices;
        
        // 6 vertices at (±r, 0, 0), (0, ±r, 0), (0, 0, ±r)
        vertices.push_back(Vec3f(radius, 0.0f, 0.0f));   // +X
        vertices.push_back(Vec3f(-radius, 0.0f, 0.0f));  // -X
        vertices.push_back(Vec3f(0.0f, radius, 0.0f));   // +Y
        vertices.push_back(Vec3f(0.0f, -radius, 0.0f));  // -Y
        vertices.push_back(Vec3f(0.0f, 0.0f, radius));   // +Z
        vertices.push_back(Vec3f(0.0f, 0.0f, -radius));  // -Z

        return ConvexHull3D(vertices, center);
    }

    /// Create a 3D cylinder (approximated as a convex hull with regular polygon cross-section)
    /// @param radius Radius of the cylinder
    /// @param height Height of the cylinder
    /// @param sides Number of sides in the polygonal approximation
    /// @param center Center position
    /// @return ConvexHull3D representing a cylinder
    static ConvexHull3D create_cylinder_3d(
        float radius = 1.0f,
        float height = 2.0f,
        size_t sides = 8,
        const Vec3f& center = Vec3f(0.0f)
    ) {
        if (sides < 3) {
            assert(false && "Cylinder must have at least 3 sides");
            return ConvexHull3D();
        }

        std::vector<Vec3f> vertices;
        float angle_step = 2.0f * 3.14159265358979f / static_cast<float>(sides);
        float half_height = height / 2.0f;

        // Bottom circle
        for (size_t i = 0; i < sides; ++i) {
            float angle = static_cast<float>(i) * angle_step;
            float x = radius * std::cos(angle);
            float z = radius * std::sin(angle);
            vertices.push_back(Vec3f(x, -half_height, z));
        }

        // Top circle
        for (size_t i = 0; i < sides; ++i) {
            float angle = static_cast<float>(i) * angle_step;
            float x = radius * std::cos(angle);
            float z = radius * std::sin(angle);
            vertices.push_back(Vec3f(x, half_height, z));
        }

        return ConvexHull3D(vertices, center);
    }
};

}  // namespace phynity::physics::collision

#pragma once

#include <core/physics/collision/shape_base.hpp>
#include <core/physics/collision/aabb.hpp>
#include <core/math/vectors/vec2.hpp>
#include <core/math/vectors/vec3.hpp>
#include <vector>
#include <cassert>
#include <cmath>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec2f;
using phynity::math::vectors::Vec3f;

/// Two-dimensional convex hull represented by vertices and edge normals
/// Used for SAT-based collision detection in 2D
class ConvexHull2D : public Shape2D {
public:
    std::vector<Vec2f> vertices;       ///< Vertices in CCW order
    std::vector<Vec2f> normals;        ///< Edge normals (outward facing) in same order as vertices
    Vec2f position = Vec2f(0.0f);      ///< Center/reference position
    float bounding_radius = 0.0f;      ///< Bounding circle radius for quick rejection
    AABB bound;                        ///< Bounding box in 3D (z=0)

    ConvexHull2D() = default;

    /// Construct from vertices (assumes CCW winding)
    /// @param verts Vertices in counter-clockwise order
    /// @param center Reference position (typically centroid)
    explicit ConvexHull2D(const std::vector<Vec2f>& verts, const Vec2f& center = Vec2f(0.0f))
        : vertices(verts), position(center)
    {
        if (vertices.size() < 3) {
            assert(false && "ConvexHull2D requires at least 3 vertices");
            return;
        }
        compute_normals();
        compute_bounds();  // This does NOT modify position anymore
        validate_invariants();
    }

    /// Add a vertex to the hull (rebuilds normals and bounds)
    /// @param vertex Vertex position
    void add_vertex(const Vec2f& vertex) {
        vertices.push_back(vertex);
        if (vertices.size() >= 3) {
            compute_normals();
            compute_bounds();
            validate_invariants();
        }
    }

    /// Recompute centroid position and update reference
    void update_centroid() {
        position = Vec2f(0.0f);
        for (const auto& v : vertices) {
            position = position + v;
        }
        if (!vertices.empty()) {
            position = position / static_cast<float>(vertices.size());
        }
    }

    /// Check if vertices are in counter-clockwise order
    /// @return true if CCW, false otherwise
    bool is_ccw() const {
        if (vertices.size() < 3) return true;
        
        float signed_area = 0.0f;
        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t next = (i + 1) % vertices.size();
            signed_area += (vertices[next].x - vertices[i].x) * (vertices[next].y + vertices[i].y);
        }
        return signed_area < 0.0f;  // negative area = CCW in standard coordinates
    }

    /// Validate hull invariants (CCW winding, normal directions)
    void validate_invariants() const {
        assert(vertices.size() >= 3 && "Hull must have at least 3 vertices");
        assert(is_ccw() && "Hull vertices must be in counter-clockwise order");
        assert(normals.size() == vertices.size() && "Normals count must match vertices count");
        assert(bounding_radius > 0.0f && "Bounding radius must be positive");
    }

    // Shape2D interface implementation
    Vec2f support_point_2d(const Vec2f& direction) const override {
        if (vertices.empty()) return Vec2f(0.0f);
        
        size_t best_idx = 0;
        float best_dot = vertices[0].dot(direction);
        
        for (size_t i = 1; i < vertices.size(); ++i) {
            float dot = vertices[i].dot(direction);
            if (dot > best_dot) {
                best_dot = dot;
                best_idx = i;
            }
        }
        
        return vertices[best_idx];
    }

    AABB get_aabb() const override {
        return bound;
    }

    float get_radius() const override {
        return bounding_radius;
    }

private:
    /// Compute outward-facing normals for each edge
    /// Assumes vertices are in CCW order
    void compute_normals() {
        normals.clear();
        normals.reserve(vertices.size());

        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t next = (i + 1) % vertices.size();
            Vec2f edge = vertices[next] - vertices[i];
            
            // Perpendicular in 2D: rotate 90° CCW gives (-y, x)
            // But for outward normal in CCW hull, we rotate 90° CW: (y, -x)
            Vec2f normal(edge.y, -edge.x);
            
            // Normalize
            float len = normal.length();
            if (len > 1e-6f) {
                normal = normal / len;
            } else {
                // Degenerate edge, use zero normal (will be caught in validation)
                normal = Vec2f(0.0f);
            }
            
            normals.push_back(normal);
        }
    }

    /// Compute bounding box and bounding circle
    void compute_bounds() {
        if (vertices.empty()) {
            bound = AABB(Vec3f(0.0f), Vec3f(0.0f));
            bounding_radius = 0.0f;
            return;
        }

        Vec2f min_pt = vertices[0];
        Vec2f max_pt = vertices[0];
        
        for (const auto& v : vertices) {
            min_pt.x = std::min(min_pt.x, v.x);
            min_pt.y = std::min(min_pt.y, v.y);
            max_pt.x = std::max(max_pt.x, v.x);
            max_pt.y = std::max(max_pt.y, v.y);
        }

        // Convert to 3D AABB (z=0 plane)
        bound = AABB(Vec3f(min_pt.x, min_pt.y, 0.0f),
                     Vec3f(max_pt.x, max_pt.y, 0.0f));

        // Compute bounding circle radius relative to the stored position
        // DO NOT update the position itself - it was set by the constructor parameter
        bounding_radius = 0.0f;
        for (const auto& v : vertices) {
            float dist = (v - position).length();
            bounding_radius = std::max(bounding_radius, dist);
        }
        // Add small margin for numerical stability
        bounding_radius *= 1.001f;
    }
};

/// Three-dimensional convex hull represented by vertices and face normals
/// Used for SAT-based collision detection in 3D
class ConvexHull3D : public Shape3D {
public:
    std::vector<Vec3f> vertices;       ///< Vertices of the hull
    std::vector<Vec3f> face_normals;   ///< Face normals (outward facing)
    std::vector<Vec3f> edge_normals;   ///< Edge normals for edge-edge testing in SAT
    Vec3f position = Vec3f(0.0f);      ///< Center/reference position
    float bounding_radius = 0.0f;      ///< Bounding sphere radius
    AABB bound;                        ///< Axis-aligned bounding box

    ConvexHull3D() = default;

    /// Construct from vertices (calls convex hull computation)
    /// @param verts Vertex positions
    /// @param center Reference position (typically centroid)
    explicit ConvexHull3D(const std::vector<Vec3f>& verts, const Vec3f& center = Vec3f(0.0f))
        : vertices(verts), position(center)
    {
        if (vertices.size() < 4) {
            assert(false && "ConvexHull3D requires at least 4 vertices");
            return;
        }
        compute_bounds();
        // NOTE: Full 3D convex hull computation (with faces/edges) is complex
        // For now, we store face_normals and edge_normals as empty placeholders
        // Actual hull computation will be done in Phase 1.2 subtask
        validate_invariants();
    }

    /// Add a vertex to the hull
    /// @param vertex Vertex position
    void add_vertex(const Vec3f& vertex) {
        vertices.push_back(vertex);
        if (vertices.size() >= 4) {
            compute_bounds();
            validate_invariants();
        }
    }

    /// Recompute centroid position
    void update_centroid() {
        position = Vec3f(0.0f);
        for (const auto& v : vertices) {
            position = position + v;
        }
        if (!vertices.empty()) {
            position = position / static_cast<float>(vertices.size());
        }
    }

    /// Validate hull invariants
    void validate_invariants() const {
        assert(vertices.size() >= 4 && "Hull must have at least 4 vertices");
        assert(bounding_radius > 0.0f && "Bounding radius must be positive");
    }

    // Shape3D interface implementation
    Vec3f support_point(const Vec3f& direction) const override {
        if (vertices.empty()) return Vec3f(0.0f);
        
        size_t best_idx = 0;
        float best_dot = vertices[0].dot(direction);
        
        for (size_t i = 1; i < vertices.size(); ++i) {
            float dot = vertices[i].dot(direction);
            if (dot > best_dot) {
                best_dot = dot;
                best_idx = i;
            }
        }
        
        return vertices[best_idx];
    }

    AABB get_aabb() const override {
        return bound;
    }

    float get_radius() const override {
        return bounding_radius;
    }

private:
    /// Compute bounding box and bounding sphere
    void compute_bounds() {
        if (vertices.empty()) {
            bound = AABB(Vec3f(0.0f), Vec3f(0.0f));
            bounding_radius = 0.0f;
            return;
        }

        Vec3f min_pt = vertices[0];
        Vec3f max_pt = vertices[0];
        
        for (const auto& v : vertices) {
            min_pt.x = std::min(min_pt.x, v.x);
            min_pt.y = std::min(min_pt.y, v.y);
            min_pt.z = std::min(min_pt.z, v.z);
            max_pt.x = std::max(max_pt.x, v.x);
            max_pt.y = std::max(max_pt.y, v.y);
            max_pt.z = std::max(max_pt.z, v.z);
        }

        bound = AABB(min_pt, max_pt);

        // Compute bounding sphere radius relative to the stored position
        // DO NOT update the position itself - it was set by the constructor parameter
        bounding_radius = 0.0f;
        for (const auto& v : vertices) {
            float dist = (v - position).length();
            bounding_radius = std::max(bounding_radius, dist);
        }
        // Add small margin for numerical stability
        bounding_radius *= 1.001f;
    }
};

}  // namespace phynity::physics::collision

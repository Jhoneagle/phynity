#pragma once

#include <core/physics/collision/support_function.hpp>
#include <core/math/vectors/vec3.hpp>
#include <vector>
#include <cmath>
#include <cstring>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Result of GJK algorithm: distance and closest points on both shapes
struct GJKResult {
    bool collision = false;              ///< true if shapes are overlapping (distance <= 0)
    float distance = 0.0f;               ///< Minimum distance between shapes (negative if penetrating)
    Vec3f closest_point_a = Vec3f(0.0f); ///< Closest point on shape A
    Vec3f closest_point_b = Vec3f(0.0f); ///< Closest point on shape B
    Vec3f closest_normal = Vec3f(0.0f, 1.0f, 0.0f); ///< Normal from A to B (direction of separation)
    int iterations = 0;                  ///< Number of iterations performed
};

/// Simplex: manages a set of points in the GJK algorithm
/// In 3D, can hold up to 4 points (tetrahedron)
/// In practice, we work with the simplex in Minkowski difference space
class SimplexGJK {
private:
    static constexpr int MAX_VERTICES = 4;
    static constexpr int MAX_VERTICES_2D = 3;

    struct Vertex {
        Vec3f point;           ///< Point in Minkowski difference space
        Vec3f support_a;       ///< Original support point on shape A
        Vec3f support_b;       ///< Original support point on shape B
    };

    Vertex vertices[MAX_VERTICES];
    int vertex_count = 0;
    bool is_2d = false;  // Set to true if all points have z=0

public:
    SimplexGJK() = default;

    /// Reset the simplex
    void reset() {
        vertex_count = 0;
        is_2d = false;
    }

    /// Add a vertex to the simplex
    /// @param point Point in Minkowski difference space
    /// @param support_a Support point on shape A
    /// @param support_b Support point on shape B
    /// @return true if vertex was added, false if simplex is full
    bool add_vertex(const Vec3f& point, const Vec3f& support_a, const Vec3f& support_b) {
        if (vertex_count >= MAX_VERTICES) {
            return false;
        }
        vertices[vertex_count].point = point;
        vertices[vertex_count].support_a = support_a;
        vertices[vertex_count].support_b = support_b;
        vertex_count++;

        // Check if all points are in 2D (z ≈ 0)
        if (vertex_count > 1) {
            is_2d = true;
            for (int i = 0; i < vertex_count; ++i) {
                if (std::abs(vertices[i].point.z) > 1e-6f) {
                    is_2d = false;
                    break;
                }
            }
        }

        return true;
    }

    /// Get the simplex vertices
    const Vertex* get_vertices() const {
        return vertices;
    }

    /// Get the number of vertices in the simplex
    int get_vertex_count() const {
        return vertex_count;
    }

    /// Get a single vertex
    const Vertex& get_vertex(int idx) const {
        return vertices[idx];
    }

    /// Check if origin is inside the simplex
    /// In 3D with 4 points, uses dot product tests with normals
    /// In 2D or with fewer points, approximates
    bool contains_origin(Vec3f& closest_point, Vec3f& closest_normal) const {
        if (vertex_count == 1) {
            closest_point = vertices[0].point;
            closest_normal = -closest_point.normalized();
            return false;
        }

        if (vertex_count == 2) {
            // Closest point on line segment
            const Vec3f& a = vertices[0].point;
            const Vec3f& b = vertices[1].point;
            Vec3f ab = b - a;
            float t = -a.dot(ab) / ab.dot(ab);
            t = std::max(0.0f, std::min(1.0f, t));
            closest_point = a + ab * t;
            closest_normal = -closest_point.normalized();
            return false;
        }

        if (vertex_count == 3) {
            // Closest point on triangle
            return closest_point_on_triangle(closest_point, closest_normal);
        }

        if (vertex_count == 4) {
            // Check if origin is inside tetrahedron
            // If not, find the closest feature (face, edge, or vertex)
            return closest_point_on_tetrahedron(closest_point, closest_normal);
        }

        return false;
    }

    /// Remove a vertex by index (shifts remaining vertices)
    void remove_vertex(int idx) {
        if (idx < 0 || idx >= vertex_count) return;
        for (int i = idx; i < vertex_count - 1; ++i) {
            vertices[i] = vertices[i + 1];
        }
        vertex_count--;
    }

private:
    /// Closest point on triangle ABC to origin
    bool closest_point_on_triangle(Vec3f& closest_point, Vec3f& closest_normal) const {
        const Vec3f& a = vertices[0].point;
        const Vec3f& b = vertices[1].point;
        const Vec3f& c = vertices[2].point;

        Vec3f ab = b - a;
        Vec3f ac = c - a;
        Vec3f normal = ab.cross(ac);

        // Check if origin projects inside the triangle
        float dist_to_plane = -a.dot(normal);
        if (std::abs(dist_to_plane) < 1e-6f) {
            // Origin is on the plane
            // Find closest point within the triangle
            Vec3f closest = closest_point_in_triangle(a, b, c);
            closest_point = closest;
            closest_normal = normal.normalized();
            return std::abs(closest.length()) < 1e-6f;
        }

        closest_point = a - normal * (a.dot(normal) / normal.dot(normal));
        closest_normal = -normal.normalized();
        return false;
    }

    /// Closest point on tetrahedron ABCD to origin
    bool closest_point_on_tetrahedron(Vec3f& closest_point, Vec3f& closest_normal) const {
        const Vec3f& a = vertices[0].point;
        const Vec3f& b = vertices[1].point;
        const Vec3f& c = vertices[2].point;
        const Vec3f& d = vertices[3].point;

        // Check if origin is inside tetrahedron using determinants
        // If inside, return origin with normal toward origin from closest face
        // Otherwise, find closest face/edge/vertex

        // For simplicity, find closest of the 4 faces
        float min_dist = 1e30f;
        Vec3f closest = a;
        Vec3f best_normal(0.0f, 1.0f, 0.0f);

        // Face ABC
        {
            Vec3f n = (b - a).cross(c - a);
            float d_n = a.dot(n);
            if (d_n < 0) n = -n;
            Vec3f p = a - n * (a.dot(n) / n.dot(n));
            if (is_point_in_triangle(p, a, b, c)) {
                float dist = p.length();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest = p;
                    best_normal = n.normalized();
                }
            }
        }

        // Face ABD
        {
            Vec3f n = (b - a).cross(d - a);
            float d_n = a.dot(n);
            if (d_n < 0) n = -n;
            Vec3f p = a - n * (a.dot(n) / n.dot(n));
            if (is_point_in_triangle(p, a, b, d)) {
                float dist = p.length();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest = p;
                    best_normal = n.normalized();
                }
            }
        }

        // Face ACD
        {
            Vec3f n = (c - a).cross(d - a);
            float d_n = a.dot(n);
            if (d_n < 0) n = -n;
            Vec3f p = a - n * (a.dot(n) / n.dot(n));
            if (is_point_in_triangle(p, a, c, d)) {
                float dist = p.length();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest = p;
                    best_normal = n.normalized();
                }
            }
        }

        // Face BCD
        {
            Vec3f n = (c - b).cross(d - b);
            float d_n = b.dot(n);
            if (d_n < 0) n = -n;
            Vec3f p = b - n * (b.dot(n) / n.dot(n));
            if (is_point_in_triangle(p, b, c, d)) {
                float dist = p.length();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest = p;
                    best_normal = n.normalized();
                }
            }
        }

        closest_point = closest;
        closest_normal = -best_normal;
        return min_dist < 1e-6f;
    }

    /// Check if point p is inside triangle ABC (barycentric method)
    bool is_point_in_triangle(const Vec3f& p, const Vec3f& a, const Vec3f& b, const Vec3f& c) const {
        Vec3f ab = b - a;
        Vec3f ac = c - a;
        Vec3f ap = p - a;

        float d00 = ab.dot(ab);
        float d01 = ab.dot(ac);
        float d11 = ac.dot(ac);
        float d20 = ap.dot(ab);
        float d21 = ap.dot(ac);

        float denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) < 1e-10f) return false;

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;

        return u >= -1e-5f && v >= -1e-5f && w >= -1e-5f;
    }

    /// Closest point within triangle ABC
    Vec3f closest_point_in_triangle(const Vec3f& a, const Vec3f& b, const Vec3f& c) const {
        // Project origin onto plane, then project onto triangle edges
        Vec3f ab = b - a;
        Vec3f ac = c - a;
        Vec3f normal = ab.cross(ac);

        // Project origin to plane
        float t = a.dot(normal) / normal.dot(normal);
        Vec3f proj = Vec3f(0.0f) - normal * t;

        // Now find closest point in triangle using barycentric coordinates
        Vec3f ap = proj - a;
        float d00 = ab.dot(ab);
        float d01 = ab.dot(ac);
        float d11 = ac.dot(ac);
        float d20 = ap.dot(ab);
        float d21 = ap.dot(ac);

        float denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) < 1e-10f) {
            // Degenerate triangle, return closest vertex
            float da = a.squaredLength();
            float db = b.squaredLength();
            float dc = c.squaredLength();
            if (da <= db && da <= dc) return a;
            if (db <= dc) return b;
            return c;
        }

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;

        // Clamp to barycentric bounds
        v = std::max(0.0f, std::min(1.0f, v));
        w = std::max(0.0f, std::min(1.0f, w));
        u = std::max(0.0f, std::min(1.0f, 1.0f - v - w));

        return a * u + b * v + c * w;
    }
};

/// GJK (Gilbert-Johnson-Keerthi) Algorithm: computes minimum distance between two convex shapes
class GJKSolver {
public:
    // Configuration
    static constexpr float DISTANCE_EPSILON = 1e-6f;  ///< Convergence threshold for distance
    static constexpr float SUPPORT_EPSILON = 1e-6f;   ///< Threshold for detecting no progress
    static constexpr int MAX_ITERATIONS = 64;         ///< Maximum iteration count

    /// Solve for minimum distance between two shapes (in 3D)
    /// @param shape_a Support function for shape A
    /// @param shape_b Support function for shape B
    /// @return GJK result with distance and closest points
    static GJKResult solve(const SupportFunction& shape_a, const SupportFunction& shape_b) {
        GJKResult result;
        SimplexGJK simplex;

        // Initial direction: from A's origin to B's origin
        Vec3f initial_dir = shape_b.get_origin() - shape_a.get_origin();
        if (initial_dir.squaredLength() < 1e-10f) {
            // Shapes are at same position
            initial_dir = Vec3f(1.0f, 0.0f, 0.0f);
        }
        initial_dir = initial_dir.normalized();

        // Iteration loop
        for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
            // Get support point of Minkowski difference in current direction
            Vec3f support_a = shape_a.get_support_point(initial_dir);
            Vec3f support_b = shape_b.get_support_point(-initial_dir);
            Vec3f minkowski_point = support_a - support_b;

            // Add vertex to simplex
            if (!simplex.add_vertex(minkowski_point, support_a, support_b)) {
                // Simplex is full (4 vertices in 3D)
                break;
            }

            // Check if we've made progress in the direction we're looking
            float dot_with_dir = minkowski_point.dot(initial_dir);
            if (dot_with_dir < SUPPORT_EPSILON && simplex.get_vertex_count() > 1) {
                // We're not making progress and have enough points
                break;
            }

            // Find closest point to origin on the simplex
            Vec3f closest_point;
            Vec3f closest_normal;
            if (simplex.contains_origin(closest_point, closest_normal)) {
                // Origin is inside the simplex: shapes are overlapping
                result.collision = true;
                result.distance = 0.0f;  // Penetrating
                // Ensure we have a proper collision normal
                if (closest_normal.squaredLength() > 1e-10f) {
                    result.closest_normal = closest_normal.normalized();
                } else {
                    result.closest_normal = initial_dir;
                }
                break;
            }

            // Update distance and search direction
            result.distance = closest_point.length();
            if (result.distance < DISTANCE_EPSILON) {
                result.collision = true;
                result.closest_normal = initial_dir;
                break;
            }

            // New direction toward origin
            Vec3f new_dir = -closest_point;
            float new_dir_len = new_dir.length();
            if (new_dir_len < DISTANCE_EPSILON) {
                // No direction to search
                break;
            }
            initial_dir = new_dir / new_dir_len;

            result.iterations = iter + 1;
        }

        // Extract closest points on original shapes
        extract_closest_points(shape_a, shape_b, simplex, result);

        return result;
    }

private:
    /// Extract the closest points on the original shapes from the simplex
    static void extract_closest_points(const SupportFunction& shape_a,
                                       const SupportFunction& shape_b,
                                       const SimplexGJK& simplex,
                                       GJKResult& result) {
        // Use the support points from the simplex vertices
        if (simplex.get_vertex_count() == 0) {
            result.closest_point_a = shape_a.get_origin();
            result.closest_point_b = shape_b.get_origin();
            return;
        }

        // Use the first vertex's support points
        const auto& v = simplex.get_vertex(0);
        result.closest_point_a = v.support_a;
        result.closest_point_b = v.support_b;
    }
};

}  // namespace phynity::physics::collision

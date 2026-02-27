#pragma once

#include <core/physics/collision/convex_hull.hpp>
#include <core/physics/collision/contact_manifold.hpp>
#include <core/math/vectors/vec2.hpp>
#include <core/math/vectors/vec3.hpp>
#include <optional>
#include <vector>
#include <cmath>
#include <algorithm>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec2f;
using phynity::math::vectors::Vec3f;

/// Result of a SAT axis test
struct SATAxisTest {
    float penetration_depth = 0.0f;  ///< Minimum penetration depth along the axis
    Vec3f axis = Vec3f(0.0f);        ///< The separating axis direction (normalized)
    bool is_separated = false;       ///< true if objects are separated on this axis
};

/// Result of SAT-based collision detection
struct SATCollisionResult {
    bool is_colliding = false;           ///< true if objects are overlapping
    float penetration_depth = 0.0f;      ///< Depth of penetration
    Vec3f collision_normal = Vec3f(0.0f);///< Normal vector (from A to B)
    Vec3f contact_point = Vec3f(0.0f);   ///< Approximate contact point
};

/// Separating Axis Theorem (SAT) collision detector for convex shapes
class SATSolver {
public:
    /// Test two 2D convex hulls for collision using SAT
    /// @param hull_a First convex hull (2D)
    /// @param hull_b Second convex hull (2D)
    /// @param transform_a Position offset of hull_a (2D)
    /// @param transform_b Position offset of hull_b (2D)
    /// @return Collision result with penetration info
    static SATCollisionResult test_convex_hull_2d(
        const ConvexHull2D& hull_a,
        const ConvexHull2D& hull_b,
        const Vec2f& transform_a = Vec2f(0.0f),
        const Vec2f& transform_b = Vec2f(0.0f)
    );

    /// Test two 3D convex hulls for collision using SAT
    /// @param hull_a First convex hull (3D)
    /// @param hull_b Second convex hull (3D)
    /// @param transform_a Position/transform of hull_a
    /// @param transform_b Position/transform of hull_b
    /// @return Collision result with penetration info
    static SATCollisionResult test_convex_hull_3d(
        const ConvexHull3D& hull_a,
        const ConvexHull3D& hull_b,
        const Vec3f& transform_a = Vec3f(0.0f),
        const Vec3f& transform_b = Vec3f(0.0f)
    );

private:
    /// Numerical tolerance for floating-point comparisons
    static constexpr float EPSILON = 1e-6f;

    /// Test a single axis for separation between two 2D hulls
    /// @param hull_a First hull
    /// @param hull_b Second hull
    /// @param axis Test axis (should be normalized)
    /// @param transform_a Absolute world position for hull_a
    /// @param transform_b Absolute world position for hull_b
    /// @return Test result; is_separated=true means no overlap on this axis
    static SATAxisTest test_axis_2d(
        const ConvexHull2D& hull_a,
        const ConvexHull2D& hull_b,
        const Vec3f& axis,
        const Vec2f& transform_a,
        const Vec2f& transform_b
    );

    /// Test a single axis for separation between two 3D hulls
    static SATAxisTest test_axis_3d(
        const ConvexHull3D& hull_a,
        const ConvexHull3D& hull_b,
        const Vec3f& axis,
        const Vec3f& offset
    );

    /// Project all vertices of a 2D hull onto an axis
    /// @param hull The convex hull
    /// @param axis Projection axis (normalized)
    /// @param offset Positional offset (2D)
    /// @return {min_projection, max_projection}
    static std::pair<float, float> project_hull_2d(
        const ConvexHull2D& hull,
        const Vec3f& axis,
        const Vec2f& offset
    );

    /// Project all vertices of a 3D hull onto an axis
    static std::pair<float, float> project_hull_3d(
        const ConvexHull3D& hull,
        const Vec3f& axis,
        const Vec3f& offset
    );

    /// Find closest points on two line segments for contact point estimation
    static Vec3f find_contact_point_2d(
        const ConvexHull2D& hull_a,
        const ConvexHull2D& hull_b,
        const Vec3f& normal,
        const Vec2f& offset
    );

    /// Find contact point on a 3D edge
    static Vec3f find_contact_point_3d(
        const ConvexHull3D& hull_a,
        const ConvexHull3D& hull_b,
        const Vec3f& normal,
        const Vec3f& offset
    );

    /// Normalize a vector, handle zero-length case
    static Vec3f safe_normalize(const Vec3f& v);

    /// Check if two floats are approximately equal
    static bool approximately_equal(float a, float b, float eps = EPSILON);
};

// ============================================================================
// Implementation
// ============================================================================

inline Vec3f SATSolver::safe_normalize(const Vec3f& v) {
    float len = v.length();
    if (len > EPSILON) {
        return v / len;
    }
    return v;  // Return zero vector if degenerate
}

inline bool SATSolver::approximately_equal(float a, float b, float eps) {
    return std::abs(a - b) < eps;
}

inline std::pair<float, float> SATSolver::project_hull_2d(
    const ConvexHull2D& hull,
    const Vec3f& axis,
    const Vec2f& offset
) {
    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();
    
    // World position = LOCAL vertex + TRANSFORM offset
    // We ignore hull.position because transform already specifies absolute world position
    Vec3f offset_3d(offset.x, offset.y, 0.0f);

    for (const auto& vertex : hull.vertices) {
        Vec3f v_3d(vertex.x, vertex.y, 0.0f);
        Vec3f world_pos = v_3d + offset_3d;
        float proj = world_pos.dot(axis);
        
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }

    return {min_proj, max_proj};
}

inline std::pair<float, float> SATSolver::project_hull_3d(
    const ConvexHull3D& hull,
    const Vec3f& axis,
    const Vec3f& offset
) {
    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();

    for (const auto& vertex : hull.vertices) {
        float proj = (vertex + offset).dot(axis);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }

    return {min_proj, max_proj};
}

inline SATAxisTest SATSolver::test_axis_2d(
    const ConvexHull2D& hull_a,
    const ConvexHull2D& hull_b,
    const Vec3f& axis,
    const Vec2f& transform_a,
    const Vec2f& transform_b
) {
    SATAxisTest result;
    result.axis = axis;

    // Project both hulls at their absolute world positions
    auto [min_a, max_a] = project_hull_2d(hull_a, axis, transform_a);
    auto [min_b, max_b] = project_hull_2d(hull_b, axis, transform_b);

    // Check for separation
    if (max_a < min_b - EPSILON || max_b < min_a - EPSILON) {
        result.is_separated = true;
        return result;
    }

    // Compute penetration depth
    float overlap1 = max_a - min_b;  // How far hull_a extends past hull_b's min
    float overlap2 = max_b - min_a;  // How far hull_b extends past hull_a's min

    result.penetration_depth = std::min(overlap1, overlap2);
    result.is_separated = false;

    return result;
}

inline SATAxisTest SATSolver::test_axis_3d(
    const ConvexHull3D& hull_a,
    const ConvexHull3D& hull_b,
    const Vec3f& axis,
    const Vec3f& offset
) {
    SATAxisTest result;
    result.axis = axis;

    auto [min_a, max_a] = project_hull_3d(hull_a, axis, Vec3f(0.0f));
    auto [min_b, max_b] = project_hull_3d(hull_b, axis, offset);

    // Check for separation
    if (max_a < min_b - EPSILON || max_b < min_a - EPSILON) {
        result.is_separated = true;
        return result;
    }

    // Compute penetration depth
    float overlap1 = max_a - min_b;
    float overlap2 = max_b - min_a;

    result.penetration_depth = std::min(overlap1, overlap2);
    result.is_separated = false;

    return result;
}

inline Vec3f SATSolver::find_contact_point_2d(
    const ConvexHull2D& hull_a,
    const ConvexHull2D& hull_b,
    const Vec3f& normal,
    const Vec2f& offset
) {
    // Simple approach: find vertices from each hull that are closest along the normal
    Vec3f contact(0.0f);
    float best_dist = std::numeric_limits<float>::lowest();
    Vec3f offset_3d(offset.x, offset.y, 0.0f);

    // Find vertices from hull_a that are furthest along -normal (moving into hull_b)
    for (const auto& v : hull_a.vertices) {
        Vec3f v_3d(v.x, v.y, 0.0f);
        float dist = v_3d.dot(-normal);
        if (dist > best_dist) {
            best_dist = dist;
            contact = v_3d;
        }
    }

    // Average with similar search in hull_b
    best_dist = std::numeric_limits<float>::lowest();
    Vec3f contact_b(0.0f);
    for (const auto& v : hull_b.vertices) {
        Vec3f v_3d(v.x, v.y, 0.0f);
        float dist = (v_3d + offset_3d).dot(normal);
        if (dist > best_dist) {
            best_dist = dist;
            contact_b = v_3d + offset_3d;
        }
    }

    return (contact + contact_b) * 0.5f;
}

inline Vec3f SATSolver::find_contact_point_3d(
    const ConvexHull3D& hull_a,
    const ConvexHull3D& hull_b,
    const Vec3f& normal,
    const Vec3f& offset
) {
    // Simple approach: find vertices that are closest along the normal
    Vec3f contact(0.0f);
    float best_dist = std::numeric_limits<float>::lowest();

    for (const auto& v : hull_a.vertices) {
        float dist = v.dot(-normal);
        if (dist > best_dist) {
            best_dist = dist;
            contact = v;
        }
    }

    best_dist = std::numeric_limits<float>::lowest();
    Vec3f contact_b(0.0f);
    for (const auto& v : hull_b.vertices) {
        float dist = (v + offset).dot(normal);
        if (dist > best_dist) {
            best_dist = dist;
            contact_b = v + offset;
        }
    }

    return (contact + contact_b) * 0.5f;
}

inline SATCollisionResult SATSolver::test_convex_hull_2d(
    const ConvexHull2D& hull_a,
    const ConvexHull2D& hull_b,
    const Vec2f& transform_a,
    const Vec2f& transform_b
) {
    SATCollisionResult result;

    // Quick AABB rejection
    AABB aabb_a = hull_a.get_aabb();
    AABB aabb_b = hull_b.get_aabb();
    // Apply transforms to AABBs (hull.position is ignored, using transform only)
    Vec3f transform_a_3d(transform_a.x, transform_a.y, 0.0f);
    Vec3f transform_b_3d(transform_b.x, transform_b.y, 0.0f);
    aabb_a.min = aabb_a.min + transform_a_3d;
    aabb_a.max = aabb_a.max + transform_a_3d;
    aabb_b.min = aabb_b.min + transform_b_3d;
    aabb_b.max = aabb_b.max + transform_b_3d;

    // Use 2D-specific overlap check for 2D shapes (ignores z-axis)
    if (!aabb_a.overlaps_2d(aabb_b)) {
        result.is_colliding = false;
        return result;
    }

    // SAT testing
    float min_penetration = std::numeric_limits<float>::max();
    Vec3f best_axis(0.0f);

    // Test normals from hull_a
    for (const auto& normal : hull_a.normals) {
        Vec3f axis_3d(normal.x, normal.y, 0.0f);
        auto test = test_axis_2d(hull_a, hull_b, axis_3d, transform_a, transform_b);
        
        if (test.is_separated) {
            result.is_colliding = false;
            return result;
        }

        if (test.penetration_depth < min_penetration) {
            min_penetration = test.penetration_depth;
            best_axis = axis_3d;
        }
    }

    // Test normals from hull_b
    for (const auto& normal : hull_b.normals) {
        Vec3f axis_3d(normal.x, normal.y, 0.0f);
        auto test = test_axis_2d(hull_a, hull_b, axis_3d, transform_a, transform_b);
        
        if (test.is_separated) {
            result.is_colliding = false;
            return result;
        }
        if (test.penetration_depth < min_penetration) {
            min_penetration = test.penetration_depth;
            best_axis = axis_3d;
        }
    }

    // Epsilon check: touching shapes (penetration ~= 0) should not count as colliding
    constexpr float COLLISION_EPSILON = 1e-5f;
    if (min_penetration <= COLLISION_EPSILON) {
        result.is_colliding = false;
        result.penetration_depth = 0.0f;
        return result;
    }

    result.is_colliding = true;
    result.penetration_depth = min_penetration;
    result.collision_normal = safe_normalize(best_axis);
    Vec2f offset = transform_b - transform_a;
    result.contact_point = find_contact_point_2d(hull_a, hull_b, result.collision_normal, offset);

    return result;
}

inline SATCollisionResult SATSolver::test_convex_hull_3d(
    const ConvexHull3D& hull_a,
    const ConvexHull3D& hull_b,
    const Vec3f& transform_a,
    const Vec3f& transform_b
) {
    SATCollisionResult result;

    // Quick AABB rejection
    AABB aabb_a = hull_a.get_aabb();
    AABB aabb_b = hull_b.get_aabb();
    // Include both hull positions and additional transforms
    aabb_a.min = aabb_a.min + hull_a.position + transform_a;
    aabb_a.max = aabb_a.max + hull_a.position + transform_a;
    aabb_b.min = aabb_b.min + hull_b.position + transform_b;
    aabb_b.max = aabb_b.max + hull_b.position + transform_b;

    if (!aabb_a.overlaps(aabb_b)) {
        result.is_colliding = false;
        return result;
    }

    // SAT testing
    float min_penetration = std::numeric_limits<float>::max();
    Vec3f best_axis(0.0f);
    Vec3f offset = transform_b - transform_a;

    // Test face normals from hull_a
    for (const auto& normal : hull_a.face_normals) {
        auto test = test_axis_3d(hull_a, hull_b, normal, offset);
        
        if (test.is_separated) {
            result.is_colliding = false;
            return result;
        }

        if (test.penetration_depth < min_penetration) {
            min_penetration = test.penetration_depth;
            best_axis = normal;
        }
    }

    // Test face normals from hull_b
    for (const auto& normal : hull_b.face_normals) {
        auto test = test_axis_3d(hull_a, hull_b, normal, offset);
        
        if (test.is_separated) {
            result.is_colliding = false;
            return result;
        }

        if (test.penetration_depth < min_penetration) {
            min_penetration = test.penetration_depth;
            best_axis = normal;
        }
    }

    // Test edge-edge normals (cross products of edges from both hulls)
    for (size_t i = 0; i < std::min(hull_a.vertices.size(), size_t(6)); ++i) {
        for (size_t j = 0; j < std::min(hull_b.vertices.size(), size_t(6)); ++j) {
            // This is a simplified version; full implementation needs actual edge lists
            // For now, skip edge-edge tests
        }
    }

    // Epsilon check: touching shapes (penetration ~= 0) should not count as colliding
    constexpr float COLLISION_EPSILON = 1e-5f;
    if (min_penetration <= COLLISION_EPSILON) {
        result.is_colliding = false;
        result.penetration_depth = 0.0f;
        return result;
    }

    result.is_colliding = true;
    result.penetration_depth = min_penetration;
    result.collision_normal = safe_normalize(best_axis);
    result.contact_point = find_contact_point_3d(hull_a, hull_b, result.collision_normal, offset);

    return result;
}

}  // namespace phynity::physics::collision

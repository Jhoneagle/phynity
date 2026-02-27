#pragma once

#include <core/physics/collision/support_function.hpp>
#include <core/physics/collision/gjk_solver.hpp>
#include <core/math/vectors/vec3.hpp>
#include <vector>
#include <cmath>
#include <limits>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Result of EPA algorithm: contact information for penetrating shapes
struct EPAResult {
    Vec3f contact_point = Vec3f(0.0f);     ///< Contact point position
    Vec3f contact_normal = Vec3f(0.0f, 1.0f, 0.0f); ///< Contact normal (A to B direction)
    float penetration_depth = 0.0f;        ///< Penetration depth (positive = penetrating)
    int iterations = 0;                    ///< Number of iterations performed
};

/// EPA (Expanding Polytope Algorithm): computes contact normal and penetration depth
/// Simplified version that works for basic overlapping shapes
class EPASolver {
public:
    static constexpr float EPA_EPSILON = 1e-5f;
    static constexpr int MAX_ITERATIONS = 20;

    /// Solve EPA to find contact information from a GJK result
    /// @param shape_a Support function for shape A
    /// @param shape_b Support function for shape B
    /// @param gjk_result The result from GJK algorithm (must have collision = true)
    /// @return EPA result with contact point, normal, and penetration depth
    static EPAResult solve(const SupportFunction& shape_a,
                          const SupportFunction& shape_b,
                          const GJKResult& gjk_result) {
        EPAResult result;

        // Only run EPA if shapes are actually colliding
        if (!gjk_result.collision) {
            result.penetration_depth = gjk_result.distance;
            result.contact_normal = gjk_result.closest_normal;
            return result;
        }

        // For overlapping shapes, estimate penetration depth by sampling
        // support points along the collision normal
        Vec3f normal = gjk_result.closest_normal;
        if (normal.squaredLength() < 1e-10f) {
            normal = Vec3f(1.0f, 0.0f, 0.0f);
        }
        normal = normal.normalized();

        // Get support points along the normal direction
        Vec3f support_a = shape_a.get_support_point(normal);
        Vec3f support_b = shape_b.get_support_point(-normal);
        
        // Penetration depth is the signed distance along the normal
        float penetration_depth = (support_a - support_b).dot(normal);
        
        // In case of numerical issues, ensure we return a positive depth for collisions
        result.penetration_depth = std::max(0.0f, penetration_depth);
        result.contact_normal = normal;
        result.contact_point = shape_a.get_origin();
        result.iterations = 1;

        return result;
    }
};

}  // namespace phynity::physics::collision

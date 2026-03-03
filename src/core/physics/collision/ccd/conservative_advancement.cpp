#include <core/physics/collision/ccd/conservative_advancement.hpp>

#include <algorithm>
#include <cmath>

namespace phynity::physics::collision::ccd {

TimeOfImpactResult ConservativeAdvancement::solve_sphere_sphere(
    const SweptSphereSolver::Sphere& sphere_a,
    const SweptSphereSolver::Sphere& sphere_b,
    float max_time,
    int max_iterations,
    float separation_tolerance,
    int root_refinement_iterations
) {
    constexpr float kEpsilon = 1e-6f;

    auto finite_vec = [](const Vec3f& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    };

    if (!std::isfinite(max_time) || max_time <= 0.0f) {
        return TimeOfImpactResult();
    }

    if (!std::isfinite(sphere_a.radius) || !std::isfinite(sphere_b.radius) ||
        sphere_a.radius <= 0.0f || sphere_b.radius <= 0.0f) {
        return TimeOfImpactResult();
    }

    if (!finite_vec(sphere_a.position) || !finite_vec(sphere_b.position) ||
        !finite_vec(sphere_a.velocity) || !finite_vec(sphere_b.velocity)) {
        return TimeOfImpactResult();
    }

    float clamped_max_time = std::max(max_time, kEpsilon);
    int clamped_iterations = std::max(max_iterations, 1);
    int refinement_iterations = std::max(root_refinement_iterations, 0);
    float tolerance = std::max(separation_tolerance, 1e-6f);

    Vec3f relative_velocity = sphere_b.velocity - sphere_a.velocity;
    float combined_radius = sphere_a.radius + sphere_b.radius;

    float normalized_time = 0.0f;
    float previous_normalized_time = 0.0f;

    auto signed_distance = [&](float query_normalized_time) -> float {
        float physical_time = query_normalized_time * clamped_max_time;
        Vec3f pos_a = sphere_a.position + sphere_a.velocity * physical_time;
        Vec3f pos_b = sphere_b.position + sphere_b.velocity * physical_time;
        return (pos_b - pos_a).length() - combined_radius;
    };

    for (int iter = 0; iter < clamped_iterations; ++iter) {
        float physical_time = normalized_time * clamped_max_time;
        Vec3f pos_a = sphere_a.position + sphere_a.velocity * physical_time;
        Vec3f pos_b = sphere_b.position + sphere_b.velocity * physical_time;

        Vec3f delta = pos_b - pos_a;
        float distance = delta.length();
        float separation = distance - combined_radius;

        Vec3f normal = (distance > kEpsilon) ? (delta / distance) : Vec3f(1.0f, 0.0f, 0.0f);

        if (separation <= tolerance) {
            float refined_normalized_time = normalized_time;

            if (refinement_iterations > 0 && previous_normalized_time < normalized_time) {
                float lo = previous_normalized_time;
                float hi = normalized_time;

                for (int root_iter = 0; root_iter < refinement_iterations; ++root_iter) {
                    float mid = 0.5f * (lo + hi);
                    float f_mid = signed_distance(mid);

                    if (f_mid > 0.0f) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }

                refined_normalized_time = 0.5f * (lo + hi);
            }

            float refined_physical_time = refined_normalized_time * clamped_max_time;
            Vec3f refined_pos_a = sphere_a.position + sphere_a.velocity * refined_physical_time;
            Vec3f refined_pos_b = sphere_b.position + sphere_b.velocity * refined_physical_time;
            Vec3f refined_delta = refined_pos_b - refined_pos_a;
            float refined_distance = refined_delta.length();
            Vec3f refined_normal =
                (refined_distance > kEpsilon) ? (refined_delta / refined_distance) : Vec3f(1.0f, 0.0f, 0.0f);

            Vec3f contact_point = refined_pos_a + refined_normal * sphere_a.radius;
            return TimeOfImpactResult(refined_normalized_time, contact_point, refined_normal, relative_velocity);
        }

        float rel_normal_speed = relative_velocity.dot(normal);
        if (rel_normal_speed >= -kEpsilon) {
            return TimeOfImpactResult();
        }

        float advance_physical = separation / (-rel_normal_speed);
        advance_physical = std::max(advance_physical, kEpsilon);
        float advance_normalized = advance_physical / clamped_max_time;

        previous_normalized_time = normalized_time;
        normalized_time += advance_normalized;

        if (normalized_time > 1.0f + kEpsilon) {
            return TimeOfImpactResult();
        }
    }

    return TimeOfImpactResult();
}

}  // namespace phynity::physics::collision::ccd

#pragma once

#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/collision/shapes/sphere_collider.hpp>
#include <core/math/vectors/vec3.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

namespace phynity::physics::collision {

using phynity::math::vectors::Vec3f;

/// Configuration for PGS solver behavior
struct PGSConfig {
    int max_iterations = 4;              ///< Maximum number of iterations
    float convergence_threshold = 1e-5f; ///< Threshold for early termination
    float friction_coefficient = 0.2f;   ///< Default friction coefficient
    float restitution_default = 0.0f;    ///< Default coefficient of restitution
};

/// Projected Gauss-Seidel solver for iterative constraint resolution
/// Solves contact constraints by iteratively applying impulses to resolve
/// penetration and velocity constraints. Supports friction and warm-start.
class PGSSolver {
public:
    /// Solve contact constraints using PGS iteration
    /// @param manifolds Vector of contact manifolds to resolve
    /// @param colliders Vector of colliders (indexed by object_id)
    /// @param config PGS solver configuration
    /// @return Vector of applied impulses (one per contact)
    static std::vector<Vec3f> solve(
        const std::vector<ContactManifold>& manifolds,
        std::vector<SphereCollider>& colliders,
        const PGSConfig& config = PGSConfig()
    ) {
        if (manifolds.empty() || colliders.empty()) {
            return std::vector<Vec3f>();
        }

        // Accumulated impulses for each contact (what we return)
        std::vector<Vec3f> accumulated_impulses(manifolds.size(), Vec3f(0.0f));

        // Phase 1: Apply warm-start impulses from previous frame
        for (size_t i = 0; i < manifolds.size(); ++i) {
            if (!manifolds[i].is_valid()) continue;
            if (manifolds[i].object_a_id >= colliders.size() || 
                manifolds[i].object_b_id >= colliders.size()) continue;

            const Vec3f& prev_impulse = manifolds[i].previous_impulse;
            if (prev_impulse.length() > 1e-10f) {
                apply_impulse(
                    colliders[manifolds[i].object_a_id],
                    colliders[manifolds[i].object_b_id],
                    prev_impulse
                );
                accumulated_impulses[i] = prev_impulse;
            }
        }

        // Phase 2: Main PGS iteration loop
        for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
            float max_impulse_change = 0.0f;

            // Process each contact
            for (size_t i = 0; i < manifolds.size(); ++i) {
                const ContactManifold& manifold = manifolds[i];
                
                if (!manifold.is_valid()) {
                    continue;
                }

                // Get colliders
                if (manifold.object_a_id >= colliders.size() || manifold.object_b_id >= colliders.size()) {
                    continue;
                }

                SphereCollider& collider_a = colliders[manifold.object_a_id];
                SphereCollider& collider_b = colliders[manifold.object_b_id];

                // Compute the impulse needed at this contact with current velocities
                Vec3f delta_impulse = compute_contact_impulse(
                    manifold, collider_a, collider_b, config, accumulated_impulses[i]
                );

                // Apply the delta impulse
                apply_impulse(collider_a, collider_b, delta_impulse);
                
                // Track change for convergence check
                max_impulse_change = std::max(max_impulse_change, delta_impulse.length());

                // Accumulate impulse
                accumulated_impulses[i] += delta_impulse;
            }

            // Check for convergence
            if (max_impulse_change < config.convergence_threshold) {
                break;
            }
        }

        return accumulated_impulses;
    }

    /// Solve with adaptive iterations based on constraint count
    /// Adds more iterations for larger contact sets
    static std::vector<Vec3f> solve_adaptive(
        const std::vector<ContactManifold>& manifolds,
        std::vector<SphereCollider>& colliders,
        const PGSConfig& config = PGSConfig()
    ) {
        // Scale iterations with contact count
        PGSConfig adaptive_config = config;
        int contact_count = 0;
        for (const auto& m : manifolds) {
            if (m.is_valid()) contact_count++;
        }
        
        // Add iterations for larger systems: 4 for 1-10 contacts, 8 for 11-50, 16 for 50+
        if (contact_count <= 10) {
            adaptive_config.max_iterations = 4;
        } else if (contact_count <= 50) {
            adaptive_config.max_iterations = 8;
        } else {
            adaptive_config.max_iterations = 16;
        }

        return solve(manifolds, colliders, adaptive_config);
    }

private:
    /// Compute the incremental impulse to apply at a contact point
    /// This solves the constraint: resolve relative velocity and penetration
    /// The impulse is incremental - what needs to be added to what's already been applied
    static Vec3f compute_contact_impulse(
        const ContactManifold& manifold,
        const SphereCollider& collider_a,
        const SphereCollider& collider_b,
        const PGSConfig& config,
        const Vec3f& accumulated_impulse
    ) {
        const ContactPoint& contact = manifold.contact;

        // Compute inverse masses
        float inv_m1 = collider_a.inverse_mass;
        float inv_m2 = collider_b.inverse_mass;
        float inv_sum = inv_m1 + inv_m2;

        // Skip if both objects are static
        if (inv_sum <= 0.0f) {
            return Vec3f(0.0f);
        }

        // Compute relative velocity at contact point  
        Vec3f relative_velocity = collider_b.velocity - collider_a.velocity;
        float velocity_along_normal = relative_velocity.dot(contact.normal);

        // Compute restitution
        float restitution = std::min(collider_a.restitution, collider_b.restitution);

        // Compute what velocity we want along the normal
        // If already separating (v > 0), we're done - no impulse needed
        // Otherwise, apply impulse to achieve v = 0 (or v > 0 if restitution)
        float target_velocity = -restitution * velocity_along_normal;

        // Apply Baumgarte stabilization for penetration
        float baumgarte_scalar = 0.2f;
        float dt = 1.0f / 60.0f;
        float penetration_bias = 0.0f;
        if (contact.penetration > 0.0f) {
            penetration_bias = (baumgarte_scalar / dt) * contact.penetration;
        }

        // Compute the normal impulse magnitude needed
        // J_n = (target_velocity + penetration_bias) / inv_sum
        float normal_impulse_magnitude = (target_velocity + penetration_bias) / inv_sum;

        // Get the current accumulated normal impulse (scalar component along normal)
        float current_normal_impulse = accumulated_impulse.dot(contact.normal);

        // Clamp total normal impulse to [0, max]
        float new_normal_impulse = std::max(0.0f, current_normal_impulse + normal_impulse_magnitude);
        float delta_normal_impulse = new_normal_impulse - current_normal_impulse;

        // Compute normal impulse vector
        Vec3f normal_impulse = contact.normal * delta_normal_impulse;

        // Compute friction impulse (tangential constraints)
        Vec3f tangent = compute_tangent_direction(contact.normal, relative_velocity);
        Vec3f friction_impulse = Vec3f(0.0f);

        if (tangent.squaredLength() > 1e-10f && new_normal_impulse > 1e-10f) {
            // Current tangential impulse from accumulated
            Vec3f current_tangent_impulse = accumulated_impulse - contact.normal * current_normal_impulse;
            float current_tangent_magnitude = current_tangent_impulse.dot(tangent);

            // Velocity along tangent direction
            float velocity_along_tangent = relative_velocity.dot(tangent);
            
            // Tangential impulse needed to bring velocity to zero
            float friction_coefficient = config.friction_coefficient;
            float max_friction_impulse = friction_coefficient * new_normal_impulse;
            
            float tangent_impulse_magnitude = -velocity_along_tangent / inv_sum;
            float new_tangent_impulse = std::clamp(
                current_tangent_magnitude + tangent_impulse_magnitude,
                -max_friction_impulse,
                max_friction_impulse
            );
            float delta_tangent_impulse = new_tangent_impulse - current_tangent_magnitude;
            friction_impulse = tangent * delta_tangent_impulse;
        }

        // Return the total delta impulse (normal + friction)
        return normal_impulse + friction_impulse;
    }

    /// Compute a tangent direction (perpendicular to normal)
    static Vec3f compute_tangent_direction(const Vec3f& normal, const Vec3f& relative_velocity) {
        // Project relative velocity onto tangent plane (perpendicular to normal)
        Vec3f tangent_velocity = relative_velocity - normal * relative_velocity.dot(normal);
        
        if (tangent_velocity.squaredLength() > 1e-10f) {
            return tangent_velocity.normalized();
        }
        
        // If no tangent velocity, pick an arbitrary perpendicular direction
        // Find a direction not parallel to normal
        Vec3f arbitrary = (std::abs(normal.x) < 0.9f) ? Vec3f(1.0f, 0.0f, 0.0f) : Vec3f(0.0f, 1.0f, 0.0f);
        return normal.cross(arbitrary).normalized();
    }

    /// Apply an impulse to both colliders
    static void apply_impulse(
        SphereCollider& collider_a,
        SphereCollider& collider_b,
        const Vec3f& impulse
    ) {
        float inv_m1 = collider_a.inverse_mass;
        float inv_m2 = collider_b.inverse_mass;

        collider_a.velocity -= impulse * inv_m1;
        collider_b.velocity += impulse * inv_m2;
    }
};

}  // namespace phynity::physics::collision

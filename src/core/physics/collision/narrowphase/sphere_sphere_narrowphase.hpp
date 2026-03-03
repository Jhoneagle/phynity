#pragma once

#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/collision/shapes/sphere_collider.hpp>
#include <core/physics/common/physics_constants.hpp>
#include <core/physics/common/ccd_config.hpp>
#include <core/physics/collision/ccd/swept_sphere.hpp>
#include <core/physics/collision/ccd/conservative_advancement.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <algorithm>

namespace phynity::physics::collision {

using phynity::physics::constants::COLLISION_EPSILON;
using phynity::math::vectors::Vec3f;
using phynity::physics::collision::ccd::SweptSphereSolver;
using phynity::physics::collision::ccd::TimeOfImpactResult;
using phynity::physics::collision::ccd::ConservativeAdvancement;

/// Narrowphase collision detection for sphere-sphere contacts
/// Generates contact manifolds from geometric overlap.
/// Works with any objects that provide SphereCollider properties.
class SphereSpherNarrowphase {
public:
    /// Detect collision between two spheres
    /// @param collider_a First colliding object
    /// @param collider_b Second colliding object
    /// @param index_a Index or ID of first object
    /// @param index_b Index or ID of second object
    /// @return Contact manifold if collision detected, otherwise invalid manifold
    static ContactManifold detect(
        const SphereCollider& collider_a,
        const SphereCollider& collider_b,
        size_t index_a,
        size_t index_b
    ) {
        ContactManifold manifold;
        manifold.object_a_id = index_a;
        manifold.object_b_id = index_b;

        Vec3f delta = collider_b.position - collider_a.position;
        float dist = delta.length();
        float min_dist = collider_a.radius + collider_b.radius;

        // Check for zero distance (touching at same point)
        using phynity::math::utilities::is_zero;
        if (is_zero(dist, COLLISION_EPSILON) || dist > min_dist) {
            // No collision - return invalid manifold
            manifold.object_a_id = static_cast<size_t>(-1);
            manifold.object_b_id = static_cast<size_t>(-1);
            return manifold;
        }

        // Compute contact normal (from A to B)
        Vec3f normal = delta / dist;
        
        // Compute relative velocity
        Vec3f relative_velocity = collider_b.velocity - collider_a.velocity;
        float rel_normal = relative_velocity.dot(normal);

        // Skip if objects are separating
        if (rel_normal >= 0.0f) {
            manifold.object_a_id = static_cast<size_t>(-1);
            manifold.object_b_id = static_cast<size_t>(-1);
            return manifold;
        }

        // Populate contact information
        manifold.contact.position = collider_a.position + normal * collider_a.radius;
        manifold.contact.normal = normal;
        manifold.contact.penetration = min_dist - dist;
        manifold.contact.relative_velocity_along_normal = rel_normal;

        return manifold;
    }

    /// Detect collision with continuous collision detection (CCD) support.
    /// Performs both discrete and swept sphere detection depending on object velocities.
    /// 
    /// @param collider_a First colliding object with position and velocity
    /// @param collider_b Second colliding object with position and velocity
    /// @param index_a Index or ID of first object
    /// @param index_b Index or ID of second object
    /// @param dt Timestep duration (seconds)
    /// @param ccd_config CCD configuration options
    /// @return Contact manifold with collision time if CCD detected collision early
    static ContactManifold detect_with_ccd(
        const SphereCollider& collider_a,
        const SphereCollider& collider_b,
        size_t index_a,
        size_t index_b,
        float dt,
        const CCDConfig& ccd_config = CCDConfig()
    ) {
        // First, check discrete collision at current frame (fast path)
        ContactManifold discrete_result = detect(collider_a, collider_b, index_a, index_b);
        if (discrete_result.is_valid()) {
            // Already colliding - discrete detection found it
            return discrete_result;
        }

        // CCD not enabled, return no collision
        if (!ccd_config.enabled || dt < 1e-10f) {
            return discrete_result;
        }

        // Decide whether to use CCD based on speed heuristic
        if (!should_use_ccd(collider_a, collider_b, dt, ccd_config)) {
            return discrete_result;
        }

        // Try swept sphere CCD detection
        return detect_swept(collider_a, collider_b, index_a, index_b, dt);
    }

    /// Pure continuous collision detection using swept sphere algorithm.
    /// Ignores discrete collision status - only checks swept volumes.
    /// Useful for fast-moving objects that might miss discrete detections.
    /// 
    /// @param collider_a First sphere with position and velocity
    /// @param collider_b Second sphere with position and velocity
    /// @param index_a Index or ID of first object
    /// @param index_b Index or ID of second object
    /// @param dt Timestep duration (seconds)
    /// @return Contact manifold with collision time if swept collision detected
    static ContactManifold detect_swept(
        const SphereCollider& collider_a,
        const SphereCollider& collider_b,
        size_t index_a,
        size_t index_b,
        float dt
    ) {
        ContactManifold manifold;
        manifold.object_a_id = index_a;
        manifold.object_b_id = index_b;

        // Set up swept sphere solver
        SweptSphereSolver::Sphere sphere_a{
            collider_a.position,
            collider_a.velocity,
            collider_a.radius
        };

        SweptSphereSolver::Sphere sphere_b{
            collider_b.position,
            collider_b.velocity,
            collider_b.radius
        };

        // Solve for time of impact
        TimeOfImpactResult toi_result = SweptSphereSolver::solve_static(sphere_a, sphere_b, dt);

        // Fallback for difficult numerical scenarios: iterative conservative advancement
        if (!toi_result.collision_occurs) {
            toi_result = ConservativeAdvancement::solve_sphere_sphere(sphere_a, sphere_b, dt);
        }

        if (!toi_result.collision_occurs) {
            // No collision during timestep
            manifold.object_a_id = static_cast<size_t>(-1);
            manifold.object_b_id = static_cast<size_t>(-1);
            return manifold;
        }

        // Found collision via CCD
        // Store contact information at collision time
        manifold.contact.position = toi_result.contact_point;
        manifold.contact.normal = toi_result.contact_normal;
        manifold.contact.penetration = 0.0f;  // CCD gives exact contact, no penetration
        manifold.contact.relative_velocity_along_normal = 
            toi_result.relative_velocity.dot(toi_result.contact_normal);
        
        // Store collision time for sub-stepping (as normalized [0, 1])
        manifold.toi = toi_result.toi;

        return manifold;
    }

private:
    /// Determine if CCD should be used based on object velocities and thresholds.
    /// Checks both velocity_threshold and distance_threshold heuristics.
    /// 
    /// @param collider_a First object
    /// @param collider_b Second object
    /// @param dt Timestep duration
    /// @param ccd_config CCD configuration
    /// @return True if CCD should be tested, false if discrete detection sufficient
    static bool should_use_ccd(
        const SphereCollider& collider_a,
        const SphereCollider& collider_b,
        float dt,
        const CCDConfig& ccd_config
    ) {
        // Optimization: compute velocity magnitudes once and cache
        float speed_a = collider_a.velocity.length();
        float speed_b = collider_b.velocity.length();
        float max_speed = std::max(speed_a, speed_b);

        // Early exit: velocity threshold check (most common fast path)
        if (max_speed >= ccd_config.velocity_threshold) {
            return true;  // Fast enough to warrant CCD
        }

        // Distance threshold check: optimize by avoiding division
        // Original: (speed_a + speed_b) * dt / average_radius > distance_threshold
        // Rearranged: (speed_a + speed_b) * dt > distance_threshold * average_radius
        float combined_distance = (speed_a + speed_b) * dt;
        float average_radius = (collider_a.radius + collider_b.radius) * 0.5f;
        float distance_threshold_scaled = ccd_config.distance_threshold * average_radius;

        if (combined_distance > distance_threshold_scaled) {
            return true;  // Moving far enough relative to size
        }

        return false;  // Below both thresholds - discrete detection sufficient
    }
};

}  // namespace phynity::physics::collision

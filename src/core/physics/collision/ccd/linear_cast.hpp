#pragma once

#include <core/math/vectors/vec3.hpp>

#include <cmath>
#include <optional>

namespace phynity::physics::collision::ccd
{

using phynity::math::vectors::Vec3f;

/// Result of a ray-sphere intersection test.
struct RaySpherIntersection
{
    /// Whether the ray intersects the sphere
    bool hit = false;

    /// Near intersection time [0, 1] (enters sphere)
    float t_near = 0.0f;

    /// Far intersection time [0, 1] (exits sphere)
    float t_far = 0.0f;

    /// Contact point at near intersection
    Vec3f contact_point = Vec3f(0.0f);

    /// Surface normal at near intersection (pointing outward from sphere)
    Vec3f contact_normal = Vec3f(0.0f, 1.0f, 0.0f);
};

/// Utility functions for linear (translational) casting operations.
/// These are the building blocks for more complex CCD algorithms.
class LinearCast
{
public:
    /// Cast a ray against a stationary sphere.
    /// Solves: |ray_origin + t * ray_direction - sphere_center|² = sphere_radius²
    ///
    /// @param ray_origin Starting point of the ray
    /// @param ray_direction Normalized direction of the ray
    /// @param ray_max_t Maximum parameter value [0, 1] for timestep-bounded ray
    /// @param sphere_center Center of the sphere
    /// @param sphere_radius Radius of the sphere (must be > 0)
    /// @return RaySpherIntersection with hit info or empty if no intersection
    static RaySpherIntersection raycast_sphere(const Vec3f &ray_origin,
                                               const Vec3f &ray_direction,
                                               float ray_max_t,
                                               const Vec3f &sphere_center,
                                               float sphere_radius);

    /// Cast a moving sphere against a stationary sphere (conservative approach).
    /// Returns first time they touch (distance = r1 + r2).
    ///
    /// Algorithm:
    /// 1. Compute relative velocity: v_rel = v_moving - v_static = v_moving (since static)
    /// 2. Compute closest approach time using quadratic formula
    /// 3. Return first contact time within [0, 1]
    ///
    /// @param moving_sphere_center Starting position of moving sphere
    /// @param moving_sphere_velocity Velocity of moving sphere during timestep
    /// @param moving_sphere_radius Radius of moving sphere
    /// @param static_sphere_center Center of static sphere (stationary)
    /// @param static_sphere_radius Radius of static sphere
    /// @param max_time Maximum time to check [0, 1] (1 = full timestep)
    /// @return TimeOfImpactResult with contact info, or no collision if toi > max_time
    static RaySpherIntersection sweep_sphere_vs_sphere_static(const Vec3f &moving_sphere_center,
                                                              const Vec3f &moving_sphere_velocity,
                                                              float moving_sphere_radius,
                                                              const Vec3f &static_sphere_center,
                                                              float static_sphere_radius,
                                                              float max_time = 1.0f);

    /// Cast a moving sphere against a stationary AABB.
    /// Used for swept box-sphere collision detection.
    ///
    /// @param sphere_center Starting position of sphere
    /// @param sphere_velocity Velocity of sphere
    /// @param sphere_radius Radius of sphere
    /// @param aabb_min Minimum corner of AABB
    /// @param aabb_max Maximum corner of AABB
    /// @param max_time Maximum time to check [0, 1]
    /// @return RaySpherIntersection with contact time
    static RaySpherIntersection sweep_sphere_vs_aabb(const Vec3f &sphere_center,
                                                     const Vec3f &sphere_velocity,
                                                     float sphere_radius,
                                                     const Vec3f &aabb_min,
                                                     const Vec3f &aabb_max,
                                                     float max_time = 1.0f);

    /// Compute time of closest approach between two moving spheres.
    /// Used internally for swept sphere-sphere collision.
    ///
    /// Solves: d/dt |p1(t) - p2(t)| = 0
    /// where p1(t) = pos1 + vel1 * t, p2(t) = pos2 + vel2 * t
    ///
    /// @param pos1 Initial position of sphere 1
    /// @param vel1 Velocity of sphere 1
    /// @param pos2 Initial position of sphere 2
    /// @param vel2 Velocity of sphere 2
    /// @return Time of closest approach in [0, 1], or 1.0 if no approach
    static float
    compute_closest_approach_time(const Vec3f &pos1, const Vec3f &vel1, const Vec3f &pos2, const Vec3f &vel2);

    /// Check if two moving spheres are approaching (getting closer).
    ///
    /// @param pos1 Position of sphere 1
    /// @param vel1 Velocity of sphere 1
    /// @param pos2 Position of sphere 2
    /// @param vel2 Velocity of sphere 2
    /// @return True if relative velocity points toward contact
    static bool are_spheres_approaching(const Vec3f &pos1, const Vec3f &vel1, const Vec3f &pos2, const Vec3f &vel2);

    /// Clamp time value to [0, max_time] range.
    /// @param time Time value to clamp
    /// @param max_time Maximum allowed time
    /// @return Clamped time value
    static float clamp_time(float time, float max_time = 1.0f)
    {
        return (time < 0.0f) ? 0.0f : (time > max_time) ? max_time : time;
    }
};

} // namespace phynity::physics::collision::ccd

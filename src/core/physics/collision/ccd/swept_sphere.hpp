#pragma once

#include <core/physics/collision/ccd/toi_solver.hpp>
#include <core/physics/collision/ccd/linear_cast.hpp>
#include <core/math/vectors/vec3.hpp>

namespace phynity::physics::collision::ccd {

using phynity::math::vectors::Vec3f;

/// Swept sphere-sphere collision detection using time-of-impact calculation.
/// 
/// Handles collision between two moving spheres with arbitrary velocities.
/// Uses continuous collision detection to find exact contact time within timestep.
/// 
/// Algorithm:
/// 1. Compute relative position and velocity
/// 2. Solve quadratic equation for time = distance / relative_speed
/// 3. Find the smallest non-negative root <= 1.0 (within timestep)
/// 4. Compute contact point and normal at collision time
/// 5. Return detailed TimeOfImpactResult with all contact information
/// 
/// Time complexity: O(1) - analytical solution, no iteration needed
/// 
/// Mathematical formulation:
/// Let p1(t) = pos1 + vel1 * t, p2(t) = pos2 + vel2 * t
/// Let rel_pos(t) = p1(t) - p2(t) = (pos1 - pos2) + (vel1 - vel2) * t
/// Collision when: |rel_pos(t)| = r1 + r2
/// 
/// Expanding: |p(0) + v*t|² = (r1+r2)²
/// (p·p) + 2*t*(p·v) + t²*(v·v) = (r1+r2)²
/// t²*(v·v) + t*2*(p·v) + (p·p - (r1+r2)²) = 0
/// 
/// Standard quadratic: a*t² + b*t + c = 0
/// where: a = v·v, b = 2*(p·v), c = p·p - (r1+r2)²
class SweptSphereSolver : public TOISolver {
public:
    /// Sphere A (first sphere)
    struct Sphere {
        Vec3f position;      ///< Position at start of timestep
        Vec3f velocity;      ///< Velocity during timestep
        float radius = 0.5f; ///< Collision radius
    };

    /// Construct swept sphere collision detector.
    /// @param sphere_a First sphere with position, velocity, radius
    /// @param sphere_b Second sphere with position, velocity, radius
    /// @param dt Timestep duration (used to scale time result to [0,1])
    SweptSphereSolver(
        const Sphere& sphere_a,
        const Sphere& sphere_b,
        float dt = 1.0f
    )
        : sphere_a_(sphere_a),
          sphere_b_(sphere_b),
          dt_(dt > 0.0f ? dt : 1.0f)
    {
    }

    /// Solve for time of impact between the two spheres.
    /// @return TimeOfImpactResult with collision_occurs flag and contact info
    TimeOfImpactResult solve() const override;

    /// Alternative solve without storing in member variables.
    /// Useful for one-off computations without constructing an object.
    /// 
    /// @param sphere_a First sphere
    /// @param sphere_b Second sphere
    /// @param dt Timestep duration
    /// @return TimeOfImpactResult with collision timing and contact information
    static TimeOfImpactResult solve_static(
        const Sphere& sphere_a,
        const Sphere& sphere_b,
        float dt = 1.0f
    );

private:
    Sphere sphere_a_;
    Sphere sphere_b_;
    float dt_;

    /// Solve quadratic equation a*t² + b*t + c = 0.
    /// Returns smallest non-negative root, or -1 if no valid root.
    /// 
    /// @param a Quadratic coefficient
    /// @param b Linear coefficient
    /// @param c Constant coefficient
    /// @param max_t Maximum time value to accept
    /// @return Smallest valid root in [0, max_t], or -1 if none
    static float solve_quadratic(float a, float b, float c, float max_t = 1.0f);

    /// Compute contact point at given time.
    /// @param time Time value [0, 1]
    /// @param sphere_a First sphere at time
    /// @param sphere_b Second sphere at time
    /// @return World-space contact point (midpoint of surface contact)
    static Vec3f compute_contact_point(
        float time,
        const Sphere& sphere_a,
        const Sphere& sphere_b
    );

    /// Compute contact normal at given time.
    /// @param time Time value [0, 1]
    /// @param sphere_a First sphere at time
    /// @param sphere_b Second sphere at time
    /// @return Normalized normal pointing from A to B
    static Vec3f compute_contact_normal(
        float time,
        const Sphere& sphere_a,
        const Sphere& sphere_b
    );

    /// Compute relative velocity at given time.
    /// @param time Time value [0, 1] (constant for linear motion)
    /// @param sphere_a First sphere
    /// @param sphere_b Second sphere
    /// @return B's velocity relative to A at contact
    static Vec3f compute_relative_velocity(
        const Sphere& sphere_a,
        const Sphere& sphere_b
    );
};

}  // namespace phynity::physics::collision::ccd

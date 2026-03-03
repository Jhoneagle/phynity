#pragma once

#include <core/math/vectors/vec3.hpp>
#include <cmath>
#include <limits>

namespace phynity::physics::collision::ccd {

using phynity::math::vectors::Vec3f;

/// Result of a time-of-impact (TOI) calculation between two objects.
/// Contains timing information and contact details if a collision occurs.
struct TimeOfImpactResult {
    /// Whether a collision occurs within the time interval [0, dt]
    bool collision_occurs = false;
    
    /// Time of collision in [0, 1] where 1 represents the full timestep dt
    /// If collision_occurs is true, the actual time is toi * dt
    /// Range: [0, 1] (1 means collision at end of timestep)
    float toi = std::numeric_limits<float>::max();
    
    /// World-space contact point at collision time
    Vec3f contact_point = Vec3f(0.0f);
    
    /// World-space contact normal (pointing from object A to object B)
    Vec3f contact_normal = Vec3f(0.0f, 1.0f, 0.0f);
    
    /// Relative velocity of B with respect to A at collision time
    Vec3f relative_velocity = Vec3f(0.0f);
    
    /// Constructor for non-collision case
    TimeOfImpactResult() = default;
    
    /// Constructor for collision case
    TimeOfImpactResult(
        float time_of_impact,
        const Vec3f& contact_pt,
        const Vec3f& contact_norm,
        const Vec3f& rel_vel
    )
        : collision_occurs(true),
          toi(time_of_impact),
          contact_point(contact_pt),
          contact_normal(contact_norm),
          relative_velocity(rel_vel)
    {
    }
};

/// Generic interface for time-of-impact solvers.
/// Implementations compute when two moving objects first collide.
class TOISolver {
public:
    virtual ~TOISolver() = default;

    /// Compute time of impact between two moving objects within timestep dt.
    /// @return TimeOfImpactResult with collision timing and contact information
    virtual TimeOfImpactResult solve() const = 0;
};

}  // namespace phynity::physics::collision::ccd

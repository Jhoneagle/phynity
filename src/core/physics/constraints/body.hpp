#pragma once

#include <core/math/matrices/mat3.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/vectors/vec3.hpp>

namespace phynity::physics::constraints
{

using phynity::math::matrices::Mat3f;
using phynity::math::quaternions::Quatf;
using phynity::math::vectors::Vec3f;

/// Abstract interface for anything that can be constrained.
/// Both Particle and RigidBody implement this, allowing constraints
/// to work generically across body types.
class Body
{
public:
    virtual ~Body() = default;

    // Linear state (all bodies)
    virtual Vec3f get_position() const = 0;
    virtual Vec3f get_velocity() const = 0;
    virtual float get_inverse_mass() const = 0;
    virtual void apply_velocity_impulse(const Vec3f &impulse) = 0;

    // Angular state (no-op defaults for point masses)
    virtual Quatf get_orientation() const
    {
        return Quatf();
    }
    virtual Vec3f get_angular_velocity() const
    {
        return Vec3f(0.0f);
    }
    virtual Mat3f get_inverse_inertia_world() const
    {
        return Mat3f(0.0f);
    }
    virtual void apply_angular_impulse(const Vec3f &impulse)
    {
        (void) impulse;
    }

    // Queries
    virtual bool is_static() const = 0;
    virtual bool is_alive() const
    {
        return true;
    }

    // Material access for restitution
    virtual float get_restitution() const
    {
        return 0.0f;
    }
};

} // namespace phynity::physics::constraints

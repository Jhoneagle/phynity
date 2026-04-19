#pragma once

#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_conversions.hpp>
#include <core/physics/constraints/pivot_joint.hpp>

#include <cmath>

namespace phynity::physics::constraints
{

using phynity::math::quaternions::Quatf;

/// Weld joint: locks both position and orientation between two bodies (6-DOF lock).
/// Inherits positional pivot constraint from PivotJoint and adds rotation error.
class WeldJoint : public PivotJoint
{
public:
    WeldJoint(Body *body_a,
              Body *body_b,
              const Vec3f &anchor_a_local,
              const Vec3f &anchor_b_local)
        : PivotJoint(body_a, body_b, anchor_a_local, anchor_b_local)
    {
        if (body_a && body_b)
        {
            initial_relative_q_ = body_b->get_orientation().conjugate() * body_a->get_orientation();
        }
    }

protected:
    float compute_pivot_error() const override
    {
        float pos_error = cached_error_len_;

        Quatf orient_a = body_a_->get_orientation();
        Quatf relative_q = body_b_
            ? body_b_->get_orientation() * orient_a.conjugate()
            : orient_a;
        Quatf error_q = relative_q * initial_relative_q_.conjugate();

        float rotation_error = 2.0f * std::acos(std::clamp(error_q.w, -1.0f, 1.0f));

        return std::max(pos_error, rotation_error);
    }

private:
    Quatf initial_relative_q_;
};

} // namespace phynity::physics::constraints

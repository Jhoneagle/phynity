#pragma once

#include <core/physics/constraints/pivot_joint.hpp>

namespace phynity::physics::constraints
{

/// Hinge joint: locks the pivot point while allowing rotation around one axis.
/// Inherits positional pivot constraint from PivotJoint.
class HingeJoint : public PivotJoint
{
public:
    HingeJoint(
        Body *body_a, Body *body_b, const Vec3f &pivot_a_local, const Vec3f &pivot_b_local, const Vec3f &axis_local)
        : PivotJoint(body_a, body_b, pivot_a_local, pivot_b_local), axis_local_(axis_local.normalized())
    {
    }

private:
    Vec3f axis_local_;
};

} // namespace phynity::physics::constraints

#pragma once

#include <core/physics/collision/ccd/swept_sphere.hpp>

namespace phynity::physics::collision::ccd
{

/// Conservative advancement fallback for sphere-sphere CCD.
///
/// Uses iterative advancement by separation/closing-speed estimate:
/// 1) Measure current separation between expanded spheres.
/// 2) Advance by sep / closing_speed.
/// 3) Repeat until contact or no approach.
class ConservativeAdvancement
{
public:
    static TimeOfImpactResult solve_sphere_sphere(const SweptSphereSolver::Sphere &sphere_a,
                                                  const SweptSphereSolver::Sphere &sphere_b,
                                                  float max_time = 1.0f,
                                                  int max_iterations = 16,
                                                  float separation_tolerance = 1e-4f,
                                                  int root_refinement_iterations = 8);
};

} // namespace phynity::physics::collision::ccd

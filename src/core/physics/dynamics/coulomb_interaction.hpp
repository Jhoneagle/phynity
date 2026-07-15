#pragma once

#include <core/math/utilities/float_comparison.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/particles/particle.hpp>

#include <algorithm>
#include <vector>

namespace phynity::physics
{

using phynity::math::vectors::Vec3f;

/// Mutual (particle-particle) electrostatic interaction.
///
/// Unlike the field-based electromagnetic ForceFields, mutual Coulomb is
/// neighbor-coupled: each charged particle's force depends on every other
/// charged particle, which cannot be expressed by the per-body ForceField
/// contract. It is therefore an optional force-accumulation pass over the
/// existing particles, invoked like the collision resolver — not a ForceField.
///
/// Direct O(N²) summation over ordered pairs (i < j), applying exactly
/// equal-and-opposite forces (Newton's third law) so total linear momentum is
/// conserved. The fixed ascending pair order keeps the summation deterministic.
/// Coulomb is long-range (1/r², no natural cutoff), so no spatial acceleration
/// is used here; a grid cutoff would be a physical approximation and is deferred.
///
/// @param particles Bodies to interact; forces are accumulated in place.
/// @param coulomb_constant Coupling constant k (simulation units).
/// @param min_distance Softening clamp (> 0) bounding the 1/r² singularity.
inline void accumulate_coulomb_forces(std::vector<Particle> &particles,
                                      float coulomb_constant = constants::COULOMB_CONSTANT,
                                      float min_distance = 1e-3f)
{
    using phynity::math::utilities::is_zero;

    const float min_distance_sq = min_distance * min_distance;
    const size_t count = particles.size();

    for (size_t i = 0; i < count; ++i)
    {
        Particle &pi = particles[i];
        if (!pi.is_alive() || is_zero(pi.material.charge))
        {
            continue;
        }

        for (size_t j = i + 1; j < count; ++j)
        {
            Particle &pj = particles[j];
            if (!pj.is_alive() || is_zero(pj.material.charge))
            {
                continue;
            }

            Vec3f d = pi.position - pj.position; // points from j toward i
            float r2 = std::max(d.squaredLength(), min_distance_sq);
            if (is_zero(r2))
            {
                continue;
            }

            // F on i = k q_i q_j r̂ / r² (repulsive for like signs).
            Vec3f force = d.normalized() * (coulomb_constant * pi.material.charge * pj.material.charge / r2);
            pi.apply_force(force);
            pj.apply_force(-force);
        }
    }
}

} // namespace phynity::physics

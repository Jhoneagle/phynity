#pragma once

#include <core/math/vectors/vec3.hpp>

namespace phynity::physics::fluids
{

using phynity::math::vectors::Vec3f;

/// Lean per-particle state for the SPH/PBF fluid prototype.
///
/// Deliberately *not* the heavier `Particle` type: SPH needs per-particle
/// `density`/`pressure` scratch and none of `Particle`'s material, lifetime,
/// collision-radius, or Body-interface baggage. Keeping it a plain struct with
/// default member initializers (for clang-tidy cleanliness) keeps the prototype
/// isolated and cache-friendly.
///
/// PBF-specific scratch (predicted position, lambda, delta position) lives in
/// `PbfFluidSystem` as parallel arrays, so this struct stays shared and lean
/// across both solvers.
struct FluidParticle
{
    Vec3f position{0.0f}; ///< Current position in world space
    Vec3f velocity{0.0f}; ///< Current velocity
    Vec3f force{0.0f};    ///< Accumulated force this step (WCSPH)
    float mass{0.0f};     ///< Particle mass (kg); tie to rest density via mass_for_spacing()
    float density{0.0f};  ///< SPH density estimate ρ_i (scratch, recomputed each step)
    float pressure{0.0f}; ///< EOS pressure p_i (scratch, recomputed each step)
};

} // namespace phynity::physics::fluids

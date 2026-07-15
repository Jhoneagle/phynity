#pragma once

#include <core/physics/config/physics_constants.hpp>
#include <core/physics/shapes/aabb.hpp>

namespace phynity::physics::fluids
{

using phynity::physics::constants::WATER_DENSITY;
using phynity::physics::shapes::AABB;

/// Tunable parameters for the weakly-compressible SPH solver.
///
/// Defaults target a small water-like prototype pool. Stiffness/viscosity are
/// intentionally conservative: WCSPH is subject to a CFL-like stability limit
/// (higher stiffness needs smaller dt), and the validation suite asserts
/// *bounded/finite* behavior rather than exact trajectories.
struct SphParameters
{
    float smoothing_radius{0.1f}; ///< Kernel support radius h (world units)
    float rest_density{WATER_DENSITY}; ///< Target rest density ρ₀ (kg/m³)
    float stiffness{2000.0f}; ///< EOS stiffness k in p = k·(ρ − ρ₀)
    float viscosity{0.1f}; ///< Dynamic viscosity μ (Müller viscosity term)
    float surface_tension{0.0f}; ///< Surface-tension coefficient σ (off by default)
    float surface_tension_threshold{1.0f}; ///< Min ‖∇color‖ before tension applies (noise gate)
    float particle_mass{0.125f}; ///< Per-particle mass; keep consistent via mass_for_spacing()
    float boundary_restitution{0.3f}; ///< Normal-velocity damping on wall reflection [0,1]
    bool clamp_negative_pressure{false}; ///< If true, clamp p to ≥ 0 (suppresses tensile instability)
    AABB bounds{Vec3f(-1.0f), Vec3f(1.0f)}; ///< Container the fluid is confined to
};

/// Mass a particle must have to sit at `rest_density` when seeded on a regular
/// lattice of the given spacing: m = ρ₀ · s³.
///
/// `particle_mass` and `rest_density` are NOT independent knobs — if they
/// disagree, a lattice-initialized fluid is not actually at rest and will
/// instantly compress or explode. Every spawn site (tests + demo) should size
/// mass through this helper so no scenario silently seeds an inconsistent state.
///
/// @param rest_density Target rest density ρ₀ (kg/m³)
/// @param spacing      Lattice spacing s (world units)
/// @return Consistent per-particle mass
constexpr float mass_for_spacing(float rest_density, float spacing) noexcept
{
    return rest_density * spacing * spacing * spacing;
}

} // namespace phynity::physics::fluids

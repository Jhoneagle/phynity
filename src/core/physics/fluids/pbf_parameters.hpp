#pragma once

#include <core/physics/fluids/sph_parameters.hpp>

namespace phynity::physics::fluids
{

/// Parameters for the position-based fluids (PBF, Macklin & Müller 2013) solver.
///
/// Builds on the shared `SphParameters` (smoothing radius, rest density,
/// particle mass, bounds, boundary restitution) and adds the constraint-solver
/// knobs. Stabilization parameters (tensile correction, XSPH, vorticity) are
/// added in a later phase and default to off/negligible.
struct PbfParameters
{
    SphParameters sph{};        ///< Shared kernel/rest-density/mass/bounds settings
    int solver_iterations{4};   ///< Density-constraint projection iterations per step
    float relaxation{1.0e-4f};  ///< CFM relaxation ε in λ = −C / (Σ‖∇C‖² + ε)
};

} // namespace phynity::physics::fluids

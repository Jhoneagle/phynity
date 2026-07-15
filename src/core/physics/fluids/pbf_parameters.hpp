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
    SphParameters sph{}; ///< Shared kernel/rest-density/mass/bounds settings
    int solver_iterations{4}; ///< Density-constraint projection iterations per step
    float relaxation{1.0e-4f}; ///< CFM relaxation ε in λ = −C / (Σ‖∇C‖² + ε)

    /// Compression-only constraint: zero λ for under-dense (C < 0) particles so
    /// the solver never pulls free-surface particles inward. Without this, a
    /// free surface collapses (surface particles can never reach ρ₀, so the
    /// constraint keeps contracting the fluid). The PBF analogue of WCSPH's
    /// `clamp_negative_pressure`.
    bool clamp_density_deficiency{true};

    // --- Stabilization (all off/negligible by default) ---

    /// Artificial-pressure tensile-instability correction (Macklin §4):
    /// s_corr = −k·(W(r)/W(Δq))ⁿ added inside Δp. Prevents particle clumping and
    /// gives a surface-tension-like clustering. `scorr_k == 0` disables it.
    float scorr_k{0.0f}; ///< Strength k (0 = off)
    float scorr_dq{0.2f}; ///< Reference distance Δq as a fraction of h
    float scorr_n{4.0f}; ///< Exponent n

    /// XSPH velocity smoothing: v_i += c·Σ_j (v_j−v_i)·W/ρ_j. `xsph_c == 0` off.
    float xsph_c{0.0f};

    /// Vorticity confinement strength ε. `vorticity_epsilon == 0` off.
    float vorticity_epsilon{0.0f};
};

} // namespace phynity::physics::fluids

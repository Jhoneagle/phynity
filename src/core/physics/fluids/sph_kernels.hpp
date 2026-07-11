#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>

namespace phynity::physics::fluids
{

using phynity::math::vectors::Vec3f;

/// ============================================================================
/// SPH Smoothing Kernels (Müller et al. 2003)
/// ============================================================================
///
/// The classic textbook triad used by weakly-compressible SPH:
///   - poly6              : density accumulation (smooth, well-behaved at r=0)
///   - spiky gradient     : pressure force (steep near r=0 so it stays repulsive)
///   - viscosity laplacian: viscosity force (positive, monotone decreasing)
///
/// All kernels are 3D-normalized and have compact support: they are exactly
/// zero for r > h. Constants are the standard closed forms; keeping them here
/// (rather than a shared magic number) makes the normalization auditable.
///
/// Threshold at r=0 for the spiky gradient: it divides by r, so callers must
/// guard tiny separations. The gradient here returns the zero vector for
/// r <= SPH_KERNEL_EPSILON to keep sanitizers clean and forces finite.

/// Separation below which a pairwise kernel gradient is treated as zero.
/// Mirrors the is_zero()/normalized() guard discipline used elsewhere so the
/// r-division in the spiky gradient never produces inf/NaN.
constexpr float SPH_KERNEL_EPSILON = 1e-12f;

/// poly6 density kernel: W(r,h) = 315 / (64 π h⁹) · (h² − r²)³ for 0 ≤ r ≤ h.
///
/// Takes r² directly so the density loop can skip a sqrt.
/// @param r2 Squared distance between the two particles (‖r_ij‖²)
/// @param h  Smoothing radius
/// @return Kernel weight, or 0 when r > h
inline float poly6(float r2, float h) noexcept
{
    const float h2 = h * h;
    if (r2 < 0.0f || r2 > h2)
    {
        return 0.0f;
    }

    const float h9 = h2 * h2 * h2 * h2 * h; // h⁹
    const float coeff = 315.0f / (64.0f * constants::PI * h9);
    const float diff = h2 - r2;
    return coeff * diff * diff * diff;
}

/// Spiky pressure-kernel gradient: ∇W(r,h) = −45 / (π h⁶) · (h − r)² · (r_vec / r).
///
/// Points along −r_vec (from j toward i is repulsive under a positive pressure
/// coefficient). Zero for r > h and for r ≈ 0 (singularity guard).
/// @param r_vec Displacement vector r_i − r_j
/// @param r     Its magnitude ‖r_vec‖ (passed in to avoid recomputing a sqrt)
/// @param h     Smoothing radius
/// @return Gradient vector, or the zero vector when r > h or r ≈ 0
inline Vec3f spiky_gradient(const Vec3f &r_vec, float r, float h) noexcept
{
    if (r <= SPH_KERNEL_EPSILON || r > h)
    {
        return Vec3f(0.0f);
    }

    const float h3 = h * h * h;
    const float h6 = h3 * h3;
    const float coeff = -45.0f / (constants::PI * h6);
    const float diff = h - r;
    // coeff · (h − r)² · (r_vec / r)
    return r_vec * (coeff * diff * diff / r);
}

/// Viscosity-kernel laplacian: ∇²W(r,h) = 45 / (π h⁶) · (h − r) for 0 ≤ r ≤ h.
///
/// Non-negative and monotone decreasing to 0 at r = h.
/// @param r Distance between the two particles ‖r_ij‖
/// @param h Smoothing radius
/// @return Laplacian value, or 0 when r > h
inline float viscosity_laplacian(float r, float h) noexcept
{
    if (r < 0.0f || r > h)
    {
        return 0.0f;
    }

    const float h3 = h * h * h;
    const float h6 = h3 * h3;
    const float coeff = 45.0f / (constants::PI * h6);
    return coeff * (h - r);
}

} // namespace phynity::physics::fluids

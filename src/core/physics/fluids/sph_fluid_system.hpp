#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/fluids/fluid_neighbor_search.hpp>
#include <core/physics/fluids/fluid_particle.hpp>
#include <core/physics/fluids/sph_kernels.hpp>
#include <core/physics/fluids/sph_parameters.hpp>

#include <algorithm>
#include <vector>

namespace phynity::physics::fluids
{

using phynity::math::vectors::Vec3f;
using phynity::physics::constants::EARTH_GRAVITY_VECTOR;

/// Weakly-compressible SPH fluid solver (Müller 2003 kernels, Monaghan
/// symmetric pressure force, semi-implicit Euler integration).
///
/// A peer subsystem to `ParticleSystem` / `RigidBodySystem`: it owns its own
/// particles, neighbor search, parameters, and per-step pipeline. It is a
/// separate system rather than a `ForceField` because SPH's
/// density→pressure→force data dependency across neighbors cannot be expressed
/// by the pure per-particle `apply(ctx)` contract.
///
/// This class is grown phase by phase: Phase 2 lands storage + accessors +
/// neighbor rebuild; density/pressure/forces/integration/boundaries and the
/// full `update(dt)` pipeline follow in later phases.
class SphFluidSystem
{
public:
    SphFluidSystem() = default;

    explicit SphFluidSystem(const SphParameters &params) : params_(params)
    {
    }

    // ========================================================================
    // Particle Management
    // ========================================================================

    /// Spawn a fluid particle. Mass defaults to the parameter set's
    /// `particle_mass` (keep it consistent with rest density via
    /// `mass_for_spacing()`).
    /// @param position Starting position
    /// @param velocity Starting velocity (default: rest)
    /// @param mass     Particle mass (default: params.particle_mass)
    void spawn(const Vec3f &position, const Vec3f &velocity = Vec3f(0.0f), float mass = -1.0f)
    {
        FluidParticle p;
        p.position = position;
        p.velocity = velocity;
        p.mass = (mass > 0.0f) ? mass : params_.particle_mass;
        particles_.push_back(p);
    }

    /// Remove all fluid particles.
    void clear()
    {
        particles_.clear();
    }

    [[nodiscard]] std::vector<FluidParticle> &particles() noexcept
    {
        return particles_;
    }

    [[nodiscard]] const std::vector<FluidParticle> &particles() const noexcept
    {
        return particles_;
    }

    [[nodiscard]] size_t particle_count() const noexcept
    {
        return particles_.size();
    }

    // ========================================================================
    // Environment & Parameters
    // ========================================================================

    void set_ambient_gravity(const Vec3f &gravity) noexcept
    {
        ambient_gravity_ = gravity;
    }

    [[nodiscard]] Vec3f ambient_gravity() const noexcept
    {
        return ambient_gravity_;
    }

    [[nodiscard]] SphParameters &parameters() noexcept
    {
        return params_;
    }

    [[nodiscard]] const SphParameters &parameters() const noexcept
    {
        return params_;
    }

    void set_parameters(const SphParameters &params) noexcept
    {
        params_ = params;
    }

    [[nodiscard]] const FluidNeighborSearch &neighbor_search() const noexcept
    {
        return neighbor_search_;
    }

    // ========================================================================
    // Neighbor Search
    // ========================================================================

    /// Rebuild the neighbor lists from the current particle positions using the
    /// smoothing radius as the grid cell size. Called at the start of each step.
    void rebuild_neighbors()
    {
        position_cache_.resize(particles_.size());
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            position_cache_[i] = particles_[i].position;
        }
        neighbor_search_.rebuild(position_cache_, params_.smoothing_radius);
    }

    // ========================================================================
    // Solver Passes
    // ========================================================================

    /// Density pass: ρ_i = Σ_j m_j · poly6(‖r_ij‖², h), including the self term
    /// at r = 0. Assumes `rebuild_neighbors()` has run this step.
    void compute_density()
    {
        const float h = params_.smoothing_radius;
        const float self_w = poly6(0.0f, h);

        for (size_t i = 0; i < particles_.size(); ++i)
        {
            FluidParticle &pi = particles_[i];
            float density = pi.mass * self_w; // self contribution (r = 0)

            for (const uint32_t j : neighbor_search_.neighbors(i))
            {
                const FluidParticle &pj = particles_[j];
                const float r2 = (pi.position - pj.position).squaredLength();
                density += pj.mass * poly6(r2, h);
            }

            pi.density = density;
        }
    }

    /// Pressure pass: linear equation of state p_i = k·(ρ_i − ρ₀).
    /// When `clamp_negative_pressure` is set, negative (tensile) pressure is
    /// clamped to 0 to suppress the tensile instability that pulls particles
    /// into clumps.
    void compute_pressure()
    {
        const float k = params_.stiffness;
        const float rho0 = params_.rest_density;

        for (FluidParticle &p : particles_)
        {
            float pressure = k * (p.density - rho0);
            if (params_.clamp_negative_pressure)
            {
                pressure = std::max(0.0f, pressure);
            }
            p.pressure = pressure;
        }
    }

protected:
    std::vector<FluidParticle> particles_;
    FluidNeighborSearch neighbor_search_;
    SphParameters params_{};
    Vec3f ambient_gravity_{EARTH_GRAVITY_VECTOR};
    std::vector<Vec3f> position_cache_; ///< Scratch for neighbor rebuild
};

} // namespace phynity::physics::fluids

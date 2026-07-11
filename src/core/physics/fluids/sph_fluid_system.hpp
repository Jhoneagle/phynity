#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/fluids/fluid_neighbor_search.hpp>
#include <core/physics/fluids/fluid_particle.hpp>
#include <core/physics/fluids/sph_parameters.hpp>

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

protected:
    std::vector<FluidParticle> particles_;
    FluidNeighborSearch neighbor_search_;
    SphParameters params_{};
    Vec3f ambient_gravity_{EARTH_GRAVITY_VECTOR};
    std::vector<Vec3f> position_cache_; ///< Scratch for neighbor rebuild
};

} // namespace phynity::physics::fluids

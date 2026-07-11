#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/fluids/fluid_neighbor_search.hpp>
#include <core/physics/fluids/fluid_particle.hpp>
#include <core/physics/fluids/pbf_parameters.hpp>
#include <core/physics/fluids/sph_kernels.hpp>

#include <cmath>
#include <vector>

namespace phynity::physics::fluids
{

using phynity::math::vectors::Vec3f;
using phynity::physics::constants::EARTH_GRAVITY_VECTOR;

/// Position-based fluids solver (Macklin & Müller 2013).
///
/// A sibling to `SphFluidSystem` built on the *same* shared scaffolding — the
/// Müller kernels, `FluidNeighborSearch`, `FluidParticle`, `mass_for_spacing`,
/// and an equivalent box boundary. Instead of an equation-of-state pressure
/// force it enforces incompressibility with an iterative density-constraint
/// projection, which lets it stay stable at much larger timesteps than WCSPH.
///
/// Per-step pipeline:
///   1. apply body force + gravity to velocity, predict x* = x + dt·v
///   2. rebuild neighbors on the predicted positions
///   3. for solver_iterations: compute ρ, λ, Δp; apply x* += Δp; clamp to bounds
///   4. v = (x* − x)/dt;  x = x*
///
/// PBF-specific scratch (predicted position, λ, Δp) lives here as parallel
/// arrays so `FluidParticle` stays lean and shared with WCSPH.
class PbfFluidSystem
{
public:
    PbfFluidSystem() = default;

    explicit PbfFluidSystem(const PbfParameters &params) : params_(params)
    {
    }

    // ========================================================================
    // Particle Management
    // ========================================================================

    /// Spawn a fluid particle. Mass defaults to the shared `particle_mass`
    /// (keep it consistent with rest density via `mass_for_spacing()`).
    void spawn(const Vec3f &position, const Vec3f &velocity = Vec3f(0.0f), float mass = -1.0f)
    {
        FluidParticle p;
        p.position = position;
        p.velocity = velocity;
        p.mass = (mass > 0.0f) ? mass : params_.sph.particle_mass;
        particles_.push_back(p);
    }

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

    [[nodiscard]] PbfParameters &parameters() noexcept
    {
        return params_;
    }
    [[nodiscard]] const PbfParameters &parameters() const noexcept
    {
        return params_;
    }
    void set_parameters(const PbfParameters &params) noexcept
    {
        params_ = params;
    }

    [[nodiscard]] const FluidNeighborSearch &neighbor_search() const noexcept
    {
        return neighbor_search_;
    }

    // Scratch accessors (mainly for tests / diagnostics).
    [[nodiscard]] const std::vector<Vec3f> &predicted() const noexcept
    {
        return predicted_;
    }
    [[nodiscard]] const std::vector<float> &lambdas() const noexcept
    {
        return lambda_;
    }
    [[nodiscard]] const std::vector<Vec3f> &delta_positions() const noexcept
    {
        return delta_position_;
    }

    // ========================================================================
    // Solver Stages (public so the constraint loop is independently testable)
    // ========================================================================

    /// Apply body force + gravity to velocity and predict x* = x + dt·v.
    void predict(float dt)
    {
        predicted_.resize(particles_.size());
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            FluidParticle &p = particles_[i];
            p.velocity += ambient_gravity_ * dt; // body force per unit mass
            predicted_[i] = p.position + p.velocity * dt;
        }
    }

    /// Rebuild neighbor lists from the predicted positions.
    void rebuild_neighbors_predicted()
    {
        neighbor_search_.rebuild(predicted_, params_.sph.smoothing_radius);
    }

    /// One constraint-projection iteration: density → λ → Δp → apply → clamp.
    void iterate()
    {
        compute_densities();
        compute_lambdas();
        compute_and_apply_deltas();
    }

    /// v = (x* − x)/dt;  x = x*.
    void finalize(float dt)
    {
        const float inv_dt = (dt > 0.0f) ? 1.0f / dt : 0.0f;
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            FluidParticle &p = particles_[i];
            p.velocity = (predicted_[i] - p.position) * inv_dt;
            p.position = predicted_[i];
        }
    }

    // ========================================================================
    // Full Step
    // ========================================================================

    void update(float dt)
    {
        predict(dt);
        rebuild_neighbors_predicted();
        for (int iter = 0; iter < params_.solver_iterations; ++iter)
        {
            iterate();
        }
        finalize(dt);
        apply_vorticity_confinement(dt); // no-op when ε = 0
        apply_xsph();                    // no-op when c = 0
    }

    /// XSPH velocity smoothing: v_i += c·Σ_j (v_j−v_i)·W_poly6/ρ_j.
    /// Reduces velocity noise without adding net momentum (the pairwise term is
    /// antisymmetric at uniform density). No-op when `xsph_c == 0`. Uses the
    /// predicted-position neighbor lists and the densities from the last iterate.
    void apply_xsph()
    {
        const float c = params_.xsph_c;
        if (c == 0.0f || particles_.empty())
        {
            return;
        }

        const float h = params_.sph.smoothing_radius;
        xsph_delta_.assign(particles_.size(), Vec3f(0.0f));
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            Vec3f dv(0.0f);
            for (const uint32_t j : neighbor_search_.neighbors(i))
            {
                const float rho_j = particles_[j].density;
                if (rho_j <= kDensityEpsilon)
                {
                    continue;
                }
                const float r2 = (predicted_[i] - predicted_[j]).squaredLength();
                dv += (particles_[j].velocity - particles_[i].velocity) * (poly6(r2, h) / rho_j);
            }
            xsph_delta_[i] = dv * c;
        }
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            particles_[i].velocity += xsph_delta_[i];
        }
    }

    /// Vorticity confinement: reintroduces the rotational detail that the
    /// constraint solve damps out. No-op when `vorticity_epsilon == 0`.
    void apply_vorticity_confinement(float dt)
    {
        const float eps = params_.vorticity_epsilon;
        if (eps == 0.0f || particles_.empty())
        {
            return;
        }

        const float h = params_.sph.smoothing_radius;

        // ω_i = Σ_j (v_j − v_i) × ∇W_ij.
        omega_.assign(particles_.size(), Vec3f(0.0f));
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            Vec3f omega(0.0f);
            for (const uint32_t j : neighbor_search_.neighbors(i))
            {
                const Vec3f r_vec = predicted_[i] - predicted_[j];
                const float r = r_vec.length();
                const Vec3f vij = particles_[j].velocity - particles_[i].velocity;
                omega += vij.cross(spiky_gradient(r_vec, r, h));
            }
            omega_[i] = omega;
        }

        // η_i = ∇|ω| ≈ Σ_j |ω_j|·∇W_ij ; f_i = ε·(η̂_i × ω_i); v_i += dt·f_i.
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            Vec3f eta(0.0f);
            for (const uint32_t j : neighbor_search_.neighbors(i))
            {
                const Vec3f r_vec = predicted_[i] - predicted_[j];
                const float r = r_vec.length();
                eta += spiky_gradient(r_vec, r, h) * omega_[j].length();
            }
            const float eta_len = eta.length();
            if (eta_len > kDensityEpsilon)
            {
                const Vec3f n = eta * (1.0f / eta_len);
                const Vec3f force = n.cross(omega_[i]) * eps;
                particles_[i].velocity += force * dt;
            }
        }
    }

    /// Density at the predicted positions: ρ_i = Σ_j m_j·poly6(‖r*_ij‖², h),
    /// including the self term. Stored into `FluidParticle::density`.
    void compute_densities()
    {
        const float h = params_.sph.smoothing_radius;
        const float self_w = poly6(0.0f, h);
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            float density = particles_[i].mass * self_w;
            for (const uint32_t j : neighbor_search_.neighbors(i))
            {
                const float r2 = (predicted_[i] - predicted_[j]).squaredLength();
                density += particles_[j].mass * poly6(r2, h);
            }
            particles_[i].density = density;
        }
    }

private:
    /// λ_i = −C_i / (Σ_k ‖∇_{p_k} C_i‖² + ε), with C_i = ρ_i/ρ₀ − 1.
    void compute_lambdas()
    {
        const float h = params_.sph.smoothing_radius;
        const float rho0 = params_.sph.rest_density;
        const float inv_rho0 = (rho0 > 0.0f) ? 1.0f / rho0 : 0.0f;
        const float eps = params_.relaxation;

        lambda_.assign(particles_.size(), 0.0f);
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            const float c_i = particles_[i].density * inv_rho0 - 1.0f;

            // Compression-only: under-dense particles generate no correction, so
            // the free surface is not pulled inward (see clamp_density_deficiency).
            if (params_.clamp_density_deficiency && c_i < 0.0f)
            {
                lambda_[i] = 0.0f;
                continue;
            }

            // ∇_{p_i} C_i = (1/ρ₀) Σ_j m_j ∇W_ij  ;  ∇_{p_j} C_i = −(1/ρ₀) m_j ∇W_ij
            Vec3f grad_i(0.0f);
            float sum_grad2 = 0.0f;
            for (const uint32_t j : neighbor_search_.neighbors(i))
            {
                const Vec3f r_vec = predicted_[i] - predicted_[j];
                const float r = r_vec.length();
                const Vec3f grad = spiky_gradient(r_vec, r, h) * (particles_[j].mass * inv_rho0);
                grad_i += grad;
                sum_grad2 += grad.squaredLength(); // ‖∇_{p_j} C_i‖²
            }
            sum_grad2 += grad_i.squaredLength(); // ‖∇_{p_i} C_i‖²

            // Guard the CFM denominator: with relaxation ε = 0 an isolated
            // particle (zero gradient sum) would divide by zero and produce a
            // non-finite λ. A degenerate denominator ⇒ no correction (λ = 0).
            const float denom = sum_grad2 + eps;
            lambda_[i] = (denom > kDensityEpsilon) ? -c_i / denom : 0.0f;
        }
    }

    /// Δp_i = (1/ρ₀) Σ_j (λ_i + λ_j)·∇W_ij, applied to the predicted positions,
    /// then re-projected against the container walls.
    void compute_and_apply_deltas()
    {
        const float h = params_.sph.smoothing_radius;
        const float rho0 = params_.sph.rest_density;
        const float inv_rho0 = (rho0 > 0.0f) ? 1.0f / rho0 : 0.0f;

        // Precompute the tensile-correction reference weight W(Δq) once.
        const bool use_scorr = params_.scorr_k > 0.0f;
        const float dq = params_.scorr_dq * h;
        const float w_dq = poly6(dq * dq, h);
        const bool scorr_ok = use_scorr && w_dq > 0.0f;

        delta_position_.assign(particles_.size(), Vec3f(0.0f));
        for (size_t i = 0; i < particles_.size(); ++i)
        {
            Vec3f dp(0.0f);
            for (const uint32_t j : neighbor_search_.neighbors(i))
            {
                const Vec3f r_vec = predicted_[i] - predicted_[j];
                const float r = r_vec.length();

                float s_corr = 0.0f;
                if (scorr_ok)
                {
                    // s_corr = −k·(W(r)/W(Δq))ⁿ  — an artificial repulsive pressure.
                    const float ratio = poly6(r * r, h) / w_dq;
                    s_corr = -params_.scorr_k * std::pow(ratio, params_.scorr_n);
                }

                const float scale = (lambda_[i] + lambda_[j] + s_corr) * inv_rho0;
                dp += spiky_gradient(r_vec, r, h) * scale;
            }
            delta_position_[i] = dp;
        }

        for (size_t i = 0; i < particles_.size(); ++i)
        {
            predicted_[i] += delta_position_[i];
            clamp_to_bounds(predicted_[i]);
        }
    }

    /// Clamp a predicted position inside the container. Mirrors the position
    /// half of SphFluidSystem::resolve_boundaries (velocity is recovered from
    /// the position change in finalize(), so no explicit reflection is needed).
    void clamp_to_bounds(Vec3f &pos) const
    {
        const Vec3f &lo = params_.sph.bounds.min;
        const Vec3f &hi = params_.sph.bounds.max;
        for (int axis = 0; axis < 3; ++axis)
        {
            if (pos[axis] < lo[axis])
            {
                pos[axis] = lo[axis];
            }
            else if (pos[axis] > hi[axis])
            {
                pos[axis] = hi[axis];
            }
        }
    }

    /// Densities below this are treated as degenerate and skipped to avoid
    /// dividing by ρ in the XSPH/vorticity accumulation.
    static constexpr float kDensityEpsilon = 1e-6f;

    std::vector<FluidParticle> particles_;
    FluidNeighborSearch neighbor_search_;
    PbfParameters params_{};
    Vec3f ambient_gravity_{EARTH_GRAVITY_VECTOR};

    // PBF scratch (parallel arrays keep FluidParticle lean).
    std::vector<Vec3f> predicted_;
    std::vector<float> lambda_;
    std::vector<Vec3f> delta_position_;
    std::vector<Vec3f> xsph_delta_;
    std::vector<Vec3f> omega_;
};

} // namespace phynity::physics::fluids

#pragma once

#include <core/diagnostics/profiling_macros.hpp>
#include <core/physics/collision/broadphase/spatial_grid.hpp>
#include <core/physics/collision/contact/contact_cache.hpp>
#include <core/physics/collision/contact/impulse_resolver.hpp>
#include <core/physics/collision/contact/pgs_solver.hpp>
#include <core/physics/collision/narrowphase/sphere_sphere_narrowphase.hpp>
#include <core/physics/collision/shapes/sphere_collider.hpp>
#include <core/physics/common/ccd_config.hpp>
#include <core/physics/constraints/contact/contact_constraint.hpp>
#include <core/physics/constraints/solver/constraint.hpp>
#include <core/physics/constraints/solver/constraint_solver.hpp>
#include <core/physics/micro/particle.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <unordered_set>
#include <vector>

namespace phynity::physics
{

using phynity::math::vectors::Vec3f;

/// Context passed from ParticleSystem to the collision resolver for constraint solving.
struct ParticleCollisionContext
{
    constraints::ConstraintSolver &constraint_solver;
    std::vector<std::unique_ptr<constraints::Constraint>> &constraints;
    bool constraints_enabled;
};

/// Collision statistics reported back to ParticleSystem for diagnostics.
struct ParticleCollisionStats
{
    uint32_t broadphase_candidates = 0;
    uint32_t narrowphase_tests = 0;
    uint32_t actual_collisions = 0;
};

/// Handles broadphase, narrowphase, CCD, contact caching, and constraint-based
/// collision resolution for the particle system.
/// Extracted from ParticleSystem to reduce class complexity.
class ParticleCollisionResolver
{
public:
    explicit ParticleCollisionResolver(float cell_size = 2.0f) : spatial_grid_(cell_size), broadphase_cell_size_(cell_size)
    {
    }

    // ========================================================================
    // Main resolve entry point
    // ========================================================================

    /// Run the full collision detection and resolution pipeline.
    void resolve(std::vector<Particle> &particles,
                 float dt,
                 const CCDConfig &ccd_config,
                 const ParticleCollisionContext &ctx)
    {
        using namespace phynity::physics::collision;

        const size_t count = particles.size();

        // Phase 1: Build broadphase spatial grid
        {
            PROFILE_SCOPE("broadphase_grid_build");
            spatial_grid_.clear();
            for (size_t i = 0; i < count; ++i)
            {
                const Particle &p = particles[i];
                if (p.is_alive())
                {
                    spatial_grid_.insert(static_cast<uint32_t>(i), p.position);
                }
            }
        }

        // Phase 2: Collision detection using broadphase culling + narrowphase
        std::unordered_set<uint64_t> processed_pairs;
        processed_pairs.reserve(count * 2);
        last_stats_ = {};

        std::vector<ContactManifold> detected_manifolds;

        for (size_t i = 0; i < count; ++i)
        {
            Particle &a = particles[i];
            if (!a.is_alive())
            {
                continue;
            }

            const auto candidates = spatial_grid_.get_neighbor_objects_tracked(a.position);
            last_stats_.broadphase_candidates += static_cast<uint32_t>(candidates.size());

            for (uint32_t j_index : candidates)
            {
                const auto j = static_cast<size_t>(j_index);

                if (i >= j)
                {
                    continue;
                }

                Particle &b = particles[j];
                if (!b.is_alive())
                {
                    continue;
                }

                const uint64_t pair_id = (static_cast<uint64_t>(i) << 32) | static_cast<uint32_t>(j);
                if (processed_pairs.count(pair_id) > 0)
                {
                    continue;
                }
                processed_pairs.insert(pair_id);

                SphereCollider collider_a = particle_to_collider(a);
                SphereCollider collider_b = particle_to_collider(b);

                ++last_stats_.narrowphase_tests;
                ContactManifold manifold =
                    collision::SphereSpherNarrowphase::detect_with_ccd(collider_a, collider_b, i, j, dt, ccd_config);

                if (manifold.is_valid())
                {
                    detected_manifolds.push_back(manifold);
                }
            }
        }

        // Phase 2.5: CCD fallback scan for fast movers
        if (ccd_config.enabled && dt > 1e-8f)
        {
            PROFILE_SCOPE("ccd_fallback_scan");

            std::vector<float> particle_speeds;
            particle_speeds.reserve(count);
            for (size_t i = 0; i < count; ++i)
            {
                if (particles[i].is_alive())
                {
                    particle_speeds.push_back(particles[i].velocity.length());
                }
                else
                {
                    particle_speeds.push_back(0.0f);
                }
            }

            const float ultra_fast_threshold = std::max(ccd_config.velocity_threshold * 5.0f, 50.0f);

            for (size_t i = 0; i < count; ++i)
            {
                Particle &a = particles[i];
                if (!a.is_alive())
                {
                    continue;
                }

                float speed_a = particle_speeds[i];
                if (speed_a < ultra_fast_threshold)
                {
                    continue;
                }

                for (size_t j = i + 1; j < count; ++j)
                {
                    Particle &b = particles[j];
                    if (!b.is_alive())
                    {
                        continue;
                    }

                    float speed_b = particle_speeds[j];
                    float max_speed = std::max(speed_a, speed_b);

                    if (max_speed < ultra_fast_threshold)
                    {
                        continue;
                    }

                    const uint64_t pair_id = (static_cast<uint64_t>(i) << 32) | static_cast<uint32_t>(j);
                    if (processed_pairs.count(pair_id) > 0)
                    {
                        continue;
                    }

                    Vec3f delta_now = b.position - a.position;
                    float distance_sq = delta_now.dot(delta_now);
                    float max_travel = (speed_a + speed_b) * dt;
                    float combined_radius = a.radius + b.radius;
                    float max_reach = combined_radius + max_travel + ccd_config.speculative_distance;

                    if (distance_sq > max_reach * max_reach)
                    {
                        continue;
                    }

                    SphereCollider collider_a = particle_to_collider(a);
                    SphereCollider collider_b = particle_to_collider(b);
                    collider_a.position = a.position - a.velocity * dt;
                    collider_b.position = b.position - b.velocity * dt;

                    ++last_stats_.narrowphase_tests;
                    ContactManifold manifold =
                        collision::SphereSpherNarrowphase::detect_with_ccd(collider_a, collider_b, i, j, dt, ccd_config);

                    processed_pairs.insert(pair_id);
                    if (manifold.is_valid())
                    {
                        detected_manifolds.push_back(manifold);
                    }
                }
            }
        }

        // Phase 3.5: Update manifolds through contact cache (applies warm-start data)
        auto cached_manifolds = contact_cache_.update_tracked(detected_manifolds);
        last_stats_.actual_collisions = static_cast<uint32_t>(cached_manifolds.size());

        // Phase 3.75: CCD Sub-stepping
        if (ccd_config.enabled && ccd_config.max_substeps > 0)
        {
            PROFILE_SCOPE("ccd_substepping");
            perform_ccd_substeps(cached_manifolds, ccd_config);
        }

        // Phase 4: Unified Constraint Solving
        if (!cached_manifolds.empty() || !ctx.constraints.empty())
        {
            PROFILE_SCOPE("constraint_solving");

            std::vector<std::unique_ptr<constraints::Constraint>> temp_constraints;
            temp_constraints.reserve(cached_manifolds.size());

            for (const ContactManifold &manifold : cached_manifolds)
            {
                if (manifold.object_a_id < particles.size() && manifold.object_b_id < particles.size())
                {
                    auto contact_constraint = std::make_unique<constraints::ContactConstraint>(
                        manifold,
                        particles[manifold.object_a_id],
                        particles[manifold.object_b_id],
                        constraints::ContactConstraint::ContactType::Normal);
                    temp_constraints.push_back(std::move(contact_constraint));
                }
            }

            ctx.constraint_solver.solve(temp_constraints, particles);

            for (size_t i = 0; i < temp_constraints.size() && i < cached_manifolds.size(); ++i)
            {
                const auto &constraint = temp_constraints[i];
                const auto &manifold = cached_manifolds[i];
                float impulse = constraint->get_accumulated_impulse();
                contact_cache_.store_impulse(manifold.contact_id, Vec3f(impulse, 0.0f, 0.0f));
            }

            if (ctx.constraints_enabled)
            {
                ctx.constraint_solver.solve(ctx.constraints, particles);
            }
        }
    }

    // ========================================================================
    // Configuration
    // ========================================================================

    void set_cell_size(float cell_size)
    {
        if (cell_size > 0.0f)
        {
            broadphase_cell_size_ = cell_size;
            spatial_grid_.set_cell_size(cell_size);
        }
    }

    float cell_size() const
    {
        return broadphase_cell_size_;
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    const ParticleCollisionStats &last_stats() const
    {
        return last_stats_;
    }

private:
    collision::SpatialGrid spatial_grid_;
    collision::ContactCache contact_cache_;
    float broadphase_cell_size_;
    ParticleCollisionStats last_stats_;

    static collision::SphereCollider particle_to_collider(const Particle &p)
    {
        collision::SphereCollider collider;
        collider.position = p.position;
        collider.velocity = p.velocity;
        collider.radius = p.radius;
        collider.inverse_mass = p.inverse_mass();
        collider.restitution = p.material.restitution;
        return collider;
    }

    static void perform_ccd_substeps(std::vector<collision::ContactManifold> &manifolds,
                                     const CCDConfig &ccd_config) noexcept
    {
        PROFILE_SCOPE("ccd_substeps");

        if (manifolds.empty() || !ccd_config.enabled)
        {
            return;
        }

        std::sort(manifolds.begin(),
                  manifolds.end(),
                  [](const collision::ContactManifold &a, const collision::ContactManifold &b)
                  { return a.toi < b.toi; });
    }
};

} // namespace phynity::physics

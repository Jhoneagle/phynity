#pragma once

#include <core/physics/particle.hpp>
#include <core/physics/force_field.hpp>
#include <core/physics/physics_constants.hpp>
#include <core/physics/collision/sphere_collider.hpp>
#include <core/physics/collision/sphere_sphere_narrowphase.hpp>
#include <core/physics/collision/impulse_resolver.hpp>
#include <core/physics/collision/spatial_grid.hpp>
#include <core/physics/collision/contact_cache.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <core/jobs/job_system.hpp>
#include <core/diagnostics/energy_monitor.hpp>
#include <core/diagnostics/momentum_monitor.hpp>
#include <core/diagnostics/collision_monitor.hpp>
#include <algorithm>
#include <memory>
#include <vector>
#include <unordered_set>

namespace phynity::physics {

using namespace phynity::physics::constants;

/// Manages a collection of particles and force fields, providing simulation stepping.
/// Integrates Material system, ForceField system, and provides energy/momentum diagnostics.
class ParticleSystem {
public:
    /// Diagnostic information about the particle system
    struct Diagnostics {
        float total_kinetic_energy = 0.0f;  ///< Sum of all particle kinetic energies
        Vec3f total_momentum = Vec3f(0.0f); ///< Sum of all particle momenta (mass * velocity)
        size_t particle_count = 0;          ///< Number of active particles
    };

    ParticleSystem() = default;
    
    // Move semantics
    ParticleSystem(ParticleSystem&& other) noexcept = default;
    ParticleSystem& operator=(ParticleSystem&& other) noexcept = default;

    // ========================================================================
    // Particle Management
    // ========================================================================

    /// Spawn a new particle at position with initial velocity.
    /// @param position Starting position
    /// @param velocity Starting velocity
    /// @param mass Particle mass (default: 1.0)
    /// @param lifetime Particle lifetime (-1 = infinite, > 0 = finite)
    void spawn(
        const Vec3f& position,
        const Vec3f& velocity,
        float mass = 1.0f,
        float lifetime = -1.0f,
        float radius = -1.0f
    ) {
        particles_.emplace_back();
        Particle& p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material.mass = mass;
        p.lifetime = lifetime;
        p.radius = (radius > 0.0f) ? radius : default_collision_radius_;
    }

    /// Spawn a new particle with full material specification.
    /// @param position Starting position
    /// @param velocity Starting velocity
    /// @param material Complete material definition
    /// @param lifetime Particle lifetime (-1 = infinite, > 0 = finite)
    void spawn(
        const Vec3f& position,
        const Vec3f& velocity,
        const Material& material,
        float lifetime = -1.0f,
        float radius = -1.0f
    ) {
        particles_.emplace_back();
        Particle& p = particles_.back();
        p.position = position;
        p.velocity = velocity;
        p.material = material;
        p.lifetime = lifetime;
        p.radius = (radius > 0.0f) ? radius : default_collision_radius_;
    }

    /// Clear all particles.
    void clear() { 
        particles_.clear(); 
    }

    /// Remove dead particles from the system.
    /// WARNING: This invalidates any outstanding iterators or references to particles.
    /// Do not keep references to particles obtained via particles() before calling this method.
    /// The internal particle storage may be reordered or resized during removal.
    void remove_dead_particles() {
        particles_.erase(
            std::remove_if(particles_.begin(), particles_.end(),
                          [](const Particle& p) { return !p.is_alive(); }),
            particles_.end()
        );
    }

    // ========================================================================
    // Force Field Management
    // ========================================================================

    /// Add a force field to the system.
    /// The system takes ownership of the field.
    /// @param field Unique pointer to force field
    void add_force_field(std::unique_ptr<ForceField> field) {
        force_fields_.push_back(std::move(field));
    }

    /// Remove all force fields from the system.
    void clear_force_fields() {
        force_fields_.clear();
    }

    /// Get the number of active force fields.
    size_t force_field_count() const {
        return force_fields_.size();
    }

    // ========================================================================
    // Collision Management
    // ========================================================================

    /// Enable or disable simple sphere-sphere collision handling.
    void enable_collisions(bool enabled) {
        collisions_enabled_ = enabled;
    }

    /// Check whether collisions are enabled.
    bool collisions_enabled() const {
        return collisions_enabled_;
    }

    /// Set default collision radius used by spawn() when radius is not specified.
    void set_default_collision_radius(float radius) {
        if (radius > 0.0f) {
            default_collision_radius_ = radius;
        }
    }

    /// Get the current default collision radius.
    float default_collision_radius() const {
        return default_collision_radius_;
    }

    /// Set the broadphase grid cell size (in world units).
    /// Larger cells = fewer cells, faster insertion but more candidates per query.
    /// Smaller cells = more cells, slower insertion but fewer candidates.
    /// Recommended: 2x to 4x the average particle diameter.
    void set_broadphase_cell_size(float cell_size) {
        if (cell_size > 0.0f) {
            broadphase_cell_size_ = cell_size;
            spatial_grid_.set_cell_size(cell_size);
        }
    }

    /// Get the current broadphase grid cell size.
    float broadphase_cell_size() const {
        return broadphase_cell_size_;
    }

    // ========================================================================
    // Simulation Update
    // ========================================================================

    /// Update the simulation by one timestep.
    /// Order of operations:
    /// 1. Clear all particle forces
    /// 2. Apply each force field to each particle
    /// 3. Update particle accelerations from accumulated forces
    /// 4. Integrate particle positions/velocities
    /// 5. Remove dead particles
    /// @param dt Time step in seconds
    void update(float dt) {
        PROFILE_FUNCTION();

        auto for_each_alive = [this](auto&& fn) {
            const size_t count = particles_.size();
            if (job_system_ && job_system_->is_running()) {
                job_system_->parallel_for(0, static_cast<uint32_t>(count), 1, [&](uint32_t i) {
                    Particle& p = particles_[i];
                    if (p.is_alive()) {
                        fn(p);
                    }
                });
            } else {
                for (auto& p : particles_) {
                    if (p.is_alive()) {
                        fn(p);
                    }
                }
            }
        };
        
        // Step 1: Clear forces from previous frame
        {
            PROFILE_SCOPE("clear_forces");
            for_each_alive([](Particle& p) { p.clear_forces(); });
        }

        // Step 2: Apply all force fields to all particles
        {
            PROFILE_SCOPE("apply_force_fields");
            for (const auto& field : force_fields_) {
                if (job_system_ && job_system_->is_running()) {
                    const size_t count = particles_.size();
                    job_system_->parallel_for(0, static_cast<uint32_t>(count), 1, [&](uint32_t i) {
                        Particle& p = particles_[i];
                        if (!p.is_alive()) {
                            return;
                        }
                        Vec3f force = field->apply(p.position, p.velocity, p.material.mass);
                        p.apply_force(force);
                    });
                } else {
                    for (auto& p : particles_) {
                        if (p.is_alive()) {
                            Vec3f force = field->apply(p.position, p.velocity, p.material.mass);
                            p.apply_force(force);
                        }
                    }
                }
            }
        }

        // Step 3: Update accelerations from accumulated forces
        {
            PROFILE_SCOPE("update_accelerations");
            for_each_alive([](Particle& p) { p.update_acceleration(); });
        }

        // Step 4: Integrate particle state
        {
            PROFILE_SCOPE("integration");
            for_each_alive([dt](Particle& p) { p.integrate(dt); });
        }

        // Step 5: Resolve collisions (optional)
        if (collisions_enabled_) {
            PROFILE_SCOPE("collision_resolution");
            resolve_collisions();
        }

        // Step 6: Monitor physics (energy, momentum)
        if (energy_monitor_enabled_ && energy_monitor_) {
            PROFILE_SCOPE("energy_monitoring");
            const Diagnostics diag = compute_diagnostics();
            // Include potential energy from gravity fields if present
            float total_energy = diag.total_kinetic_energy;
            energy_monitor_->update(static_cast<double>(total_energy));
        }

        if (momentum_monitor_enabled_ && momentum_monitor_) {
            PROFILE_SCOPE("momentum_monitoring");
            const Diagnostics diag = compute_diagnostics();
            diagnostics::Vec3 momentum(diag.total_momentum.x, diag.total_momentum.y, diag.total_momentum.z);
            momentum_monitor_->update(momentum);
        }

        // Step 7: Remove dead particles
        {
            PROFILE_SCOPE("remove_dead_particles");
            remove_dead_particles();
        }
    }

    /// Legacy step method for backwards compatibility.
    /// @deprecated Use update() instead
    void step(float dt) {
        update(dt);
    }

    /// Apply gravity to all particles (legacy method).
    /// @deprecated Add a GravityField instead
    void applyGravity(const Vec3f& gravity) {
        for (auto& p : particles_) {
            if (p.is_alive()) {
                p.apply_force(gravity * p.material.mass);
            }
        }
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Compute current system diagnostics (energy, momentum).
    /// @return Diagnostics struct with current values
    Diagnostics compute_diagnostics() const {
        Diagnostics diag;
        diag.particle_count = particles_.size();
        diag.total_kinetic_energy = 0.0f;
        diag.total_momentum = Vec3f(0.0f);

        for (const auto& p : particles_) {
            if (p.is_alive()) {
                diag.total_kinetic_energy += p.kinetic_energy();
                diag.total_momentum += p.velocity * p.material.mass;
            }
        }

        return diag;
    }

    // ========================================================================
    // Physics Monitoring (Optional)
    // ========================================================================

    /// Enable energy conservation monitoring.
    /// Monitors total system energy and detects violations (excess loss/gain).
    void enable_energy_monitor(std::shared_ptr<diagnostics::EnergyMonitor> monitor) {
        energy_monitor_ = monitor;
        energy_monitor_enabled_ = true;
    }

    /// Disable energy monitoring.
    void disable_energy_monitor() {
        energy_monitor_enabled_ = false;
    }

    /// Enable momentum conservation monitoring.
    /// Monitors total system momentum and detects unexpected changes.
    void enable_momentum_monitor(std::shared_ptr<diagnostics::MomentumMonitor> monitor) {
        momentum_monitor_ = monitor;
        momentum_monitor_enabled_ = true;
    }

    /// Disable momentum monitoring.
    void disable_momentum_monitor() {
        momentum_monitor_enabled_ = false;
    }

    /// Enable collision efficiency monitoring.
    /// Tracks broadphase/narrowphase efficiency to detect poor grid configuration.
    void enable_collision_monitor(std::shared_ptr<diagnostics::CollisionMonitor> monitor) {
        collision_monitor_ = monitor;
        collision_monitor_enabled_ = true;
    }

    /// Disable collision monitoring.
    void disable_collision_monitor() {
        collision_monitor_enabled_ = false;
    }

    // ========================================================================
    // Job System Integration
    // ========================================================================

    /// Provide a job system for optional parallel update passes.
    /// The system is not owned by ParticleSystem.
    void set_job_system(phynity::jobs::JobSystem* job_system) {
        job_system_ = job_system;
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Get particle count.
    size_t particleCount() const { return particles_.size(); }

    /// Get all particles (const access).
    const std::vector<Particle>& particles() const { return particles_; }

    /// Get all particles (mutable access).
    std::vector<Particle>& particles() { return particles_; }

private:
    std::vector<Particle> particles_;
    std::vector<std::unique_ptr<ForceField>> force_fields_;
    phynity::jobs::JobSystem* job_system_ = nullptr;
    collision::SpatialGrid spatial_grid_{2.0f};  // Default cell size: 2x particle radius
    collision::ContactCache contact_cache_;      // Contact cache for frame-to-frame tracking (Phase 3)
    bool collisions_enabled_ = false;
    float default_collision_radius_ = 0.5f;
    float broadphase_cell_size_ = 2.0f;

    // Optional diagnostics monitors
    std::shared_ptr<diagnostics::EnergyMonitor> energy_monitor_;
    std::shared_ptr<diagnostics::MomentumMonitor> momentum_monitor_;
    std::shared_ptr<diagnostics::CollisionMonitor> collision_monitor_;
    bool energy_monitor_enabled_ = false;
    bool momentum_monitor_enabled_ = false;
    bool collision_monitor_enabled_ = false;

    /// Convert a Particle to a SphereCollider for generic collision handling
    static collision::SphereCollider particle_to_collider(const Particle& p) {
        collision::SphereCollider collider;
        collider.position = p.position;
        collider.velocity = p.velocity;
        collider.radius = p.radius;
        collider.inverse_mass = p.inverse_mass();
        collider.restitution = p.material.restitution;
        return collider;
    }

    /// Apply collision results back to a Particle
    static void apply_collider_to_particle(const collision::SphereCollider& collider, Particle& p) {
        p.position = collider.position;
        p.velocity = collider.velocity;
    }

    /// Internal collision resolution using spatial grid broadphase + narrowphase
    ///
    /// Algorithm:
    /// 1. Build spatial grid by inserting all particles at their positions
    /// 2. For each particle, query neighbors from spatial grid (3x3x3 cell neighborhood)
    /// 3. Run narrowphase collision detection on candidate pairs
    /// 4. Resolve collisions using impulse-based contact resolution
    ///
    /// Pair Deduplication Strategy:
    /// - Spatial grid returns all objects in neighboring cells (may include duplicates)
    /// - Each particle queries its neighbors, so pairs (i,j) may be found from both i and j queries
    /// - Deduplication uses canonical ordering: only process pairs where i < j
    /// - Hash set tracks processed pairs to avoid redundant narrowphase calls
    /// - Pair ID encoding: (i << 32) | j, ensuring unique 64-bit identifier
    ///
    /// Performance:
    /// - Broadphase: O(n) grid insertion + O(n * k) neighbor queries, k = avg neighbors
    /// - Narrowphase: O(m) where m = unique candidate pairs (typically m << n²)
    /// - Advantage over brute force: k << n, so total is O(n * k) vs O(n²)
    void resolve_collisions() {
        using namespace phynity::physics::collision;
        
        // Phase 1: Build broadphase spatial grid
        // Clear previous frame's spatial structure and re-insert all alive particles
        {
            PROFILE_SCOPE("broadphase_grid_build");
            spatial_grid_.clear();
            const size_t count = particles_.size();
            for (size_t i = 0; i < count; ++i) {
                const Particle& p = particles_[i];
                if (p.is_alive()) {
                    spatial_grid_.insert(static_cast<uint32_t>(i), p.position);
                }
            }
        }

        // Phase 2: Collision detection using broadphase culling + narrowphase
        // Track processed pairs to avoid duplicates (same pair from different queries)
        // Hash set provides O(1) lookup for pair deduplication
        std::unordered_set<uint64_t> processed_pairs;
        uint32_t broadphase_candidates = 0;
        uint32_t narrowphase_tests = 0;
        uint32_t actual_collisions = 0;
        const size_t count = particles_.size();

        // Phase 3: Collect all detected manifolds (instead of resolving immediately)
        std::vector<ContactManifold> detected_manifolds;

        for (size_t i = 0; i < count; ++i) {
            Particle& a = particles_[i];
            if (!a.is_alive()) {
                continue;
            }

            // Get candidate neighbors from spatial grid (3x3x3 cell neighborhood)
            const auto candidates = spatial_grid_.get_neighbor_objects(a.position);
            broadphase_candidates += static_cast<uint32_t>(candidates.size());

            for (uint32_t j_index : candidates) {
                const auto j = static_cast<size_t>(j_index);
                
                // Canonical ordering: only process pairs where i < j
                // This ensures each pair is considered exactly once
                if (i >= j) {
                    continue;  // Skip self-collisions (i==j) and reversed pairs (i>j)
                }

                Particle& b = particles_[j];
                if (!b.is_alive()) {
                    continue;
                }

                // Create unique pair ID for deduplication (i is guaranteed < j)
                // Encoding: high 32 bits = i, low 32 bits = j
                const uint64_t pair_id = (static_cast<uint64_t>(i) << 32) | static_cast<uint32_t>(j);
                if (processed_pairs.count(pair_id) > 0) {
                    continue;  // Already processed this pair from a different grid cell query
                }
                processed_pairs.insert(pair_id);

                // Convert particles to generic colliders for collision detection
                SphereCollider collider_a = particle_to_collider(a);
                SphereCollider collider_b = particle_to_collider(b);

                // Detect collision using generic narrowphase
                ++narrowphase_tests;
                ContactManifold manifold = SphereSpherNarrowphase::detect(collider_a, collider_b, i, j);

                // Collect valid manifolds for caching
                if (manifold.is_valid()) {
                    detected_manifolds.push_back(manifold);
                }
            }
        }

        // Phase 3.5: Update manifolds through contact cache (applies warm-start data)
        std::vector<ContactManifold> cached_manifolds = contact_cache_.update(detected_manifolds);

        // Phase 4: Resolve all cached manifolds and store applied impulses
        for (const ContactManifold& manifold : cached_manifolds) {
            ++actual_collisions;

            // Get particles for this contact
            Particle& a = particles_[manifold.object_a_id];
            Particle& b = particles_[manifold.object_b_id];

            // Convert to colliders
            SphereCollider collider_a = particle_to_collider(a);
            SphereCollider collider_b = particle_to_collider(b);

            // Resolve and get applied impulse
            Vec3f applied_impulse = ImpulseResolver::resolve(manifold, collider_a, collider_b);

            // Store impulse in cache for next frame's warm-start
            contact_cache_.store_impulse(manifold.contact_id, applied_impulse);

            // Apply results back to particles
            apply_collider_to_particle(collider_a, a);
            apply_collider_to_particle(collider_b, b);
        }

        // Report collision statistics to monitor
        if (collision_monitor_enabled_ && collision_monitor_) {
            collision_monitor_->set_broadphase_candidates(broadphase_candidates);
            collision_monitor_->set_narrowphase_tests(narrowphase_tests);
            collision_monitor_->set_actual_collisions(actual_collisions);
            collision_monitor_->end_frame();
        }
    }
};

}  // namespace phynity::physics

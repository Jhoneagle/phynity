#pragma once

#include <core/physics/rigid_body.hpp>
#include <core/physics/force_field.hpp>
#include <core/physics/inertia.hpp>
#include <core/physics/constraints/constraint.hpp>
#include <core/physics/constraints/contact_constraint.hpp>
#include <core/physics/constraints/constraint_solver.hpp>
#include <core/physics/collision/spatial_grid.hpp>
#include <core/physics/collision/sphere_sphere_narrowphase.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <core/diagnostics/energy_monitor.hpp>
#include <core/diagnostics/momentum_monitor.hpp>
#include <memory>
#include <vector>

namespace phynity::physics {

using phynity::physics::constraints::Constraint;
using phynity::physics::constraints::ConstraintSolver;
using phynity::physics::constraints::ConstraintSolverConfig;
using phynity::physics::collision::SpatialGrid;

/// Manages a collection of rigid bodies, force fields, and constraints.
/// Handles 6-DOF rigid body dynamics with rotational integration,
/// collision detection, and constraint solving.
class RigidBodySystem {
public:
    // ========================================================================
    // Configuration
    // ========================================================================

    struct Config {
        float gravity_acceleration = 9.81f;  ///< Gravitational acceleration (m/s²)
        int broadphase_grid_resolution = 20; ///< Spatial grid cell count
        float broadphase_grid_size = 100.0f; ///< Total grid size
        float default_collision_radius = 1.0f; ///< Default broadphase radius for new bodies
        
        constexpr Config() = default;
    };

    /// Constructor with default configuration
    explicit RigidBodySystem()
        : RigidBodySystem(Config{})
    {
    }

    /// Constructor with custom configuration
    explicit RigidBodySystem(const Config& config)
        : config_(config),
          spatial_grid_(
              config.broadphase_grid_size / static_cast<float>(config.broadphase_grid_resolution)
          ),
          next_body_id_(0)
    {
    }

    // ========================================================================
    // Body Management
    // ========================================================================

    /// Spawn a new rigid body
    /// @param position Starting position in world space
    /// @param orientation Starting orientation as quaternion
    /// @param shape Collision shape (sphere, box, capsule)
    /// @param mass Mass in kilograms
    /// @param material Physical properties (restitution, friction)
    /// @return ID of the created body
    RigidBodyID spawn_body(
        const Vec3f& position,
        const Quatf& orientation,
        std::shared_ptr<Shape> shape,
        float mass = 1.0f,
        const Material& material = Material{}
    ) {
        PROFILE_SCOPE("RigidBodySystem::spawn_body");
        
        bodies_.emplace_back();
        RigidBody& rb = bodies_.back();
        
        rb.position = position;
        rb.orientation = orientation;
        rb.shape = shape;
        rb.material = material;
        rb.id = next_body_id_;
        rb.collision_radius = (shape) ? shape->get_bounding_radius() : config_.default_collision_radius;
        
        rb.set_mass(mass);
        
        return next_body_id_++;
    }

    /// Get a rigid body by ID
    RigidBody* get_body(RigidBodyID id) {
        for (auto& rb : bodies_) {
            if (rb.id == id) {
                return &rb;
            }
        }
        return nullptr;
    }

    /// Get total number of active bodies
    size_t body_count() const {
        return bodies_.size();
    }

    // ========================================================================
    // Force Fields
    // ========================================================================

    /// Add a force field (gravity, drag, wind, etc.)
    void add_force_field(std::shared_ptr<ForceField> field) {
        force_fields_.push_back(field);
    }

    /// Remove all force fields
    void clear_force_fields() {
        force_fields_.clear();
    }

    // ========================================================================
    // Constraints
    // ========================================================================

    /// Register a constraint (fixed joint, hinge, etc.)
    void add_constraint(std::shared_ptr<Constraint> constraint) {
        constraints_.push_back(constraint);
    }

    /// Get all constraints
    const std::vector<std::shared_ptr<Constraint>>& get_constraints() const {
        return constraints_;
    }

    // ========================================================================
    // Simulation Step
    // ========================================================================

    /// Advance simulation by dt seconds
    void update(float dt) {
        PROFILE_SCOPE("RigidBodySystem::update");
        
        if (bodies_.empty()) {
            return;
        }

        // Phase 1: Clear forces and torques
        {
            PROFILE_SCOPE("RigidBodySystem::clear_forces");
            for (auto& rb : bodies_) {
                rb.clear_forces_and_torques();
            }
        }

        // Phase 2: Apply force fields (gravity, drag, wind, etc.)
        {
            PROFILE_SCOPE("RigidBodySystem::apply_forces");
            for (auto& field : force_fields_) {
                for (auto& rb : bodies_) {
                    Vec3f force = field->apply(
                        rb.position,
                        rb.velocity,
                        rb.get_mass()
                    );
                    rb.force_accumulator += force;
                }
            }
        }

        // Phase 3: Linear integration (semi-implicit Euler)
        {
            PROFILE_SCOPE("RigidBodySystem::linear_integration");
            for (auto& rb : bodies_) {
                if (rb.is_static()) continue;
                
                // v_new = v + (F/m) * dt
                Vec3f acceleration = rb.force_accumulator * rb.inv_mass;
                rb.velocity += acceleration * dt;
                
                // Apply linear damping
                rb.velocity *= (1.0f - rb.material.linear_damping * dt);
                
                // p_new = p + v_new * dt
                rb.position += rb.velocity * dt;
            }
        }

        // Phase 4: Angular integration (quaternion + omega)
        {
            PROFILE_SCOPE("RigidBodySystem::angular_integration");
            for (auto& rb : bodies_) {
                if (rb.is_static()) continue;
                
                // ω_new = ω + (I^-1 * τ) * dt
                Vec3f angular_accel = rb.inertia_tensor_inv * rb.torque_accumulator;
                rb.angular_velocity += angular_accel * dt;
                
                // Apply angular damping
                rb.angular_velocity *= (1.0f - rb.material.angular_damping * dt);
                
                // q_new = q + 0.5 * dt * ω_quat * q
                rb.orientation = phynity::physics::inertia::integrate_quaternion(
                    rb.orientation,
                    rb.angular_velocity,
                    dt
                );
                rb.orientation.normalize();
            }
        }

        // Phase 5: Collision detection
        // (Reuse existing narrowphase for sphere-sphere collisions)
        // TODO: Extend for rigid body shapes (box, capsule) in Phase 5

        // Phase 6: Constraint solving (collisions + joints)
        {
            PROFILE_SCOPE("RigidBodySystem::constraint_solving");
            // TODO: Implement RigidBody constraint solver
            // For MVP: use basic constraint application without iterative solving
            // Full implementation will integrate with unified ConstraintSolver in Phase 4
            for (auto& constraint : constraints_) {
                if (!constraint) continue;
                
                // Solve constraint iteratively
                for (int iter = 0; iter < 4; ++iter) {  // 4 iterations by default
                    float error = constraint->compute_error();
                    if (error < 1e-5f) break;  // Converged
                    
                    constraint->apply_impulse(0.1f);  // Small impulse for MVP
                }
            }
        }

        // Phase 7: Update lifetimes and cleanup
        {
            PROFILE_SCOPE("RigidBodySystem::cleanup");
            for (auto& rb : bodies_) {
                rb.update_lifetime(dt);
            }
            bodies_.erase(
                std::remove_if(
                    bodies_.begin(),
                    bodies_.end(),
                    [](const RigidBody& rb) { return !rb.active; }
                ),
                bodies_.end()
            );
        }

        // Phase 8: Update diagnostics
        {
            PROFILE_SCOPE("RigidBodySystem::diagnostics");
            compute_diagnostics();
        }
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Diagnostic statistics about the system
    struct Diagnostics {
        float total_kinetic_energy = 0.0f;    ///< Linear + angular KE
        float total_linear_ke = 0.0f;
        float total_angular_ke = 0.0f;
        Vec3f total_momentum = Vec3f(0.0f);   ///< Linear momentum (m*v)
        Vec3f total_angular_momentum = Vec3f(0.0f); ///< Angular momentum (I*ω)
        size_t body_count = 0;
    };

    /// Get current diagnostics
    const Diagnostics& get_diagnostics() const {
        return diagnostics_;
    }

private:
    // ========================================================================
    // Internal Helpers
    // ========================================================================

    /// Recompute system-wide diagnostics
    void compute_diagnostics() {
        diagnostics_ = Diagnostics{};
        diagnostics_.body_count = bodies_.size();
        
        for (const auto& rb : bodies_) {
            // Linear kinetic energy
            float linear_ke = 0.5f * rb.get_mass() * rb.velocity.squaredLength();
            diagnostics_.total_linear_ke += linear_ke;
            
            // Angular kinetic energy
            float angular_ke = phynity::physics::inertia::compute_angular_kinetic_energy(
                rb.inertia_tensor,
                rb.angular_velocity
            );
            diagnostics_.total_angular_ke += angular_ke;
            
            // Total kinetic energy
            diagnostics_.total_kinetic_energy += linear_ke + angular_ke;
            
            // Linear momentum
            diagnostics_.total_momentum += rb.momentum();
            
            // Angular momentum
            diagnostics_.total_angular_momentum += rb.angular_momentum();
        }
    }

    // ========================================================================
    // Member Variables
    // ========================================================================

    Config config_;
    std::vector<RigidBody> bodies_;
    std::vector<std::shared_ptr<ForceField>> force_fields_;
    std::vector<std::shared_ptr<Constraint>> constraints_;
    SpatialGrid spatial_grid_;
    // TODO: Full ConstraintSolver integration in Phase 4
    // ConstraintSolver constraint_solver_;
    RigidBodyID next_body_id_;
    Diagnostics diagnostics_;
};

}  // namespace phynity::physics

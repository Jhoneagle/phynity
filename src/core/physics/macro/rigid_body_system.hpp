#pragma once

#include <core/diagnostics/energy_monitor.hpp>
#include <core/diagnostics/momentum_monitor.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/physics/collision/broadphase/spatial_grid.hpp>
#include <core/physics/collision/ccd/conservative_advancement.hpp>
#include <core/physics/collision/ccd/convex_sweep.hpp>
#include <core/physics/collision/narrowphase/aabb_narrowphase.hpp>
#include <core/physics/collision/narrowphase/sphere_sphere_narrowphase.hpp>
#include <core/physics/collision/shapes/aabb.hpp>
#include <core/physics/collision/shapes/sphere_collider.hpp>
#include <core/physics/common/ccd_config.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/physics/constraints/contact/contact_constraint.hpp>
#include <core/physics/constraints/solver/constraint.hpp>
#include <core/physics/constraints/solver/constraint_solver.hpp>
#include <core/physics/macro/inertia.hpp>
#include <core/physics/macro/rigid_body.hpp>

#include <memory>
#include <vector>

namespace phynity::physics
{

using phynity::physics::collision::SpatialGrid;
using phynity::physics::constraints::Constraint;
using phynity::physics::constraints::ConstraintSolver;
using phynity::physics::constraints::ConstraintSolverConfig;

/// Manages a collection of rigid bodies, force fields, and constraints.
/// Handles 6-DOF rigid body dynamics with rotational integration,
/// collision detection, and constraint solving.
class RigidBodySystem
{
public:
    // ========================================================================
    // Configuration
    // ========================================================================

    struct Config
    {
        float gravity_acceleration = 9.81f; ///< Gravitational acceleration (m/s²)
        int broadphase_grid_resolution = 20; ///< Spatial grid cell count
        float broadphase_grid_size = 100.0f; ///< Total grid size
        float default_collision_radius = 1.0f; ///< Default broadphase radius for new bodies
        bool enable_linear_ccd = true; ///< Enable linear CCD for fast movers
        CCDConfig ccd_config = ccd_presets::balanced(); ///< CCD thresholds and parameters

        constexpr Config() = default;
    };

    /// Constructor with default configuration
    explicit RigidBodySystem() : RigidBodySystem(Config{})
    {
    }

    /// Constructor with custom configuration
    explicit RigidBodySystem(const Config &config)
        : config_(config),
          spatial_grid_(config.broadphase_grid_size / static_cast<float>(config.broadphase_grid_resolution)),
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
    RigidBodyID spawn_body(const Vec3f &position,
                           const Quatf &orientation,
                           std::shared_ptr<Shape> shape,
                           float mass = 1.0f,
                           const Material &material = Material{})
    {
        PROFILE_SCOPE("RigidBodySystem::spawn_body");

        bodies_.emplace_back();
        RigidBody &rb = bodies_.back();

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
    RigidBody *get_body(RigidBodyID id)
    {
        for (auto &rb : bodies_)
        {
            if (rb.id == id)
            {
                return &rb;
            }
        }
        return nullptr;
    }

    /// Get total number of active bodies
    size_t body_count() const
    {
        return bodies_.size();
    }

    // ========================================================================
    // Force Fields
    // ========================================================================

    /// Add a force field (gravity, drag, wind, etc.)
    void add_force_field(std::shared_ptr<ForceField> field)
    {
        force_fields_.push_back(field);
    }

    /// Remove all force fields
    void clear_force_fields()
    {
        force_fields_.clear();
    }

    // ========================================================================
    // Constraints
    // ========================================================================

    /// Register a constraint (fixed joint, hinge, etc.)
    void add_constraint(std::shared_ptr<Constraint> constraint)
    {
        constraints_.push_back(constraint);
    }

    /// Get all constraints
    const std::vector<std::shared_ptr<Constraint>> &get_constraints() const
    {
        return constraints_;
    }

    // ========================================================================
    // Simulation Step
    // ========================================================================

    /// Advance simulation by dt seconds
    void update(float dt)
    {
        PROFILE_SCOPE("RigidBodySystem::update");

        if (bodies_.empty())
        {
            return;
        }

        std::vector<Vec3f> frame_start_positions;
        frame_start_positions.reserve(bodies_.size());
        for (const auto &body : bodies_)
        {
            frame_start_positions.push_back(body.position);
        }

        // Phase 1: Clear forces and torques
        {
            PROFILE_SCOPE("RigidBodySystem::clear_forces");
            for (auto &rb : bodies_)
            {
                rb.clear_forces_and_torques();
            }
        }

        // Phase 2: Apply force fields (gravity, drag, wind, etc.)
        {
            PROFILE_SCOPE("RigidBodySystem::apply_forces");
            for (auto &field : force_fields_)
            {
                for (auto &rb : bodies_)
                {
                    Vec3f force = field->apply(rb.position, rb.velocity, rb.get_mass());
                    rb.force_accumulator += force;
                }
            }
        }

        // Phase 3: Linear integration (semi-implicit Euler)
        {
            PROFILE_SCOPE("RigidBodySystem::linear_integration");
            for (auto &rb : bodies_)
            {
                if (rb.is_static())
                    continue;

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
            for (auto &rb : bodies_)
            {
                if (rb.is_static())
                    continue;

                // ω_new = ω + (I^-1 * τ) * dt
                Vec3f angular_accel = rb.inertia_tensor_inv * rb.torque_accumulator;
                rb.angular_velocity += angular_accel * dt;

                // Apply angular damping
                rb.angular_velocity *= (1.0f - rb.material.angular_damping * dt);

                // q_new = q + 0.5 * dt * ω_quat * q
                rb.orientation =
                    phynity::physics::inertia::integrate_quaternion(rb.orientation, rb.angular_velocity, dt);
                rb.orientation.normalize();
            }
        }

        // Phase 5: Collision detection
        {
            PROFILE_SCOPE("RigidBodySystem::collisions");
            resolve_collisions(frame_start_positions, dt);
        }

        // Phase 6: Constraint solving (collisions + joints)
        {
            PROFILE_SCOPE("RigidBodySystem::constraint_solving");
            // TODO: Implement RigidBody constraint solver
            // For MVP: use basic constraint application without iterative solving
            // Full implementation will integrate with unified ConstraintSolver in Phase 4
            for (auto &constraint : constraints_)
            {
                if (!constraint)
                    continue;

                // Solve constraint iteratively
                for (int iter = 0; iter < 4; ++iter)
                { // 4 iterations by default
                    float error = constraint->compute_error();
                    if (error < 1e-5f)
                        break; // Converged

                    constraint->apply_impulse(0.1f); // Small impulse for MVP
                }
            }
        }

        // Phase 7: Update lifetimes and cleanup
        {
            PROFILE_SCOPE("RigidBodySystem::cleanup");
            for (auto &rb : bodies_)
            {
                rb.update_lifetime(dt);
            }
            bodies_.erase(
                std::remove_if(bodies_.begin(), bodies_.end(), [](const RigidBody &rb) { return !rb.active; }),
                bodies_.end());
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
    struct Diagnostics
    {
        float total_kinetic_energy = 0.0f; ///< Linear + angular KE
        float total_linear_ke = 0.0f;
        float total_angular_ke = 0.0f;
        Vec3f total_momentum = Vec3f(0.0f); ///< Linear momentum (m*v)
        Vec3f total_angular_momentum = Vec3f(0.0f); ///< Angular momentum (I*ω)
        size_t body_count = 0;
    };

    /// Get current diagnostics
    const Diagnostics &get_diagnostics() const
    {
        return diagnostics_;
    }

private:
    // ========================================================================
    // Internal Helpers
    // ========================================================================

    static collision::AABB compute_body_aabb(const RigidBody &body)
    {
        if (!body.shape)
        {
            return collision::AABB::from_sphere(body.position, body.collision_radius);
        }

        if (body.shape->shape_type == ShapeType::Box)
        {
            const auto *box = dynamic_cast<const BoxShape *>(body.shape.get());
            if (box)
            {
                Vec3f center = body.position + box->local_center;
                return collision::AABB(center - box->half_extents, center + box->half_extents);
            }
        }

        if (body.shape->shape_type == ShapeType::Sphere)
        {
            const auto *sphere = dynamic_cast<const SphereShape *>(body.shape.get());
            if (sphere)
            {
                Vec3f center = body.position + sphere->local_center;
                return collision::AABB::from_sphere(center, sphere->radius);
            }
        }

        return collision::AABB::from_sphere(body.position, body.collision_radius);
    }

    static void apply_contact_impulse(RigidBody &body_a, RigidBody &body_b, const collision::ContactPoint &contact)
    {
        float inv_m1 = body_a.inv_mass;
        float inv_m2 = body_b.inv_mass;
        float inv_sum = inv_m1 + inv_m2;

        if (inv_sum <= 0.0f)
        {
            return;
        }

        Vec3f r_a = contact.position - body_a.position;
        Vec3f r_b = contact.position - body_b.position;

        Vec3f velocity_a_at_contact = body_a.velocity + body_a.angular_velocity.cross(r_a);
        Vec3f velocity_b_at_contact = body_b.velocity + body_b.angular_velocity.cross(r_b);
        Vec3f relative_velocity = velocity_b_at_contact - velocity_a_at_contact;
        float rel_normal = relative_velocity.dot(contact.normal);

        if (rel_normal >= 0.0f)
        {
            return;
        }

        Vec3f ra_cross_n = r_a.cross(contact.normal);
        Vec3f rb_cross_n = r_b.cross(contact.normal);

        float angular_factor_a = (body_a.inertia_tensor_inv * ra_cross_n).cross(r_a).dot(contact.normal);
        float angular_factor_b = (body_b.inertia_tensor_inv * rb_cross_n).cross(r_b).dot(contact.normal);

        float impulse_denominator = inv_sum + angular_factor_a + angular_factor_b;
        if (impulse_denominator <= 1e-8f)
        {
            return;
        }

        float restitution = std::min(body_a.material.restitution, body_b.material.restitution);
        float impulse_mag = -(1.0f + restitution) * rel_normal / impulse_denominator;
        Vec3f impulse = contact.normal * impulse_mag;

        body_a.velocity -= impulse * inv_m1;
        body_b.velocity += impulse * inv_m2;

        if (!body_a.is_static())
        {
            body_a.angular_velocity -= body_a.inertia_tensor_inv * r_a.cross(impulse);
        }
        if (!body_b.is_static())
        {
            body_b.angular_velocity += body_b.inertia_tensor_inv * r_b.cross(impulse);
        }

        Vec3f post_velocity_a = body_a.velocity + body_a.angular_velocity.cross(r_a);
        Vec3f post_velocity_b = body_b.velocity + body_b.angular_velocity.cross(r_b);
        Vec3f post_relative_velocity = post_velocity_b - post_velocity_a;

        Vec3f tangent_velocity = post_relative_velocity - contact.normal * post_relative_velocity.dot(contact.normal);
        float tangent_speed = tangent_velocity.length();

        if (tangent_speed > 1e-6f)
        {
            Vec3f tangent = tangent_velocity / tangent_speed;

            Vec3f ra_cross_t = r_a.cross(tangent);
            Vec3f rb_cross_t = r_b.cross(tangent);

            float tangent_factor_a = (body_a.inertia_tensor_inv * ra_cross_t).cross(r_a).dot(tangent);
            float tangent_factor_b = (body_b.inertia_tensor_inv * rb_cross_t).cross(r_b).dot(tangent);

            float tangent_denominator = inv_sum + tangent_factor_a + tangent_factor_b;
            if (tangent_denominator > 1e-8f)
            {
                float jt = -post_relative_velocity.dot(tangent) / tangent_denominator;
                float mu = std::min(body_a.material.friction, body_b.material.friction);
                float max_friction_impulse = mu * impulse_mag;
                jt = std::clamp(jt, -max_friction_impulse, max_friction_impulse);

                Vec3f friction_impulse = tangent * jt;

                body_a.velocity -= friction_impulse * inv_m1;
                body_b.velocity += friction_impulse * inv_m2;

                if (!body_a.is_static())
                {
                    body_a.angular_velocity -= body_a.inertia_tensor_inv * r_a.cross(friction_impulse);
                }
                if (!body_b.is_static())
                {
                    body_b.angular_velocity += body_b.inertia_tensor_inv * r_b.cross(friction_impulse);
                }
            }
        }

        if (contact.penetration > 0.0f)
        {
            float corrected_penetration = std::max(0.0f, contact.penetration - 1e-4f);
            Vec3f correction = contact.normal * (corrected_penetration / inv_sum);
            body_a.position -= correction * inv_m1;
            body_b.position += correction * inv_m2;
        }
    }

    static Vec3f get_shape_local_center(const RigidBody &body)
    {
        return body.shape ? body.shape->local_center : Vec3f(0.0f);
    }

    static float get_shape_sweep_radius(const RigidBody &body)
    {
        if (!body.shape)
        {
            return body.collision_radius;
        }

        if (body.shape->shape_type == ShapeType::Sphere)
        {
            const auto *sphere = dynamic_cast<const SphereShape *>(body.shape.get());
            if (sphere)
            {
                return sphere->radius;
            }
        }

        return body.collision_radius;
    }

    float get_rotationally_expanded_sweep_radius(const RigidBody &body, float dt) const
    {
        float base_radius = get_shape_sweep_radius(body);
        if (!config_.ccd_config.enable_rotational_ccd || dt <= 0.0f)
        {
            return base_radius;
        }

        float angular_speed = body.angular_velocity.length();
        float angular_sweep = angular_speed * base_radius * dt;
        return base_radius + angular_sweep;
    }

    void resolve_collisions(const std::vector<Vec3f> &frame_start_positions, float dt)
    {
        if (bodies_.size() < 2)
        {
            return;
        }

        for (size_t i = 0; i + 1 < bodies_.size(); ++i)
        {
            for (size_t j = i + 1; j < bodies_.size(); ++j)
            {
                RigidBody &body_a = bodies_[i];
                RigidBody &body_b = bodies_[j];

                if (body_a.is_static() && body_b.is_static())
                {
                    continue;
                }

                collision::AABB aabb_a = compute_body_aabb(body_a);
                collision::AABB aabb_b = compute_body_aabb(body_b);
                bool end_overlap = aabb_a.overlaps(aabb_b);

                collision::ContactManifold manifold;

                bool sphere_sphere = body_a.shape && body_b.shape && body_a.shape->shape_type == ShapeType::Sphere &&
                                     body_b.shape->shape_type == ShapeType::Sphere;

                if (end_overlap && sphere_sphere)
                {
                    const auto *sphere_a = dynamic_cast<const SphereShape *>(body_a.shape.get());
                    const auto *sphere_b = dynamic_cast<const SphereShape *>(body_b.shape.get());

                    if (!sphere_a || !sphere_b)
                    {
                        continue;
                    }

                    collision::SphereCollider collider_a{body_a.position + sphere_a->local_center,
                                                         body_a.velocity,
                                                         body_a.inv_mass,
                                                         body_a.material.restitution,
                                                         sphere_a->radius};

                    collision::SphereCollider collider_b{body_b.position + sphere_b->local_center,
                                                         body_b.velocity,
                                                         body_b.inv_mass,
                                                         body_b.material.restitution,
                                                         sphere_b->radius};

                    manifold = collision::SphereSpherNarrowphase::detect(collider_a, collider_b, i, j);
                }
                else if (end_overlap)
                {
                    manifold = collision::AABBNarrowphase::detect(
                        aabb_a, aabb_b, body_a.position, body_b.position, body_a.velocity, body_b.velocity, i, j);
                }

                if (!manifold.is_valid() && config_.enable_linear_ccd && dt > 1e-8f)
                {
                    // Optimization: Early bailout if both bodies are slow-moving
                    // (CCD checks would be wasted as discrete detection is sufficient)
                    // Note: Must account for both linear AND angular velocity when rotational CCD enabled
                    float speed_a = body_a.velocity.length();
                    float speed_b = body_b.velocity.length();
                    float angular_speed_a = body_a.angular_velocity.length();
                    float angular_speed_b = body_b.angular_velocity.length();
                    float max_linear_speed = std::max(speed_a, speed_b);
                    float max_angular_speed = std::max(angular_speed_a, angular_speed_b);

                    // Skip CCD if below velocity threshold AND no significant rotation
                    bool linear_slow = max_linear_speed < config_.ccd_config.velocity_threshold;
                    bool rotation_slow = !config_.ccd_config.enable_rotational_ccd || max_angular_speed < 1.0f;

                    if (linear_slow && rotation_slow)
                    {
                        continue; // No CCD needed for this pair
                    }

                    Vec3f start_center_a = frame_start_positions[i] + get_shape_local_center(body_a);
                    Vec3f start_center_b = frame_start_positions[j] + get_shape_local_center(body_b);

                    float sweep_radius_a = get_rotationally_expanded_sweep_radius(body_a, dt);
                    float sweep_radius_b = get_rotationally_expanded_sweep_radius(body_b, dt);

                    collision::SphereCollider ccd_collider_a{
                        start_center_a, body_a.velocity, body_a.inv_mass, body_a.material.restitution, sweep_radius_a};

                    collision::SphereCollider ccd_collider_b{
                        start_center_b, body_b.velocity, body_b.inv_mass, body_b.material.restitution, sweep_radius_b};

                    manifold = collision::SphereSpherNarrowphase::detect_with_ccd(
                        ccd_collider_a, ccd_collider_b, i, j, dt, config_.ccd_config);

                    if (!manifold.is_valid() && config_.ccd_config.enable_rotational_ccd && sphere_sphere)
                    {
                        collision::ccd::SweptSphereSolver::Sphere sphere_a{
                            start_center_a, body_a.velocity, sweep_radius_a};

                        collision::ccd::SweptSphereSolver::Sphere sphere_b{
                            start_center_b, body_b.velocity, sweep_radius_b};

                        collision::ccd::TimeOfImpactResult toi =
                            collision::ccd::ConservativeAdvancement::solve_sphere_sphere(sphere_a, sphere_b, dt);

                        if (toi.collision_occurs)
                        {
                            manifold.object_a_id = i;
                            manifold.object_b_id = j;
                            manifold.toi = toi.toi;
                            manifold.contact.position = toi.contact_point;
                            manifold.contact.normal = toi.contact_normal;
                            manifold.contact.penetration = 0.0f;
                            manifold.contact.relative_velocity_along_normal =
                                (body_b.velocity - body_a.velocity).dot(toi.contact_normal);
                        }
                    }

                    if (!manifold.is_valid() && !sphere_sphere && config_.ccd_config.enable_rotational_ccd)
                    {
                        collision::ccd::SweptSphereSolver::Sphere sphere_a{
                            start_center_a, body_a.velocity, sweep_radius_a};

                        collision::ccd::SweptSphereSolver::Sphere sphere_b{
                            start_center_b, body_b.velocity, sweep_radius_b};

                        collision::ccd::TimeOfImpactResult toi =
                            collision::ccd::SweptSphereSolver::solve_static(sphere_a, sphere_b, dt);

                        if (toi.collision_occurs)
                        {
                            manifold.object_a_id = i;
                            manifold.object_b_id = j;
                            manifold.toi = toi.toi;
                            manifold.contact.position = toi.contact_point;
                            manifold.contact.normal = toi.contact_normal;
                            manifold.contact.penetration = 0.0f;
                            manifold.contact.relative_velocity_along_normal =
                                (body_b.velocity - body_a.velocity).dot(toi.contact_normal);
                        }
                    }

                    if (!manifold.is_valid() && !sphere_sphere)
                    {
                        float expansion_a = 0.0f;
                        float expansion_b = 0.0f;
                        if (config_.ccd_config.enable_rotational_ccd)
                        {
                            float radius_a = body_a.shape ? body_a.shape->get_bounding_radius() : 0.0f;
                            float radius_b = body_b.shape ? body_b.shape->get_bounding_radius() : 0.0f;
                            expansion_a = body_a.angular_velocity.length() * radius_a * dt;
                            expansion_b = body_b.angular_velocity.length() * radius_b * dt;
                        }

                        collision::ccd::ConvexSweepSolver::MovingConvex convex_a{
                            body_a.shape.get(), frame_start_positions[i], body_a.velocity, expansion_a};

                        collision::ccd::ConvexSweepSolver::MovingConvex convex_b{
                            body_b.shape.get(), frame_start_positions[j], body_b.velocity, expansion_b};

                        collision::ccd::TimeOfImpactResult toi =
                            collision::ccd::ConvexSweepSolver::solve(convex_a,
                                                                     convex_b,
                                                                     dt,
                                                                     std::max(1, config_.ccd_config.max_substeps),
                                                                     config_.ccd_config.min_toi_separation);

                        if (toi.collision_occurs)
                        {
                            manifold.object_a_id = i;
                            manifold.object_b_id = j;
                            manifold.toi = toi.toi;
                            manifold.contact.position = toi.contact_point;
                            manifold.contact.normal = toi.contact_normal;
                            manifold.contact.penetration = 0.0f;
                            manifold.contact.relative_velocity_along_normal =
                                (body_b.velocity - body_a.velocity).dot(toi.contact_normal);
                        }
                    }

                    if (manifold.is_valid())
                    {
                        float toi = std::clamp(manifold.toi, 0.0f, 1.0f);

                        if (!body_a.is_static())
                        {
                            body_a.position = frame_start_positions[i] + body_a.velocity * (dt * toi);
                        }
                        if (!body_b.is_static())
                        {
                            body_b.position = frame_start_positions[j] + body_b.velocity * (dt * toi);
                        }

                        manifold.contact.penetration = 0.0f;
                        manifold.contact.relative_velocity_along_normal =
                            (body_b.velocity - body_a.velocity).dot(manifold.contact.normal);
                    }
                }

                if (!manifold.is_valid() && config_.enable_linear_ccd && config_.ccd_config.use_speculative_contacts &&
                    config_.ccd_config.speculative_distance > 0.0f)
                {
                    Vec3f center_a = body_a.position + get_shape_local_center(body_a);
                    Vec3f center_b = body_b.position + get_shape_local_center(body_b);

                    Vec3f delta = center_b - center_a;
                    float distance = delta.length();
                    float combined_radius = get_shape_sweep_radius(body_a) + get_shape_sweep_radius(body_b);
                    float gap = distance - combined_radius;

                    if (gap >= 0.0f && gap <= config_.ccd_config.speculative_distance)
                    {
                        Vec3f normal = (distance > 1e-6f) ? (delta / distance) : Vec3f(1.0f, 0.0f, 0.0f);
                        float rel_normal = (body_b.velocity - body_a.velocity).dot(normal);

                        if (rel_normal < 0.0f)
                        {
                            manifold.object_a_id = i;
                            manifold.object_b_id = j;
                            manifold.contact.position = center_a + normal * get_shape_sweep_radius(body_a);
                            manifold.contact.normal = normal;
                            manifold.contact.penetration = 0.0f;
                            manifold.contact.relative_velocity_along_normal = rel_normal;
                        }
                    }
                }

                if (!manifold.is_valid())
                {
                    continue;
                }

                apply_contact_impulse(body_a, body_b, manifold.contact);
            }
        }
    }

    /// Recompute system-wide diagnostics
    void compute_diagnostics()
    {
        diagnostics_ = Diagnostics{};
        diagnostics_.body_count = bodies_.size();

        for (const auto &rb : bodies_)
        {
            // Linear kinetic energy
            float linear_ke = 0.5f * rb.get_mass() * rb.velocity.squaredLength();
            diagnostics_.total_linear_ke += linear_ke;

            // Angular kinetic energy
            float angular_ke =
                phynity::physics::inertia::compute_angular_kinetic_energy(rb.inertia_tensor, rb.angular_velocity);
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

} // namespace phynity::physics

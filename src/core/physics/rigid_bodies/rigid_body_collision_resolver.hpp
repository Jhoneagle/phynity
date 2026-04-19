#pragma once

#include <core/diagnostics/profiling_macros.hpp>
#include <core/math/quaternions/quat_conversions.hpp>
#include <core/physics/collision/broadphase/spatial_grid.hpp>
#include <core/physics/collision/ccd/conservative_advancement.hpp>
#include <core/physics/collision/ccd/convex_sweep.hpp>
#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/collision/narrowphase/aabb_narrowphase.hpp>
#include <core/physics/collision/narrowphase/sphere_sphere_narrowphase.hpp>
#include <core/physics/shapes/aabb.hpp>
#include <core/physics/collision/collision_proxy.hpp>
#include <core/physics/common/ccd_config.hpp>
#include <core/physics/rigid_bodies/rigid_body.hpp>

#include <algorithm>
#include <cmath>
#include <span>
#include <unordered_set>
#include <vector>

namespace phynity::physics
{

using phynity::math::matrices::Mat3f;
using phynity::math::vectors::Vec3f;
using phynity::physics::collision::SpatialGrid;
using phynity::physics::shapes::BoxShape;
using phynity::physics::shapes::SphereShape;

/// Configuration for the rigid body collision resolver.
/// Extracted from RigidBodySystem::Config to avoid circular dependency.
struct RigidBodyCollisionConfig
{
    float default_collision_radius = 1.0f;
    bool enable_linear_ccd = true;
    CCDConfig ccd_config = ccd_presets::balanced();
};

/// Handles broadphase, narrowphase, CCD, and impulse resolution for rigid bodies.
/// Extracted from RigidBodySystem to reduce class complexity.
class RigidBodyCollisionResolver
{
public:
    explicit RigidBodyCollisionResolver(float cell_size) : spatial_grid_(cell_size)
    {
    }

    /// Run the full collision detection and resolution pipeline.
    /// @param bodies Mutable reference to all rigid bodies (positions/velocities modified)
    /// @param frame_start_positions Body positions at the start of the frame (for CCD)
    /// @param dt Timestep in seconds
    /// @param config Collision configuration parameters
    void resolve(std::vector<RigidBody> &bodies,
                 std::span<const Vec3f> frame_start_positions,
                 float dt,
                 const RigidBodyCollisionConfig &config)
    {
        if (bodies.size() < 2)
        {
            return;
        }

        // Phase 1: Build broadphase spatial grid
        {
            PROFILE_SCOPE("RigidBodySystem::broadphase_build");
            spatial_grid_.clear();
            for (size_t i = 0; i < bodies.size(); ++i)
            {
                if (bodies[i].active)
                {
                    spatial_grid_.insert(static_cast<uint32_t>(i), bodies[i].position);
                }
            }
        }

        // Phase 2: Query broadphase for candidate pairs and run narrowphase
        std::unordered_set<uint64_t> processed_pairs;
        processed_pairs.reserve(bodies.size() * 2);

        for (size_t i = 0; i < bodies.size(); ++i)
        {
            if (!bodies[i].active)
            {
                continue;
            }

            const auto candidates = spatial_grid_.get_neighbor_objects_tracked(bodies[i].position);

            for (uint32_t j_index : candidates)
            {
                const auto j = static_cast<size_t>(j_index);

                if (i >= j)
                {
                    continue;
                }

                const uint64_t pair_id = (static_cast<uint64_t>(i) << 32) | static_cast<uint32_t>(j);
                if (processed_pairs.count(pair_id) > 0)
                {
                    continue;
                }
                processed_pairs.insert(pair_id);

                RigidBody &body_a = bodies[i];
                RigidBody &body_b = bodies[j];

                if (body_a.is_static() && body_b.is_static())
                {
                    continue;
                }

                collision::AABB aabb_a = compute_body_aabb(body_a);
                collision::AABB aabb_b = compute_body_aabb(body_b);
                bool end_overlap = aabb_a.overlaps(aabb_b);

                collision::ContactManifold manifold;

                bool sphere_sphere = body_a.shape && body_b.shape &&
                                     body_a.shape->get_type() == ShapeType::Sphere &&
                                     body_b.shape->get_type() == ShapeType::Sphere;

                if (end_overlap && sphere_sphere)
                {
                    const auto *sphere_a = static_cast<const SphereShape *>(body_a.shape.get());
                    const auto *sphere_b = static_cast<const SphereShape *>(body_b.shape.get());

                    collision::CollisionProxy collider_a{body_a.position + sphere_a->local_center,
                                                         body_a.velocity,
                                                         body_a.inv_mass,
                                                         body_a.material.restitution,
                                                         sphere_a->radius};

                    collision::CollisionProxy collider_b{body_b.position + sphere_b->local_center,
                                                         body_b.velocity,
                                                         body_b.inv_mass,
                                                         body_b.material.restitution,
                                                         sphere_b->radius};

                    manifold = collision::SphereSphereNarrowphase::detect(collider_a, collider_b, i, j);
                }
                else if (end_overlap)
                {
                    manifold = collision::AABBNarrowphase::detect(
                        aabb_a, aabb_b, body_a.position, body_b.position, body_a.velocity, body_b.velocity, i, j);
                }

                if (!manifold.is_valid() && config.enable_linear_ccd && dt > 1e-8f)
                {
                    float speed_a = body_a.velocity.length();
                    float speed_b = body_b.velocity.length();
                    float angular_speed_a = body_a.angular_velocity.length();
                    float angular_speed_b = body_b.angular_velocity.length();
                    float max_linear_speed = std::max(speed_a, speed_b);
                    float max_angular_speed = std::max(angular_speed_a, angular_speed_b);

                    bool linear_slow = max_linear_speed < config.ccd_config.velocity_threshold;
                    bool rotation_slow = !config.ccd_config.enable_rotational_ccd || max_angular_speed < 1.0f;

                    if (linear_slow && rotation_slow)
                    {
                        continue;
                    }

                    Vec3f start_center_a = frame_start_positions[i] + get_shape_local_center(body_a);
                    Vec3f start_center_b = frame_start_positions[j] + get_shape_local_center(body_b);

                    float sweep_radius_a = get_rotationally_expanded_sweep_radius(body_a, dt, config.ccd_config);
                    float sweep_radius_b = get_rotationally_expanded_sweep_radius(body_b, dt, config.ccd_config);

                    collision::CollisionProxy ccd_collider_a{
                        start_center_a, body_a.velocity, body_a.inv_mass, body_a.material.restitution, sweep_radius_a};

                    collision::CollisionProxy ccd_collider_b{
                        start_center_b, body_b.velocity, body_b.inv_mass, body_b.material.restitution, sweep_radius_b};

                    manifold = collision::SphereSphereNarrowphase::detect_with_ccd(
                        ccd_collider_a, ccd_collider_b, i, j, dt, config.ccd_config);

                    if (!manifold.is_valid() && config.ccd_config.enable_rotational_ccd && sphere_sphere)
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

                    if (!manifold.is_valid() && !sphere_sphere && config.ccd_config.enable_rotational_ccd)
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
                        if (config.ccd_config.enable_rotational_ccd)
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
                                                                     std::max(1, config.ccd_config.max_substeps),
                                                                     config.ccd_config.min_toi_separation);

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

                if (!manifold.is_valid() && config.enable_linear_ccd && config.ccd_config.use_speculative_contacts &&
                    config.ccd_config.speculative_distance > 0.0f)
                {
                    Vec3f center_a = body_a.position + get_shape_local_center(body_a);
                    Vec3f center_b = body_b.position + get_shape_local_center(body_b);

                    Vec3f delta = center_b - center_a;
                    float distance = delta.length();
                    float combined_radius = get_shape_sweep_radius(body_a) + get_shape_sweep_radius(body_b);
                    float gap = distance - combined_radius;

                    if (gap >= 0.0f && gap <= config.ccd_config.speculative_distance)
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

private:
    SpatialGrid spatial_grid_;

    static collision::AABB compute_body_aabb(const RigidBody &body)
    {
        if (!body.shape)
        {
            return collision::AABB::from_sphere(body.position, body.collision_radius);
        }

        if (body.shape->get_type() == ShapeType::Box)
        {
            const auto *box = static_cast<const BoxShape *>(body.shape.get());
            Mat3f R = phynity::math::quaternions::toRotationMatrix(body.orientation);

            Vec3f h = box->half_extents;
            Vec3f rotated_half(std::abs(R(0, 0)) * h.x + std::abs(R(0, 1)) * h.y + std::abs(R(0, 2)) * h.z,
                               std::abs(R(1, 0)) * h.x + std::abs(R(1, 1)) * h.y + std::abs(R(1, 2)) * h.z,
                               std::abs(R(2, 0)) * h.x + std::abs(R(2, 1)) * h.y + std::abs(R(2, 2)) * h.z);

            Vec3f center = body.position + R * box->local_center;
            return collision::AABB(center - rotated_half, center + rotated_half);
        }

        if (body.shape->get_type() == ShapeType::Sphere)
        {
            const auto *sphere = static_cast<const SphereShape *>(body.shape.get());
            Vec3f center = body.position + sphere->local_center;
            return collision::AABB::from_sphere(center, sphere->radius);
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
        return body.shape ? body.shape->get_local_center() : Vec3f(0.0f);
    }

    static float get_shape_sweep_radius(const RigidBody &body)
    {
        if (!body.shape)
        {
            return body.collision_radius;
        }

        if (body.shape->get_type() == ShapeType::Sphere)
        {
            const auto *sphere = static_cast<const SphereShape *>(body.shape.get());
            return sphere->radius;
        }

        return body.collision_radius;
    }

    static float get_rotationally_expanded_sweep_radius(const RigidBody &body, float dt, const CCDConfig &ccd_config)
    {
        float base_radius = get_shape_sweep_radius(body);
        if (!ccd_config.enable_rotational_ccd || dt <= 0.0f)
        {
            return base_radius;
        }

        float angular_speed = body.angular_velocity.length();
        float angular_sweep = angular_speed * base_radius * dt;
        return base_radius + angular_sweep;
    }
};

} // namespace phynity::physics

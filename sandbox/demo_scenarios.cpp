#include "demo_scenarios.hpp"

#include <core/physics/config/physics_constants.hpp>
#include <core/physics/constraints/hinge_joint.hpp>
#include <core/physics/shapes/box.hpp>

#include <cmath>
#include <memory>

namespace phynity::app::scenarios
{

using namespace phynity::physics::constants;

namespace
{
using phynity::physics::Material;

Material make_no_damping_material(float mass, float restitution = 0.8f)
{
    return {mass, restitution, 0.3f, 0.0f, 0.0f, 0.0f};
}
} // namespace

void GravityWell::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.0f);

    // Spawn particles at different positions, all falling
    context.spawn_particle(Vec3f(-2.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(2.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
}

void ParticleSpread::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.0f);

    // Spawn particles with various initial velocities
    const float speed = 5.0f;
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(speed, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(-speed, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f, 0.0f, speed), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f, 0.0f, -speed), 1.0f);
}

void DragInteraction::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.1f); // Moderate air resistance

    // Spawn particles with high initial velocity that will slow down
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(10.0f, 5.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(-10.0f, 5.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f, 10.0f, 0.0f), 1.0f);
}

void ProjectileMotion::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.0f);

    // Classic projectile: 45-degree angle for maximum range
    const float speed = 10.0f;
    const float angle_rad = PI / 4.0f; // 45 degrees in radians

    context.spawn_particle(
        Vec3f(0.0f, 0.0f, 0.0f), Vec3f(speed * std::cos(angle_rad), speed * std::sin(angle_rad), 0.0f), 1.0f);

    // Also launch at different angles for comparison
    context.spawn_particle(Vec3f(0.0f, 0.0f, 0.0f),
                           Vec3f(speed * std::cos(angle_rad * 0.5f), speed * std::sin(angle_rad * 0.5f), 0.0f),
                           1.0f);
}

void OrbitStability::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.particle_system().clear_force_fields();

    const float mass = 1.0f;
    const float radius = 5.0f;
    const float spring_constant = 4.0f;
    const float omega = std::sqrt(spring_constant / mass);
    const float tangential_speed = omega * radius;

    context.particle_system().add_force_field(
        std::make_unique<phynity::physics::SpringField>(Vec3f(0.0f, 0.0f, 0.0f), spring_constant));

    context.spawn_particle(
        Vec3f(radius, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, tangential_speed), make_no_damping_material(mass));
}

void MultiParticleCollision::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.particle_system().clear_force_fields();
    context.particle_system().enable_collisions(true);
    context.particle_system().set_default_collision_radius(radius_);

    Material material = make_no_damping_material(1.0f, restitution_);

    context.spawn_particle(Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), material, radius_);
    context.spawn_particle(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-2.0f, 0.0f, 0.0f), material, radius_);
}

void LowGravity::setup(PhysicsContext &context)
{
    context.clear_particles();
    // Moon gravity: approximately 1/6 of Earth
    context.set_gravity(Vec3f(0.0f, -1.62f, 0.0f));
    context.set_drag(0.0f);

    // Spawn particles
    context.spawn_particle(Vec3f(0.0f, 2.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(-1.0f, 2.0f, 0.0f), Vec3f(3.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(1.0f, 2.0f, 0.0f), Vec3f(-3.0f, 0.0f, 0.0f), 1.0f);
}

void ZeroGravity::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, 0.0f, 0.0f));
    context.set_drag(0.0f);

    // Particles move at constant velocity in zero gravity
    context.spawn_particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 5.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 5.0f), 1.0f);
}

void HighDrag::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.5f); // High drag coefficient

    // Spawn particles with initial velocity in a viscous medium
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(20.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(-20.0f, 0.0f, 0.0f), 1.0f);
}

// ============================================================================
// Rigid Body Scenarios
// ============================================================================

void BoxStacking::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.clear_bodies();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.0f);

    auto &system = context.rigid_body_system();

    // Ground (static box)
    auto ground_shape = std::make_shared<phynity::physics::shapes::BoxShape>(Vec3f(10.0f, 0.5f, 10.0f));
    system.spawn_body(Vec3f(0.0f, -1.0f, 0.0f), Quatf(), ground_shape, 0.0f);

    // Tower of boxes
    auto box_shape = std::make_shared<phynity::physics::shapes::BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));
    const int num_boxes = 5;
    const float box_height = 1.1f;

    for (int i = 0; i < num_boxes; ++i)
    {
        float y = 0.5f + static_cast<float>(i) * box_height;
        system.spawn_body(Vec3f(0.0f, y, 0.0f), Quatf(), box_shape, 1.0f);
    }
}

void TowerTopple::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.clear_bodies();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.0f);

    auto &system = context.rigid_body_system();

    // Ground
    auto ground_shape = std::make_shared<phynity::physics::shapes::BoxShape>(Vec3f(10.0f, 0.5f, 10.0f));
    system.spawn_body(Vec3f(0.0f, -1.0f, 0.0f), Quatf(), ground_shape, 0.0f);

    // Tower of boxes
    auto box_shape = std::make_shared<phynity::physics::shapes::BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));
    const int num_boxes = 5;
    const float box_height = 1.1f;

    for (int i = 0; i < num_boxes; ++i)
    {
        float y = 0.5f + static_cast<float>(i) * box_height;
        system.spawn_body(Vec3f(0.0f, y, 0.0f), Quatf(), box_shape, 1.0f);
    }

    elapsed_ = 0.0f;
    impulse_applied_ = false;
}

void TowerTopple::step_callback(PhysicsContext &context, float dt)
{
    elapsed_ += dt;

    // Apply impulse after 2 seconds of settling
    if (!impulse_applied_ && elapsed_ >= 2.0f)
    {
        auto *top_body = context.rigid_body_system().get_body(5); // Body IDs: 0=ground, 1-5=boxes
        if (top_body != nullptr)
        {
            Vec3f impulse = Vec3f(10.0f, 0.0f, 0.0f);
            top_body->velocity += impulse * top_body->inv_mass;
        }
        impulse_applied_ = true;
    }
}

void HingeDoor::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.clear_bodies();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.0f);

    auto &system = context.rigid_body_system();

    // Frame (static)
    auto frame_shape = std::make_shared<phynity::physics::shapes::BoxShape>(Vec3f(2.0f, 3.0f, 0.1f));
    system.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), frame_shape, 0.0f);

    // Door (dynamic)
    auto door_shape = std::make_shared<phynity::physics::shapes::BoxShape>(Vec3f(0.05f, 3.0f, 1.0f));
    auto door_id = system.spawn_body(Vec3f(1.0f, 0.0f, 0.0f), Quatf(), door_shape, 5.0f);

    auto *frame_body = system.get_body(0);
    auto *door_body = system.get_body(door_id);

    // Hinge constraint at door pivot
    system.add_constraint(
        std::make_unique<phynity::physics::constraints::HingeJoint>(frame_body,
                                                                    door_body,
                                                                    Vec3f(0.0f, 1.5f, 0.0f), // Pivot on frame
                                                                    Vec3f(-0.05f, 1.5f, 0.0f), // Pivot on door
                                                                    Vec3f(0.0f, 1.0f, 0.0f) // Hinge axis (vertical)
                                                                    ));

    // Apply initial spin
    if (door_body != nullptr)
    {
        door_body->angular_velocity = Vec3f(0.0f, 2.0f, 0.0f);
    }
}

} // namespace phynity::app::scenarios

#include "demo_scenarios.hpp"
#include <core/physics/common/physics_constants.hpp>
#include <cmath>

namespace phynity::app::scenarios {

using namespace phynity::physics::constants;

namespace {
using phynity::physics::Material;

Material make_no_damping_material(float mass, float restitution = 0.8f) {
    return Material(mass, restitution, 0.3f, 0.0f, 0.0f, 0.0f);
}
}  // namespace

void GravityWell::setup(PhysicsContext& context) {
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.0f);
    
    // Spawn particles at different positions, all falling
    context.spawn_particle(Vec3f(-2.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(2.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
}

void ParticleSpread::setup(PhysicsContext& context) {
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

void DragInteraction::setup(PhysicsContext& context) {
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.1f);  // Moderate air resistance
    
    // Spawn particles with high initial velocity that will slow down
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(10.0f, 5.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(-10.0f, 5.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f, 10.0f, 0.0f), 1.0f);
}

void ProjectileMotion::setup(PhysicsContext& context) {
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.0f);
    
    // Classic projectile: 45-degree angle for maximum range
    const float speed = 10.0f;
    const float angle_rad = PI / 4.0f;  // 45 degrees in radians
    
    context.spawn_particle(
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(speed * std::cos(angle_rad), speed * std::sin(angle_rad), 0.0f),
        1.0f
    );
    
    // Also launch at different angles for comparison
    context.spawn_particle(
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(speed * std::cos(angle_rad * 0.5f), speed * std::sin(angle_rad * 0.5f), 0.0f),
        1.0f
    );
}

void OrbitStability::setup(PhysicsContext& context) {
    context.clear_particles();
    context.particle_system().clear_force_fields();

    const float mass = 1.0f;
    const float radius = 5.0f;
    const float spring_constant = 4.0f;
    const float omega = std::sqrt(spring_constant / mass);
    const float tangential_speed = omega * radius;

    context.particle_system().add_force_field(
        std::make_unique<phynity::physics::SpringField>(Vec3f(0.0f, 0.0f, 0.0f), spring_constant)
    );

    context.spawn_particle(
        Vec3f(radius, 0.0f, 0.0f),
        Vec3f(0.0f, 0.0f, tangential_speed),
        make_no_damping_material(mass)
    );
}

void MultiParticleCollision::setup(PhysicsContext& context) {
    context.clear_particles();
    context.particle_system().clear_force_fields();
    context.particle_system().enable_collisions(true);
    context.particle_system().set_default_collision_radius(radius_);

    Material material = make_no_damping_material(1.0f, restitution_);

    context.spawn_particle(
        Vec3f(-1.0f, 0.0f, 0.0f),
        Vec3f(2.0f, 0.0f, 0.0f),
        material,
        radius_
    );
    context.spawn_particle(
        Vec3f(1.0f, 0.0f, 0.0f),
        Vec3f(-2.0f, 0.0f, 0.0f),
        material,
        radius_
    );
}

void LowGravity::setup(PhysicsContext& context) {
    context.clear_particles();
    // Moon gravity: approximately 1/6 of Earth
    context.set_gravity(Vec3f(0.0f, -1.62f, 0.0f));
    context.set_drag(0.0f);
    
    // Spawn particles
    context.spawn_particle(Vec3f(0.0f, 2.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(-1.0f, 2.0f, 0.0f), Vec3f(3.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(1.0f, 2.0f, 0.0f), Vec3f(-3.0f, 0.0f, 0.0f), 1.0f);
}

void ZeroGravity::setup(PhysicsContext& context) {
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, 0.0f, 0.0f));
    context.set_drag(0.0f);
    
    // Particles move at constant velocity in zero gravity
    context.spawn_particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 5.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 5.0f), 1.0f);
}

void HighDrag::setup(PhysicsContext& context) {
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));
    context.set_drag(0.5f);  // High drag coefficient
    
    // Spawn particles with initial velocity in a viscous medium
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(20.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(-20.0f, 0.0f, 0.0f), 1.0f);
}

}  // namespace phynity::app::scenarios

#include "demo_scenarios.hpp"
#include <cmath>

namespace phynity::app::scenarios {

void GravityWell::setup(PhysicsContext& context) {
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    context.set_drag(0.0f);
    
    // Spawn particles at different positions, all falling
    context.spawn_particle(Vec3f(-2.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(2.0f, 10.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
}

void ParticleSpread::setup(PhysicsContext& context) {
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -9.81f, 0.0f));
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
    context.set_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    context.set_drag(0.1f);  // Moderate air resistance
    
    // Spawn particles with high initial velocity that will slow down
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(10.0f, 5.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(-10.0f, 5.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f, 10.0f, 0.0f), 1.0f);
}

void ProjectileMotion::setup(PhysicsContext& context) {
    context.clear_particles();
    context.set_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    context.set_drag(0.0f);
    
    // Classic projectile: 45-degree angle for maximum range
    const float speed = 10.0f;
    const float angle_rad = 3.14159265f / 4.0f;  // 45 degrees in radians
    
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
    context.set_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    context.set_drag(0.5f);  // High drag coefficient
    
    // Spawn particles with initial velocity in a viscous medium
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(20.0f, 0.0f, 0.0f), 1.0f);
    context.spawn_particle(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(-20.0f, 0.0f, 0.0f), 1.0f);
}

}  // namespace phynity::app::scenarios

#include "demo_scenarios.hpp"

#include <core/physics/config/physics_constants.hpp>
#include <core/physics/constraints/hinge_joint.hpp>
#include <core/physics/fluids/sph_fluid_system.hpp>
#include <core/physics/shapes/aabb.hpp>
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

void WindTunnel::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.particle_system().clear_force_fields();

    // A bounded wind volume blowing in +x through a horizontal corridor.
    const phynity::physics::shapes::AABB region(Vec3f(-6.0f, -2.0f, -2.0f), Vec3f(6.0f, 2.0f, 2.0f));
    context.particle_system().add_force_field(
        std::make_unique<phynity::physics::WindField>(Vec3f(8.0f, 0.0f, 0.0f), 0.5f, region));

    // Particles start at rest at the upwind edge of the corridor.
    context.spawn_particle(Vec3f(-5.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), make_no_damping_material(1.0f));
    context.spawn_particle(Vec3f(-5.0f, 1.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), make_no_damping_material(1.0f));
    context.spawn_particle(Vec3f(-5.0f, -1.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), make_no_damping_material(1.0f));
}

void FloatingObjects::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.particle_system().clear_force_fields();

    const Vec3f gravity(0.0f, -EARTH_GRAVITY, 0.0f);
    const float surface_height = 0.0f;

    // Publish gravity as the system's ambient "down"; BuoyancyField reads it from
    // the shared ForceContext rather than holding its own copy.
    context.particle_system().set_ambient_gravity(gravity);

    // Gravity pulls down, buoyancy (light objects in water) pushes up, and drag
    // dissipates the bobbing so the particles settle at the surface.
    context.particle_system().add_force_field(std::make_unique<phynity::physics::GravityField>(gravity));
    context.particle_system().add_force_field(
        std::make_unique<phynity::physics::BuoyancyField>(WATER_DENSITY, 500.0f, surface_height));
    context.particle_system().add_force_field(std::make_unique<phynity::physics::DragField>(1.5f));

    // Particles start submerged at various depths and rise to the surface.
    context.spawn_particle(Vec3f(-2.0f, -4.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), make_no_damping_material(1.0f));
    context.spawn_particle(Vec3f(0.0f, -6.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), make_no_damping_material(1.0f));
    context.spawn_particle(Vec3f(2.0f, -2.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), make_no_damping_material(1.0f));
}

void DamBreak::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.clear_bodies();

    using phynity::physics::fluids::mass_for_spacing;
    using phynity::physics::fluids::SphParameters;
    using phynity::physics::shapes::AABB;

    auto &fluid = context.sph_fluid_system();
    fluid.clear();

    const float spacing = 0.05f;

    SphParameters params;
    params.smoothing_radius = 0.1f; // h = 2·spacing
    params.rest_density = WATER_DENSITY;
    params.stiffness = 100.0f; // weakly compressible, CFL-friendly
    params.viscosity = 0.05f; // small: explicit viscosity is stiff
    params.clamp_negative_pressure = true; // suppress free-surface tensile instability
    params.boundary_restitution = 0.0f; // fully damped container walls
    params.particle_mass = mass_for_spacing(WATER_DENSITY, spacing);
    params.bounds = AABB(Vec3f(-0.5f), Vec3f(0.5f));
    fluid.set_parameters(params);
    fluid.set_ambient_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));

    // A tall column held against the left wall; released at t=0 it collapses.
    // Seeded on a lattice with mass = ρ₀·spacing³ so it starts at rest density.
    const int nx = 5;
    const int ny = 14;
    const int nz = 5;
    for (int ix = 0; ix < nx; ++ix)
    {
        for (int iy = 0; iy < ny; ++iy)
        {
            for (int iz = 0; iz < nz; ++iz)
            {
                fluid.spawn(Vec3f(-0.48f + static_cast<float>(ix) * spacing,
                                  -0.49f + static_cast<float>(iy) * spacing,
                                  -0.12f + static_cast<float>(iz) * spacing));
            }
        }
    }
}

void DamBreakPbf::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.clear_bodies();

    using phynity::physics::fluids::mass_for_spacing;
    using phynity::physics::fluids::PbfParameters;
    using phynity::physics::shapes::AABB;

    auto &fluid = context.pbf_fluid_system();
    fluid.clear();

    const float spacing = 0.05f;

    PbfParameters params;
    params.sph.smoothing_radius = 0.1f; // h = 2·spacing
    params.sph.rest_density = WATER_DENSITY;
    params.sph.particle_mass = mass_for_spacing(WATER_DENSITY, spacing);
    params.sph.bounds = AABB(Vec3f(-0.5f), Vec3f(0.5f));
    params.solver_iterations = 10;
    params.relaxation = 1.0e-4f;
    params.clamp_density_deficiency = true; // compression-only ⇒ no free-surface collapse
    params.xsph_c = 0.02f; // mild velocity smoothing
    fluid.set_parameters(params);
    fluid.set_ambient_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));

    // A tall column held against the left wall; released at t=0 it collapses.
    const int nx = 5;
    const int ny = 14;
    const int nz = 5;
    for (int ix = 0; ix < nx; ++ix)
    {
        for (int iy = 0; iy < ny; ++iy)
        {
            for (int iz = 0; iz < nz; ++iz)
            {
                fluid.spawn(Vec3f(-0.48f + static_cast<float>(ix) * spacing,
                                  -0.49f + static_cast<float>(iy) * spacing,
                                  -0.12f + static_cast<float>(iz) * spacing));
            }
        }
    }
}

// ============================================================================
// Electromagnetism Scenarios
// ============================================================================

void CyclotronDemo::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.particle_system().clear_force_fields();

    // No gravity: isolate the Lorentz force so the orbits stay clean.
    context.particle_system().set_ambient_gravity(Vec3f(0.0f));

    // Uniform magnetic field along +z; motion is in the xy-plane.
    const Vec3f b_field(0.0f, 0.0f, 1.0f);
    context.particle_system().add_force_field(std::make_unique<phynity::physics::MagneticField>(b_field));

    // A few charges launched perpendicular to B trace circular orbits of radius
    // r = m|v| / (|q||B|); heavier / faster particles trace wider circles.
    struct Launch
    {
        Vec3f position;
        Vec3f velocity;
        float mass;
        float charge;
    };
    const Launch launches[] = {
        {Vec3f(0.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), 1.0f, 1.0f},
        {Vec3f(-3.0f, 0.0f, 0.0f), Vec3f(3.0f, 0.0f, 0.0f), 1.0f, 1.0f},
        {Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f, 2.0f, 0.0f), 2.0f, -1.0f},
    };
    for (const Launch &l : launches)
    {
        Material mat = make_no_damping_material(l.mass);
        mat.charge = l.charge;
        context.spawn_particle(l.position, l.velocity, mat);
    }
}

void ChargedCloud::setup(PhysicsContext &context)
{
    context.clear_particles();
    context.particle_system().clear_force_fields();
    context.particle_system().set_ambient_gravity(Vec3f(0.0f));

    // Enable the mutual particle-particle Coulomb pass (direct O(N²)).
    context.particle_system().enable_coulomb(true);
    context.particle_system().set_coulomb_params(2.0f, 0.1f);

    // Alternating positive/negative charges on a small lattice, seeded at rest;
    // like charges repel, opposite attract, and the cloud self-organizes.
    const int nx = 4;
    const int ny = 4;
    const float spacing = 1.0f;
    for (int ix = 0; ix < nx; ++ix)
    {
        for (int iy = 0; iy < ny; ++iy)
        {
            Material mat = make_no_damping_material(1.0f);
            mat.charge = ((ix + iy) % 2 == 0) ? 1.0f : -1.0f;
            const Vec3f pos(static_cast<float>(ix) * spacing - 1.5f, static_cast<float>(iy) * spacing - 1.5f, 0.0f);
            context.spawn_particle(pos, Vec3f(0.0f), mat);
        }
    }
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

#include "physics_context.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

namespace phynity::app {

PhysicsContext::PhysicsContext()
    : PhysicsContext(Config())
{
}

PhysicsContext::PhysicsContext(const Config& config)
    : config_(config),
      particle_system_(),
      timestep_controller_(
          1.0f / config.target_fps,
          config.max_timestep,
          config.use_determinism 
              ? TimestepController::OverflowMode::SUBDIVIDE 
              : TimestepController::OverflowMode::CLAMP
      )
{
    if (config_.enable_jobs) {
        JobSystemConfig job_config;
        job_config.worker_count = config_.job_workers;
        job_config.mode = config_.use_determinism
            ? SchedulingMode::Deterministic
            : SchedulingMode::Concurrent;
        job_system_.start(job_config);
        particle_system_.set_job_system(&job_system_);
    }
    initialize_force_fields();
}

PhysicsContext::~PhysicsContext() {
    job_system_.shutdown();
}

void PhysicsContext::initialize_force_fields() {
    particle_system_.clear_force_fields();
    
    // Add gravity field
    particle_system_.add_force_field(
        std::make_unique<GravityField>(config_.gravity)
    );
    
    // Add drag field if configured
    if (config_.air_drag > 0.0f) {
        particle_system_.add_force_field(
            std::make_unique<DragField>(config_.air_drag)
        );
    }
}

void PhysicsContext::update(float delta_time) {
    timestep_controller_.accumulate(delta_time);
    
    float dt = 0.0f;
    while ((dt = timestep_controller_.step()) > 0.0f) {
        particle_system_.update(dt);
    }
}

void PhysicsContext::step_deterministic() {
    // Manually perform one physics step
    float dt = 1.0f / config_.target_fps;
    particle_system_.update(dt);
}

void PhysicsContext::reset_accumulator() {
    // Create a new controller to reset accumulator
    float target_ts = 1.0f / config_.target_fps;
    TimestepController::OverflowMode mode = config_.use_determinism 
        ? TimestepController::OverflowMode::SUBDIVIDE 
        : TimestepController::OverflowMode::CLAMP;
    timestep_controller_ = TimestepController(target_ts, config_.max_timestep, mode);
}

void PhysicsContext::spawn_particle(const Vec3f& position, const Vec3f& velocity, float mass, float radius) {
    particle_system_.spawn(position, velocity, mass, -1.0f, radius);
}

void PhysicsContext::spawn_particle(
    const Vec3f& position,
    const Vec3f& velocity,
    const Material& material,
    float radius
) {
    particle_system_.spawn(position, velocity, material, -1.0f, radius);
}

void PhysicsContext::clear_particles() {
    particle_system_.clear();
}

size_t PhysicsContext::particle_count() const {
    return particle_system_.particleCount();
}

void PhysicsContext::set_gravity(const Vec3f& gravity) {
    config_.gravity = gravity;
    initialize_force_fields();
}

void PhysicsContext::set_drag(float drag_coefficient) {
    config_.air_drag = drag_coefficient;
    initialize_force_fields();
}

void PhysicsContext::clear_force_fields() {
    particle_system_.clear_force_fields();
}

size_t PhysicsContext::force_field_count() const {
    return particle_system_.force_field_count();
}

ParticleSystem::Diagnostics PhysicsContext::diagnostics() const {
    return particle_system_.compute_diagnostics();
}

const TimestepController::Statistics& PhysicsContext::timestep_statistics() const {
    return timestep_controller_.statistics();
}

void PhysicsContext::print_diagnostics() const {
    const auto diag = diagnostics();
    const auto ts_stats = timestep_statistics();
    
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Physics Diagnostics ===" << std::endl;
    std::cout << "Particles: " << diag.particle_count << std::endl;
    std::cout << "Kinetic Energy: " << diag.total_kinetic_energy << " J" << std::endl;
    std::cout << "Momentum: [" << diag.total_momentum.x << ", " 
              << diag.total_momentum.y << ", " 
              << diag.total_momentum.z << "]" << std::endl;
    std::cout << "Physics Steps: " << ts_stats.total_steps << std::endl;
    std::cout << "Accumulated Time: " << ts_stats.accumulated_time << "s" << std::endl;
    std::cout << "Max Accumulated: " << ts_stats.max_accumulated_time << "s" << std::endl;
    std::cout << "Overflows: " << ts_stats.overflow_count << std::endl;
    std::cout << "Subdivisions: " << ts_stats.subdivision_count << std::endl;
}

float PhysicsContext::target_timestep() const {
    return 1.0f / config_.target_fps;
}

}  // namespace phynity::app

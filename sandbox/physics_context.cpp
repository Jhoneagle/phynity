#include "physics_context.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace phynity::app
{

PhysicsContext::PhysicsContext() : PhysicsContext(Config())
{
}

PhysicsContext::PhysicsContext(const Config &config)
    : config_(config),
      rigid_body_system_(config.rigid_body_config),
      timestep_controller_(1.0f / config.target_fps,
                           config.max_timestep,
                           config.use_determinism ? TimestepController::OverflowMode::SUBDIVIDE
                                                  : TimestepController::OverflowMode::CLAMP)
{
    if (config_.enable_jobs)
    {
        JobSystemConfig job_config;
        job_config.worker_count = config_.job_workers;

        if (!config_.replay_schedule_path.empty())
        {
            job_config.mode = SchedulingMode::DeterministicReplay;
        }
        else
        {
            job_config.mode = config_.use_determinism ? SchedulingMode::Deterministic : SchedulingMode::Concurrent;
        }

        job_system_.start(job_config);
        particle_system_.set_job_system(&job_system_);
        rigid_body_system_.set_job_system(&job_system_);

        if (config_.record_schedule)
        {
            schedule_recorder_ = std::make_unique<phynity::jobs::ScheduleRecorder>();
        }

        if (!config_.replay_schedule_path.empty())
        {
            schedule_replayer_ = std::make_unique<phynity::jobs::ScheduleReplayer>();
            schedule_replayer_->load(config_.replay_schedule_path);
        }
    }
    initialize_force_fields();
}

PhysicsContext::~PhysicsContext()
{
    save_schedule();
    job_system_.shutdown();
}

void PhysicsContext::save_schedule()
{
    if (schedule_recorder_ && !config_.record_schedule_path.empty())
    {
        schedule_recorder_->save(config_.record_schedule_path);
    }
}

void PhysicsContext::initialize_force_fields()
{
    particle_system_.clear_force_fields();
    rigid_body_system_.clear_force_fields();

    // Add gravity field to both systems
    particle_system_.add_force_field(std::make_unique<GravityField>(config_.gravity));
    rigid_body_system_.add_force_field(std::make_unique<GravityField>(config_.gravity));

    // Add drag field if configured (particle system only)
    if (config_.air_drag > 0.0f)
    {
        particle_system_.add_force_field(std::make_unique<DragField>(config_.air_drag));
    }
}

void PhysicsContext::update(float delta_time)
{
    if (paused_)
    {
        return;
    }

    timestep_controller_.accumulate(delta_time * speed_multiplier_);

    float dt = 0.0f;
    while ((dt = timestep_controller_.step()) > 0.0f)
    {
        particle_system_.update(dt);
        rigid_body_system_.update(dt);
        capture_timeline_frame();
    }
}

void PhysicsContext::step_deterministic()
{
    // Manually perform one physics step
    float dt = 1.0f / config_.target_fps;
    particle_system_.update(dt);
    rigid_body_system_.update(dt);
    capture_timeline_frame();
}

void PhysicsContext::pause()
{
    paused_ = true;
}

void PhysicsContext::resume()
{
    paused_ = false;
}

void PhysicsContext::step_forward()
{
    float dt = 1.0f / config_.target_fps;
    particle_system_.update(dt);
    rigid_body_system_.update(dt);
    capture_timeline_frame();
}

void PhysicsContext::step_backward()
{
    if (timeline_.size() < 2)
    {
        return;
    }

    // Go back one frame: truncate the latest, then restore the new latest
    timeline_.truncate(timeline_.size() - 1);
    const auto *snapshot = timeline_.at(timeline_.size() - 1);
    if (snapshot != nullptr)
    {
        restore_from_snapshot(*snapshot);
    }
}

void PhysicsContext::seek_to_frame(size_t index)
{
    const auto *snapshot = timeline_.at(index);
    if (snapshot == nullptr)
    {
        return;
    }

    restore_from_snapshot(*snapshot);

    // Truncate everything after this frame so step_forward resumes from here
    timeline_.truncate(index + 1);
}

void PhysicsContext::set_speed(float multiplier)
{
    speed_multiplier_ = std::max(0.25f, std::min(4.0f, multiplier));
}

void PhysicsContext::capture_timeline_frame()
{
    using namespace phynity::serialization;

    double sim_time = static_cast<double>(timestep_statistics().accumulated_time);
    PhysicsSnapshot snapshot = SnapshotHelpers::capture_particle_system(particle_system_, frame_counter_, sim_time, 0);

    // Merge rigid body state into the same snapshot
    PhysicsSnapshot rb_snapshot =
        SnapshotHelpers::capture_rigid_body_system(rigid_body_system_, frame_counter_, sim_time, 0);
    snapshot.rigid_bodies = std::move(rb_snapshot.rigid_bodies);

    timeline_.push(std::move(snapshot));
    ++frame_counter_;
}

void PhysicsContext::restore_from_snapshot(const serialization::PhysicsSnapshot &snapshot)
{
    serialization::SnapshotHelpers::restore_particle_system(snapshot, particle_system_);
    serialization::SnapshotHelpers::restore_rigid_body_system(snapshot, rigid_body_system_);
}

void PhysicsContext::reset_accumulator()
{
    // Create a new controller to reset accumulator
    float target_ts = 1.0f / config_.target_fps;
    TimestepController::OverflowMode mode =
        config_.use_determinism ? TimestepController::OverflowMode::SUBDIVIDE : TimestepController::OverflowMode::CLAMP;
    timestep_controller_ = TimestepController(target_ts, config_.max_timestep, mode);
}

void PhysicsContext::spawn_particle(const Vec3f &position, const Vec3f &velocity, float mass, float radius)
{
    particle_system_.spawn(position, velocity, mass, -1.0f, radius);
}

void PhysicsContext::spawn_particle(const Vec3f &position,
                                    const Vec3f &velocity,
                                    const Material &material,
                                    float radius)
{
    particle_system_.spawn(position, velocity, material, -1.0f, radius);
}

void PhysicsContext::clear_particles()
{
    particle_system_.clear();
}

size_t PhysicsContext::particle_count() const
{
    return particle_system_.particleCount();
}

phynity::physics::RigidBodyID PhysicsContext::spawn_body(const Vec3f &position,
                                                         const phynity::math::quaternions::Quatf &orientation,
                                                         std::shared_ptr<phynity::physics::shapes::Shape> shape,
                                                         float mass,
                                                         const Material &material)
{
    return rigid_body_system_.spawn_body(position, orientation, std::move(shape), mass, material);
}

void PhysicsContext::clear_bodies()
{
    rigid_body_system_.clear_bodies();
}

size_t PhysicsContext::body_count() const
{
    return rigid_body_system_.body_count();
}

void PhysicsContext::set_gravity(const Vec3f &gravity)
{
    config_.gravity = gravity;
    initialize_force_fields();
}

void PhysicsContext::set_drag(float drag_coefficient)
{
    config_.air_drag = drag_coefficient;
    initialize_force_fields();
}

void PhysicsContext::clear_force_fields()
{
    particle_system_.clear_force_fields();
    rigid_body_system_.clear_force_fields();
}

size_t PhysicsContext::force_field_count() const
{
    return particle_system_.force_field_count();
}

PhysicsContext::Diagnostics PhysicsContext::diagnostics() const
{
    Diagnostics diag;

    // Particle system diagnostics
    const auto particle_diag = particle_system_.compute_diagnostics();
    diag.particle_count = particle_diag.particle_count;
    diag.total_kinetic_energy += particle_diag.total_kinetic_energy;
    diag.total_momentum += particle_diag.total_momentum;

    // Rigid body system diagnostics
    const auto &rb_diag = rigid_body_system_.get_diagnostics();
    diag.body_count = rb_diag.body_count;
    diag.constraint_count = rigid_body_system_.get_constraints().size();
    diag.total_linear_ke = rb_diag.total_linear_ke;
    diag.total_angular_ke = rb_diag.total_angular_ke;
    diag.total_kinetic_energy += rb_diag.total_kinetic_energy;
    diag.total_momentum += rb_diag.total_momentum;
    diag.total_angular_momentum = rb_diag.total_angular_momentum;

    return diag;
}

const TimestepController::Statistics &PhysicsContext::timestep_statistics() const
{
    return timestep_controller_.statistics();
}

void PhysicsContext::print_diagnostics() const
{
    const auto diag = diagnostics();
    const auto ts_stats = timestep_statistics();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Physics Diagnostics ===\n";
    std::cout << "Particles: " << diag.particle_count << '\n';
    std::cout << "Rigid Bodies: " << diag.body_count << '\n';
    std::cout << "Constraints: " << diag.constraint_count << '\n';
    std::cout << "Kinetic Energy: " << diag.total_kinetic_energy << " J\n";
    if (diag.body_count > 0)
    {
        std::cout << "  Linear KE: " << diag.total_linear_ke << " J\n";
        std::cout << "  Angular KE: " << diag.total_angular_ke << " J\n";
    }
    std::cout << "Momentum: [" << diag.total_momentum.x << ", " << diag.total_momentum.y << ", "
              << diag.total_momentum.z << "]\n";
    if (diag.body_count > 0)
    {
        std::cout << "Angular Momentum: [" << diag.total_angular_momentum.x << ", " << diag.total_angular_momentum.y
                  << ", " << diag.total_angular_momentum.z << "]\n";
    }
    std::cout << "Physics Steps: " << ts_stats.total_steps << '\n';
    std::cout << "Accumulated Time: " << ts_stats.accumulated_time << "s\n";
    std::cout << "Max Accumulated: " << ts_stats.max_accumulated_time << "s\n";
    std::cout << "Overflows: " << ts_stats.overflow_count << '\n';
    std::cout << "Subdivisions: " << ts_stats.subdivision_count << '\n';
}

float PhysicsContext::target_timestep() const
{
    return 1.0f / config_.target_fps;
}

} // namespace phynity::app

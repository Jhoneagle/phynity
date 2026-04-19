#include <core/physics/rigid_bodies/rigid_body_system.hpp>
#include <core/physics/particles/particle_system.hpp>
#include <core/serialization/snapshot_helpers.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <utility>

namespace phynity::serialization
{

namespace
{

std::unique_ptr<ReplayWriter> g_replay_writer;
std::mutex g_replay_mutex;
SnapshotHelpers::AuditSink g_audit_sink;
std::mutex g_audit_sink_mutex;

void emit_audit_event(const std::string &message)
{
    SnapshotHelpers::AuditSink sink;
    {
        std::lock_guard<std::mutex> lock(g_audit_sink_mutex);
        sink = g_audit_sink;
    }

    if (sink)
    {
        sink(message);
    }
}

} // namespace

SerializationResult SnapshotHelpers::start_replay_capture(const std::string &replay_file)
{
    std::lock_guard<std::mutex> lock(g_replay_mutex);
    if (g_replay_writer)
    {
        return {SerializationError::IOError, "Replay capture already active", 0};
    }

    auto writer = std::make_unique<ReplayWriter>();
    const auto open_result = writer->open(replay_file);
    if (!open_result.is_success())
    {
        return open_result;
    }

    g_replay_writer = std::move(writer);
    return {SerializationError::Success, "", 0};
}

SerializationResult SnapshotHelpers::append_replay_frame(const PhysicsSnapshot &snapshot)
{
    std::lock_guard<std::mutex> lock(g_replay_mutex);
    if (!g_replay_writer)
    {
        return {SerializationError::IOError, "Replay capture is not active", 0};
    }

    return g_replay_writer->append_frame(snapshot);
}

SerializationResult SnapshotHelpers::append_replay_frame(const phynity::physics::ParticleSystem &system,
                                                         uint64_t frame_number,
                                                         double simulated_time,
                                                         uint32_t rng_seed)
{
    const auto snapshot = capture_particle_system(system, frame_number, simulated_time, rng_seed);
    return append_replay_frame(snapshot);
}

SerializationResult SnapshotHelpers::append_replay_frame(const phynity::physics::RigidBodySystem &system,
                                                         uint64_t frame_number,
                                                         double simulated_time,
                                                         uint32_t rng_seed)
{
    const auto snapshot = capture_rigid_body_system(system, frame_number, simulated_time, rng_seed);
    return append_replay_frame(snapshot);
}

SerializationResult SnapshotHelpers::stop_replay_capture()
{
    std::lock_guard<std::mutex> lock(g_replay_mutex);
    if (!g_replay_writer)
    {
        return {SerializationError::IOError, "Replay capture is not active", 0};
    }

    const auto close_result = g_replay_writer->close();
    g_replay_writer.reset();
    return close_result;
}

// ============================================================================
// Snapshot Capture
// ============================================================================

PhysicsSnapshot SnapshotHelpers::capture_particle_system(const phynity::physics::ParticleSystem &system,
                                                         uint64_t frame_number,
                                                         double simulated_time,
                                                         uint32_t rng_seed)
{
    PhysicsSnapshot snapshot;
    snapshot.schema_version = current_schema_version();
    snapshot.frame_number = frame_number;
    snapshot.simulated_time = simulated_time;
    snapshot.rng_seed = rng_seed;

    for (const auto &particle : system.particles())
    {
        if (!particle.active)
        {
            continue;
        }

        ParticleSnapshot particle_snapshot;
        particle_snapshot.position = particle.position;
        particle_snapshot.velocity = particle.velocity;
        particle_snapshot.acceleration = particle.acceleration;
        particle_snapshot.force_accumulator = particle.force_accumulator;
        particle_snapshot.radius = particle.radius;
        particle_snapshot.mass = particle.material.mass;
        particle_snapshot.restitution = particle.material.restitution;
        particle_snapshot.friction = particle.material.friction;
        particle_snapshot.linear_damping = particle.material.linear_damping;
        particle_snapshot.angular_damping = particle.material.angular_damping;
        particle_snapshot.drag_coefficient = particle.material.drag_coefficient;
        particle_snapshot.lifetime = particle.lifetime;
        particle_snapshot.active = particle.active;
        snapshot.particles.push_back(particle_snapshot);
    }

    return snapshot;
}

PhysicsSnapshot SnapshotHelpers::capture_rigid_body_system(const phynity::physics::RigidBodySystem &system,
                                                           uint64_t frame_number,
                                                           double simulated_time,
                                                           uint32_t rng_seed)
{
    PhysicsSnapshot snapshot;
    snapshot.schema_version = current_schema_version();
    snapshot.frame_number = frame_number;
    snapshot.simulated_time = simulated_time;
    snapshot.rng_seed = rng_seed;

    for (const auto &body : system.bodies())
    {
        if (!body.active)
        {
            continue;
        }

        RigidBodySnapshot rigid_snapshot;
        rigid_snapshot.position = body.position;
        rigid_snapshot.velocity = body.velocity;
        rigid_snapshot.force_accumulator = body.force_accumulator;
        rigid_snapshot.orientation_w = body.orientation.w;
        rigid_snapshot.orientation_x = body.orientation.x;
        rigid_snapshot.orientation_y = body.orientation.y;
        rigid_snapshot.orientation_z = body.orientation.z;
        rigid_snapshot.angular_velocity = body.angular_velocity;
        rigid_snapshot.torque_accumulator = body.torque_accumulator;
        rigid_snapshot.collision_radius = body.collision_radius;

        rigid_snapshot.mass = body.get_mass();
        rigid_snapshot.restitution = body.material.restitution;
        rigid_snapshot.friction = body.material.friction;
        rigid_snapshot.linear_damping = body.material.linear_damping;
        rigid_snapshot.angular_damping = body.material.angular_damping;
        rigid_snapshot.drag_coefficient = body.material.drag_coefficient;

        rigid_snapshot.active = body.active;
        rigid_snapshot.id = body.id;
        rigid_snapshot.lifetime = body.lifetime;

        if (body.shape)
        {
            rigid_snapshot.shape_local_center = body.shape->local_center;

            if (body.shape->shape_type == phynity::physics::ShapeType::Sphere)
            {
                const auto *sphere = dynamic_cast<const phynity::physics::SphereShape *>(body.shape.get());
                if (sphere)
                {
                    rigid_snapshot.shape_type = SnapshotShapeType::Sphere;
                    rigid_snapshot.shape_radius = sphere->radius;
                }
            }
            else if (body.shape->shape_type == phynity::physics::ShapeType::Box)
            {
                const auto *box = dynamic_cast<const phynity::physics::BoxShape *>(body.shape.get());
                if (box)
                {
                    rigid_snapshot.shape_type = SnapshotShapeType::Box;
                    rigid_snapshot.shape_half_extents = box->half_extents;
                }
            }
            else if (body.shape->shape_type == phynity::physics::ShapeType::Capsule)
            {
                const auto *capsule = dynamic_cast<const phynity::physics::CapsuleShape *>(body.shape.get());
                if (capsule)
                {
                    rigid_snapshot.shape_type = SnapshotShapeType::Capsule;
                    rigid_snapshot.shape_radius = capsule->radius;
                    rigid_snapshot.shape_half_height = capsule->half_height;
                }
            }
        }

        snapshot.rigid_bodies.push_back(rigid_snapshot);
    }

    return snapshot;
}

// ============================================================================
// Snapshot Restoration
// ============================================================================

bool SnapshotHelpers::restore_particle_system(const PhysicsSnapshot &snapshot, phynity::physics::ParticleSystem &system)
{
    system.clear();

    for (const auto &particle_snapshot : snapshot.particles)
    {
        phynity::physics::Material material;
        material.mass = particle_snapshot.mass;
        material.restitution = particle_snapshot.restitution;
        material.friction = particle_snapshot.friction;
        material.linear_damping = particle_snapshot.linear_damping;
        material.angular_damping = particle_snapshot.angular_damping;
        material.drag_coefficient = particle_snapshot.drag_coefficient;

        system.spawn(particle_snapshot.position,
                     particle_snapshot.velocity,
                     material,
                     particle_snapshot.lifetime,
                     particle_snapshot.radius);

        auto &restored_particle = system.particles().back();
        restored_particle.acceleration = particle_snapshot.acceleration;
        restored_particle.force_accumulator = particle_snapshot.force_accumulator;
        restored_particle.active = particle_snapshot.active;
    }

    return true;
}

bool SnapshotHelpers::restore_rigid_body_system(const PhysicsSnapshot &snapshot,
                                                phynity::physics::RigidBodySystem &system)
{
    system.clear_bodies();

    int max_id = -1;
    for (const auto &rigid_snapshot : snapshot.rigid_bodies)
    {
        std::shared_ptr<phynity::physics::Shape> shape = nullptr;
        if (rigid_snapshot.shape_type == SnapshotShapeType::Sphere)
        {
            shape = std::make_shared<phynity::physics::SphereShape>(rigid_snapshot.shape_radius,
                                                                    rigid_snapshot.shape_local_center);
        }
        else if (rigid_snapshot.shape_type == SnapshotShapeType::Box)
        {
            shape = std::make_shared<phynity::physics::BoxShape>(rigid_snapshot.shape_half_extents,
                                                                 rigid_snapshot.shape_local_center);
        }
        else if (rigid_snapshot.shape_type == SnapshotShapeType::Capsule)
        {
            shape = std::make_shared<phynity::physics::CapsuleShape>(
                rigid_snapshot.shape_radius, rigid_snapshot.shape_half_height, rigid_snapshot.shape_local_center);
        }

        phynity::physics::Material material;
        material.mass = rigid_snapshot.mass;
        material.restitution = rigid_snapshot.restitution;
        material.friction = rigid_snapshot.friction;
        material.linear_damping = rigid_snapshot.linear_damping;
        material.angular_damping = rigid_snapshot.angular_damping;
        material.drag_coefficient = rigid_snapshot.drag_coefficient;

        const auto orientation = phynity::math::quaternions::Quatf(rigid_snapshot.orientation_w,
                                                                   rigid_snapshot.orientation_x,
                                                                   rigid_snapshot.orientation_y,
                                                                   rigid_snapshot.orientation_z);

        const auto new_id =
            system.spawn_body(rigid_snapshot.position, orientation, shape, rigid_snapshot.mass, material);

        auto *body = system.get_body(new_id);
        if (!body)
        {
            return false;
        }

        body->velocity = rigid_snapshot.velocity;
        body->force_accumulator = rigid_snapshot.force_accumulator;
        body->angular_velocity = rigid_snapshot.angular_velocity;
        body->torque_accumulator = rigid_snapshot.torque_accumulator;
        body->collision_radius = rigid_snapshot.collision_radius;
        body->active = rigid_snapshot.active;
        body->lifetime = rigid_snapshot.lifetime;
        body->id = rigid_snapshot.id;

        max_id = std::max(max_id, rigid_snapshot.id);
    }

    system.set_next_body_id(max_id + 1);
    return true;
}

// ============================================================================
// Snapshot Comparison
// ============================================================================

bool SnapshotHelpers::snapshots_equal_with_tolerance(const PhysicsSnapshot &expected,
                                                     const PhysicsSnapshot &actual,
                                                     float position_tolerance,
                                                     float impulse_tolerance,
                                                     std::string *error_report)
{
    if (expected.frame_number != actual.frame_number)
    {
        if (error_report)
        {
            *error_report = "Frame mismatch: expected " + std::to_string(expected.frame_number) + ", got " +
                            std::to_string(actual.frame_number);
        }
        return false;
    }

    if (std::abs(expected.simulated_time - actual.simulated_time) > static_cast<double>(position_tolerance) ||
        std::abs(expected.timestep - actual.timestep) > position_tolerance || expected.rng_seed != actual.rng_seed)
    {
        if (error_report)
        {
            *error_report = "Snapshot metadata mismatch";
        }
        return false;
    }

    if (expected.particles.size() != actual.particles.size())
    {
        if (error_report)
            *error_report = "Particle count mismatch: expected " + std::to_string(expected.particles.size()) +
                            ", got " + std::to_string(actual.particles.size());
        return false;
    }

    if (expected.rigid_bodies.size() != actual.rigid_bodies.size())
    {
        if (error_report)
            *error_report = "Rigid body count mismatch: expected " + std::to_string(expected.rigid_bodies.size()) +
                            ", got " + std::to_string(actual.rigid_bodies.size());
        return false;
    }

    // Check schema version compatibility
    if (expected.schema_version != actual.schema_version &&
        !expected.schema_version.is_compatible_with(actual.schema_version))
    {
        if (error_report)
            *error_report = "Schema version incompatible: expected " + expected.schema_version.to_string() + ", got " +
                            actual.schema_version.to_string();
        return false;
    }

    // Compare particles
    std::ostringstream report;
    bool all_match = true;

    for (size_t i = 0; i < expected.particles.size(); ++i)
    {
        if (!expected.particles[i].equals_with_tolerance(actual.particles[i], position_tolerance))
        {
            all_match = false;
            report << "Particle " << i << " mismatch:\n";
            const auto &exp = expected.particles[i];
            const auto &act = actual.particles[i];

            auto check_vec = [&report](const std::string &name,
                                       const phynity::math::vectors::Vec3f &exp_vec,
                                       const phynity::math::vectors::Vec3f &act_vec,
                                       float tolerance)
            {
                const float diff = (exp_vec - act_vec).length();
                if (diff > tolerance)
                {
                    report << "  " << name << " diff: " << std::scientific << std::setprecision(6) << diff << "\n";
                    report << "    expected: [" << exp_vec.x << ", " << exp_vec.y << ", " << exp_vec.z << "]\n";
                    report << "    actual:   [" << act_vec.x << ", " << act_vec.y << ", " << act_vec.z << "]\n";
                }
            };

            auto check_scalar =
                [&report](const std::string &name, float expected_value, float actual_value, float tolerance)
            {
                const float diff = std::abs(expected_value - actual_value);
                if (diff > tolerance)
                {
                    report << "  " << name << " diff: " << std::scientific << std::setprecision(6) << diff << "\n";
                    report << "    expected: " << expected_value << "\n";
                    report << "    actual:   " << actual_value << "\n";
                }
            };

            check_vec("position", exp.position, act.position, position_tolerance);
            check_vec("velocity", exp.velocity, act.velocity, position_tolerance);
            check_vec("acceleration", exp.acceleration, act.acceleration, position_tolerance);
            check_vec("force_accumulator", exp.force_accumulator, act.force_accumulator, impulse_tolerance);
            check_scalar("radius", exp.radius, act.radius, position_tolerance);
            check_scalar("mass", exp.mass, act.mass, impulse_tolerance);
            check_scalar("restitution", exp.restitution, act.restitution, impulse_tolerance);
            check_scalar("friction", exp.friction, act.friction, impulse_tolerance);
            check_scalar("linear_damping", exp.linear_damping, act.linear_damping, impulse_tolerance);
            check_scalar("angular_damping", exp.angular_damping, act.angular_damping, impulse_tolerance);
            check_scalar("drag_coefficient", exp.drag_coefficient, act.drag_coefficient, impulse_tolerance);
            check_scalar("lifetime", exp.lifetime, act.lifetime, impulse_tolerance);
            if (exp.active != act.active)
            {
                report << "  active mismatch:\n";
                report << "    expected: " << exp.active << "\n";
                report << "    actual:   " << act.active << "\n";
            }
        }
    }

    for (size_t i = 0; i < expected.rigid_bodies.size(); ++i)
    {
        const auto &exp = expected.rigid_bodies[i];
        const auto &act = actual.rigid_bodies[i];

        auto check_vec = [&report, &all_match](const std::string &name,
                                               const phynity::math::vectors::Vec3f &exp_vec,
                                               const phynity::math::vectors::Vec3f &act_vec,
                                               float tolerance,
                                               size_t index)
        {
            const float diff = (exp_vec - act_vec).length();
            if (diff > tolerance)
            {
                all_match = false;
                report << "RigidBody " << index << " " << name << " diff: " << std::scientific << std::setprecision(6)
                       << diff << "\n";
            }
        };

        auto check_scalar =
            [&report, &all_match](
                const std::string &name, float expected_value, float actual_value, float tolerance, size_t index)
        {
            const float diff = std::abs(expected_value - actual_value);
            if (diff > tolerance)
            {
                all_match = false;
                report << "RigidBody " << index << " " << name << " diff: " << std::scientific << std::setprecision(6)
                       << diff << "\n";
            }
        };

        check_vec("position", exp.position, act.position, position_tolerance, i);
        check_vec("velocity", exp.velocity, act.velocity, position_tolerance, i);
        check_vec("force_accumulator", exp.force_accumulator, act.force_accumulator, impulse_tolerance, i);
        check_scalar("orientation_w", exp.orientation_w, act.orientation_w, position_tolerance, i);
        check_scalar("orientation_x", exp.orientation_x, act.orientation_x, position_tolerance, i);
        check_scalar("orientation_y", exp.orientation_y, act.orientation_y, position_tolerance, i);
        check_scalar("orientation_z", exp.orientation_z, act.orientation_z, position_tolerance, i);
        check_vec("angular_velocity", exp.angular_velocity, act.angular_velocity, position_tolerance, i);
        check_vec("torque_accumulator", exp.torque_accumulator, act.torque_accumulator, impulse_tolerance, i);
        check_scalar("shape_radius", exp.shape_radius, act.shape_radius, impulse_tolerance, i);
        check_vec("shape_half_extents", exp.shape_half_extents, act.shape_half_extents, impulse_tolerance, i);
        check_scalar("shape_half_height", exp.shape_half_height, act.shape_half_height, impulse_tolerance, i);
        check_scalar("collision_radius", exp.collision_radius, act.collision_radius, impulse_tolerance, i);
        check_scalar("mass", exp.mass, act.mass, impulse_tolerance, i);
        check_scalar("restitution", exp.restitution, act.restitution, impulse_tolerance, i);
        check_scalar("friction", exp.friction, act.friction, impulse_tolerance, i);
        check_scalar("linear_damping", exp.linear_damping, act.linear_damping, impulse_tolerance, i);
        check_scalar("angular_damping", exp.angular_damping, act.angular_damping, impulse_tolerance, i);
        check_scalar("drag_coefficient", exp.drag_coefficient, act.drag_coefficient, impulse_tolerance, i);
        check_scalar("lifetime", exp.lifetime, act.lifetime, impulse_tolerance, i);

        if (exp.shape_type != act.shape_type || exp.active != act.active || exp.id != act.id)
        {
            all_match = false;
            report << "RigidBody " << i << " enum/bool/id mismatch\n";
        }
    }

    if (error_report && !all_match)
        *error_report = report.str();

    return all_match;
}

// ============================================================================
// Golden Files
// ============================================================================

SerializationResult SnapshotHelpers::load_golden(const std::string &golden_file, PhysicsSnapshot &snapshot)
{
    const bool requested_json = golden_file.size() >= 5 && golden_file.rfind(".json") == golden_file.size() - 5;
    const bool requested_bin = golden_file.size() >= 4 && golden_file.rfind(".bin") == golden_file.size() - 4;

    std::string json_path = golden_file;
    std::string bin_path = golden_file;
    if (requested_json)
    {
        bin_path = golden_file.substr(0, golden_file.size() - 5) + ".bin";
    }
    else if (requested_bin)
    {
        json_path = golden_file.substr(0, golden_file.size() - 4) + ".json";
    }
    else
    {
        json_path = golden_file + ".json";
        bin_path = golden_file + ".bin";
    }

    if (requested_json)
    {
        auto json_result = SnapshotSerializer::load_json(json_path, snapshot);
        if (json_result.is_success())
        {
            return json_result;
        }
        return SnapshotSerializer::load_binary(bin_path, snapshot);
    }

    if (requested_bin)
    {
        auto bin_result = SnapshotSerializer::load_binary(bin_path, snapshot);
        if (bin_result.is_success())
        {
            return bin_result;
        }
        return SnapshotSerializer::load_json(json_path, snapshot);
    }

    auto bin_result = SnapshotSerializer::load_binary(bin_path, snapshot);
    if (bin_result.is_success())
    {
        return bin_result;
    }
    return SnapshotSerializer::load_json(json_path, snapshot);
}

SerializationResult SnapshotHelpers::save_golden(const PhysicsSnapshot &snapshot, const std::string &golden_file)
{
    // Save both binary and JSON for robustness
    return SnapshotSerializer::save_both(snapshot, golden_file.substr(0, golden_file.find_last_of(".")));
}

SerializationResult SnapshotHelpers::update_golden(const PhysicsSnapshot &snapshot,
                                                   const std::string &golden_file,
                                                   const std::string &log_message)
{
    const auto save_result = save_golden(snapshot, golden_file);

    std::ostringstream audit_message;
    audit_message << "event=snapshot.update_golden"
                  << " file=" << golden_file << " reason=" << (log_message.empty() ? "(empty)" : log_message)
                  << " status=" << (save_result.is_success() ? "success" : "failure");

    if (!save_result.is_success())
    {
        audit_message << " error=" << save_result.error_message;
    }

    emit_audit_event(audit_message.str());
    return save_result;
}

void SnapshotHelpers::set_audit_sink(AuditSink sink)
{
    std::lock_guard<std::mutex> lock(g_audit_sink_mutex);
    g_audit_sink = std::move(sink);
}

void SnapshotHelpers::clear_audit_sink()
{
    std::lock_guard<std::mutex> lock(g_audit_sink_mutex);
    g_audit_sink = nullptr;
}

// ============================================================================
// Debugging
// ============================================================================

std::string SnapshotHelpers::generate_diff_report(const PhysicsSnapshot &expected, const PhysicsSnapshot &actual)
{
    std::ostringstream report;

    report << "=== Determinism Diff Report ===\n";
    report << "Expected snapshot: frame " << expected.frame_number << " at t=" << expected.simulated_time << "s\n";
    report << "Actual snapshot:   frame " << actual.frame_number << " at t=" << actual.simulated_time << "s\n\n";

    report << "Particle count: expected " << expected.particles.size() << ", actual " << actual.particles.size()
           << "\n";

    if (expected.particles.size() == actual.particles.size())
    {
        for (size_t i = 0; i < expected.particles.size(); ++i)
        {
            const auto &exp = expected.particles[i];
            const auto &act = actual.particles[i];

            auto pos_diff = (exp.position - act.position).length();
            auto vel_diff = (exp.velocity - act.velocity).length();

            if (pos_diff > 1e-6f || vel_diff > 1e-6f)
            {
                report << "\nParticle " << i << ":\n";
                report << "  Position delta:  " << std::scientific << std::setprecision(6) << pos_diff << "\n";
                report << "  Velocity delta:  " << vel_diff << "\n";
                report << "  Expected pos: [" << exp.position.x << ", " << exp.position.y << ", " << exp.position.z
                       << "]\n";
                report << "  Actual pos:   [" << act.position.x << ", " << act.position.y << ", " << act.position.z
                       << "]\n";
            }
        }
    }

    return report.str();
}

bool SnapshotHelpers::validate_golden_exists(const std::string &golden_file, const SchemaVersion &schema_version)
{
    // Simplistic check: just try to load
    PhysicsSnapshot temp;
    auto result = load_golden(golden_file, temp);
    if (!result.is_success())
        return false;

    return temp.schema_version.is_compatible_with(schema_version);
}

} // namespace phynity::serialization

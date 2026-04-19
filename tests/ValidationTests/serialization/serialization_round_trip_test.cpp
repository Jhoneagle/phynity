#include <catch2/catch_all.hpp>
#include <core/physics/rigid_bodies/rigid_body_system.hpp>
#include <core/physics/rigid_bodies/shape.hpp>
#include <core/physics/particles/particle_system.hpp>
#include <core/serialization/snapshot_helpers.hpp>
#include <core/serialization/snapshot_schema.hpp>
#include <core/serialization/snapshot_serializer.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <filesystem>
#include <fstream>
#include <utility>
#include <vector>

using namespace phynity::physics;
using namespace phynity::serialization;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers::constants;

namespace fs = std::filesystem;

namespace
{

#define STRINGIFY(x) #x
#define STRINGIFY_EXPANDED(x) STRINGIFY(x)

std::string get_golden_dir()
{
#ifdef GOLDEN_FILES_DIR
    return STRINGIFY_EXPANDED(GOLDEN_FILES_DIR);
#else
    return "tests/golden_outputs";
#endif
}

struct SerializationFixture
{
    fs::path temp_dir = fs::temp_directory_path() / "phynity_serialization_tests";

    SerializationFixture()
    {
        fs::create_directories(temp_dir);
    }

    ~SerializationFixture()
    {
        fs::remove_all(temp_dir);
    }

    PhysicsSnapshot create_snapshot(int particle_count = 3) const
    {
        PhysicsSnapshot snapshot;
        snapshot.schema_version = current_schema_version();
        snapshot.frame_number = 42;
        snapshot.simulated_time = 0.5;
        snapshot.timestep = DETERMINISTIC_TIMESTEP;
        snapshot.rng_seed = 12345;

        for (int index = 0; index < particle_count; ++index)
        {
            const float index_f = static_cast<float>(index);
            ParticleSnapshot particle;
            particle.position = Vec3f(index_f, index_f * 1.5f, index_f * -0.5f);
            particle.velocity = Vec3f(1.0f + index_f, -2.0f, 3.0f - index_f);
            particle.acceleration = Vec3f(0.1f * index_f, 0.2f, 0.3f);
            particle.force_accumulator = Vec3f(0.4f, 0.5f * index_f, -0.25f);
            particle.radius = 0.5f + index_f * 0.1f;
            particle.mass = 1.0f + index_f * 0.2f;
            particle.restitution = 0.2f + index_f * 0.05f;
            particle.friction = 0.3f + index_f * 0.05f;
            particle.linear_damping = 0.01f + index_f * 0.005f;
            particle.angular_damping = 0.02f + index_f * 0.005f;
            particle.drag_coefficient = 0.1f + index_f * 0.01f;
            particle.lifetime = (index % 2 == 0) ? -1.0f : 5.0f + index_f;
            particle.active = (index % 2 == 0) || index == particle_count - 1;
            snapshot.particles.push_back(particle);
        }

        return snapshot;
    }

    PhysicsSnapshot create_snapshot_with_rigid_bodies() const
    {
        PhysicsSnapshot snapshot = create_snapshot(3);

        RigidBodySnapshot rigid_a;
        rigid_a.position = Vec3f(1.0f, 2.0f, -1.0f);
        rigid_a.velocity = Vec3f(0.25f, -0.5f, 0.1f);
        rigid_a.force_accumulator = Vec3f(0.0f, 9.81f, 0.0f);
        rigid_a.orientation_w = 1.0f;
        rigid_a.orientation_x = 0.0f;
        rigid_a.orientation_y = 0.0f;
        rigid_a.orientation_z = 0.0f;
        rigid_a.angular_velocity = Vec3f(0.0f, 0.4f, 0.2f);
        rigid_a.torque_accumulator = Vec3f(0.0f, 0.05f, 0.0f);
        rigid_a.shape_type = SnapshotShapeType::Sphere;
        rigid_a.shape_radius = 0.45f;
        rigid_a.collision_radius = 0.45f;
        rigid_a.mass = 2.0f;
        rigid_a.restitution = 0.35f;
        rigid_a.friction = 0.55f;
        rigid_a.linear_damping = 0.02f;
        rigid_a.angular_damping = 0.03f;
        rigid_a.drag_coefficient = 0.01f;
        rigid_a.active = true;
        rigid_a.id = 7;
        rigid_a.lifetime = -1.0f;

        RigidBodySnapshot rigid_b;
        rigid_b.position = Vec3f(-2.0f, 1.0f, 0.5f);
        rigid_b.velocity = Vec3f(-0.4f, 0.0f, 0.3f);
        rigid_b.force_accumulator = Vec3f(0.0f);
        rigid_b.orientation_w = 0.9238795f;
        rigid_b.orientation_x = 0.0f;
        rigid_b.orientation_y = 0.3826834f;
        rigid_b.orientation_z = 0.0f;
        rigid_b.angular_velocity = Vec3f(0.0f, -0.2f, 0.0f);
        rigid_b.torque_accumulator = Vec3f(0.01f, 0.0f, -0.02f);
        rigid_b.shape_type = SnapshotShapeType::Box;
        rigid_b.shape_local_center = Vec3f(0.0f, 0.1f, 0.0f);
        rigid_b.shape_half_extents = Vec3f(0.3f, 0.2f, 0.4f);
        rigid_b.shape_half_height = 0.25f;
        rigid_b.shape_radius = 0.4f;
        rigid_b.collision_radius = 0.6f;
        rigid_b.mass = 3.5f;
        rigid_b.restitution = 0.2f;
        rigid_b.friction = 0.7f;
        rigid_b.linear_damping = 0.015f;
        rigid_b.angular_damping = 0.025f;
        rigid_b.drag_coefficient = 0.005f;
        rigid_b.active = true;
        rigid_b.id = 11;
        rigid_b.lifetime = 12.0f;

        snapshot.rigid_bodies.push_back(rigid_a);
        snapshot.rigid_bodies.push_back(rigid_b);
        return snapshot;
    }

    PhysicsSnapshot expected_legacy_snapshot() const
    {
        PhysicsSnapshot snapshot;
        snapshot.schema_version = current_schema_version();
        snapshot.frame_number = 7;
        snapshot.simulated_time = 0.1166667;
        snapshot.timestep = DETERMINISTIC_TIMESTEP;
        snapshot.rng_seed = 9001;

        ParticleSnapshot particle_a;
        particle_a.position = Vec3f(0.0f, 1.0f, 2.0f);
        particle_a.velocity = Vec3f(0.5f, -0.25f, 0.0f);
        particle_a.radius = 0.45f;
        particle_a.mass = 2.0f;
        particle_a.active = true;
        snapshot.particles.push_back(particle_a);

        ParticleSnapshot particle_b;
        particle_b.position = Vec3f(-1.0f, 0.0f, 1.0f);
        particle_b.velocity = Vec3f(1.0f, 0.0f, -1.0f);
        particle_b.radius = 0.55f;
        particle_b.mass = 1.5f;
        particle_b.active = false;
        snapshot.particles.push_back(particle_b);

        return snapshot;
    }

    PhysicsSnapshot expected_n2_legacy_snapshot() const
    {
        PhysicsSnapshot snapshot;
        snapshot.schema_version = current_schema_version();
        snapshot.frame_number = 5;
        snapshot.simulated_time = 0.0833333;
        snapshot.timestep = DETERMINISTIC_TIMESTEP;
        snapshot.rng_seed = 4242;

        ParticleSnapshot particle;
        particle.position = Vec3f(2.0f, -1.0f, 0.5f);
        particle.velocity = Vec3f(-0.5f, 0.25f, 0.0f);
        particle.radius = 0.7f;
        particle.mass = 3.0f;
        // v0.9 fixture intentionally omits these fields in JSON.
        // Expected migration behavior is fallback to struct defaults.
        particle.acceleration = Vec3f(0.0f);
        particle.force_accumulator = Vec3f(0.0f);
        particle.restitution = 0.1f;
        particle.friction = 0.5f;
        particle.linear_damping = 0.01f;
        particle.angular_damping = 0.01f;
        particle.drag_coefficient = 0.0f;
        particle.lifetime = -1.0f;
        particle.active = true;
        snapshot.particles.push_back(particle);

        return snapshot;
    }
};

struct SnapshotFileHeaderV1Test
{
    uint32_t magic = SnapshotFileHeader::MAGIC_NUMBER;
    uint32_t format_version = 1;
    uint32_t schema_major = 1;
    uint32_t schema_minor = 0;
    uint32_t schema_patch = 0;
    uint32_t num_particles = 0;
    uint64_t file_size = 0;
};

struct SnapshotFileHeaderV2LegacyTest
{
    uint32_t magic = SnapshotFileHeader::MAGIC_NUMBER;
    uint32_t format_version = SnapshotFileHeader::FORMAT_VERSION;
    uint32_t schema_major = 1;
    uint32_t schema_minor = 1;
    uint32_t schema_patch = 0;
    uint32_t num_particles = 0;
    uint32_t num_rigid_bodies = 0;
    uint32_t metadata_json_bytes = 0;
    uint64_t file_size = 0;
};

struct ScopedSnapshotAuditSink
{
    explicit ScopedSnapshotAuditSink(SnapshotHelpers::AuditSink sink)
    {
        SnapshotHelpers::set_audit_sink(std::move(sink));
    }

    ~ScopedSnapshotAuditSink()
    {
        SnapshotHelpers::clear_audit_sink();
    }
};

SerializationResult write_legacy_format_v1_fixture(const std::string &path, const PhysicsSnapshot &snapshot)
{
    SerializationResult result;
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output.is_open())
    {
        result.error = SerializationError::WriteError;
        result.error_message = "Cannot open legacy fixture for writing: " + path;
        return result;
    }

    SnapshotFileHeaderV1Test header;
    header.schema_major = snapshot.schema_version.major;
    header.schema_minor = snapshot.schema_version.minor;
    header.schema_patch = snapshot.schema_version.patch;
    header.num_particles = static_cast<uint32_t>(snapshot.particles.size());
    header.file_size = sizeof(SnapshotFileHeaderV1Test) + sizeof(snapshot.frame_number) +
                       sizeof(snapshot.simulated_time) + sizeof(snapshot.timestep) + sizeof(snapshot.rng_seed) +
                       snapshot.particles.size() * sizeof(ParticleSnapshot);

    output.write(reinterpret_cast<const char *>(&header), sizeof(header));
    output.write(reinterpret_cast<const char *>(&snapshot.frame_number), sizeof(snapshot.frame_number));
    output.write(reinterpret_cast<const char *>(&snapshot.simulated_time), sizeof(snapshot.simulated_time));
    output.write(reinterpret_cast<const char *>(&snapshot.timestep), sizeof(snapshot.timestep));
    output.write(reinterpret_cast<const char *>(&snapshot.rng_seed), sizeof(snapshot.rng_seed));

    for (const auto &particle : snapshot.particles)
    {
        output.write(reinterpret_cast<const char *>(&particle), sizeof(ParticleSnapshot));
    }

    if (!output.good())
    {
        result.error = SerializationError::IOError;
        result.error_message = "Failed to write full legacy fixture bytes";
        return result;
    }

    result.error = SerializationError::Success;
    result.bytes_processed = static_cast<size_t>(header.file_size);
    return result;
}

SerializationResult write_legacy_format_v2_raw_fixture(const std::string &path, const PhysicsSnapshot &snapshot)
{
    SerializationResult result;
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output.is_open())
    {
        result.error = SerializationError::WriteError;
        result.error_message = "Cannot open legacy v2 fixture for writing: " + path;
        return result;
    }

    SnapshotFileHeaderV2LegacyTest header;
    header.schema_major = snapshot.schema_version.major;
    header.schema_minor = snapshot.schema_version.minor;
    header.schema_patch = snapshot.schema_version.patch;
    header.num_particles = static_cast<uint32_t>(snapshot.particles.size());
    header.num_rigid_bodies = static_cast<uint32_t>(snapshot.rigid_bodies.size());
    header.metadata_json_bytes = 0;
    header.file_size = sizeof(SnapshotFileHeaderV2LegacyTest) + sizeof(snapshot.frame_number) +
                       sizeof(snapshot.simulated_time) + sizeof(snapshot.timestep) + sizeof(snapshot.rng_seed) +
                       snapshot.particles.size() * sizeof(ParticleSnapshot) +
                       snapshot.rigid_bodies.size() * sizeof(RigidBodySnapshot);

    output.write(reinterpret_cast<const char *>(&header), sizeof(header));
    output.write(reinterpret_cast<const char *>(&snapshot.frame_number), sizeof(snapshot.frame_number));
    output.write(reinterpret_cast<const char *>(&snapshot.simulated_time), sizeof(snapshot.simulated_time));
    output.write(reinterpret_cast<const char *>(&snapshot.timestep), sizeof(snapshot.timestep));
    output.write(reinterpret_cast<const char *>(&snapshot.rng_seed), sizeof(snapshot.rng_seed));

    for (const auto &particle : snapshot.particles)
    {
        output.write(reinterpret_cast<const char *>(&particle), sizeof(ParticleSnapshot));
    }

    for (const auto &rigid_body : snapshot.rigid_bodies)
    {
        output.write(reinterpret_cast<const char *>(&rigid_body), sizeof(RigidBodySnapshot));
    }

    if (!output.good())
    {
        result.error = SerializationError::IOError;
        result.error_message = "Failed to write full legacy v2 raw fixture bytes";
        return result;
    }

    result.error = SerializationError::Success;
    result.bytes_processed = static_cast<size_t>(header.file_size);
    return result;
}

} // namespace

TEST_CASE("Schema Version - to_string", "[serialization][schema]")
{
    SchemaVersion version{2, 3, 4};
    REQUIRE(version.to_string() == "2.3.4");
}

TEST_CASE("Schema Version - compatibility", "[serialization][schema]")
{
    const auto current = current_schema_version();
    const SchemaVersion n2_legacy{0, 9, 0};
    const SchemaVersion legacy{1, 0, 0};
    const SchemaVersion future_major{2, 0, 0};

    REQUIRE(n2_legacy != current);
    REQUIRE(SnapshotSerializer::can_migrate(legacy, current));
    REQUIRE(SnapshotSerializer::can_migrate(n2_legacy, current));
    REQUIRE_FALSE(SnapshotSerializer::can_migrate(future_major, current));
}

TEST_CASE("Binary Serialization - round-trip with valid snapshot", "[serialization][roundtrip]")
{
    SerializationFixture fixture;
    const PhysicsSnapshot original = fixture.create_snapshot(5);
    const std::string file_path = (fixture.temp_dir / "snapshot.bin").string();

    const auto save_result = SnapshotSerializer::save_binary(original, file_path);
    REQUIRE(save_result.is_success());

    PhysicsSnapshot loaded;
    const auto load_result = SnapshotSerializer::load_binary(file_path, loaded);
    REQUIRE(load_result.is_success());

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(original, loaded, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("Binary Serialization - round-trip preserves rigid body snapshots", "[serialization][roundtrip][rigid-body]")
{
    SerializationFixture fixture;
    const PhysicsSnapshot original = fixture.create_snapshot_with_rigid_bodies();
    const std::string file_path = (fixture.temp_dir / "snapshot_rigid.bin").string();

    const auto save_result = SnapshotSerializer::save_binary(original, file_path);
    REQUIRE(save_result.is_success());

    PhysicsSnapshot loaded;
    const auto load_result = SnapshotSerializer::load_binary(file_path, loaded);
    REQUIRE(load_result.is_success());
    REQUIRE(loaded.rigid_bodies.size() == original.rigid_bodies.size());

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(original, loaded, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("JSON Serialization - round-trip preserves full snapshot", "[serialization][json]")
{
    SerializationFixture fixture;
    const PhysicsSnapshot original = fixture.create_snapshot(4);
    const std::string file_path = (fixture.temp_dir / "snapshot.json").string();

    const auto save_result = SnapshotSerializer::save_json(original, file_path, true);
    REQUIRE(save_result.is_success());

    PhysicsSnapshot loaded;
    const auto load_result = SnapshotSerializer::load_json(file_path, loaded);
    REQUIRE(load_result.is_success());

    std::ifstream file(file_path);
    const std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    REQUIRE(content.find("\"force_accumulator\"") != std::string::npos);
    REQUIRE(content.find("\"linear_damping\"") != std::string::npos);

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(original, loaded, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("Save both - surfaces JSON write errors", "[serialization][roundtrip][save-both]")
{
    SerializationFixture fixture;
    const PhysicsSnapshot snapshot = fixture.create_snapshot(3);

    const fs::path base_path = fixture.temp_dir / "snapshot_save_both";
    const fs::path json_path = fs::path(base_path.string() + ".json");
    fs::create_directories(json_path);

    const auto result = SnapshotSerializer::save_both(snapshot, base_path.string());
    REQUIRE_FALSE(result.is_success());
    REQUIRE(result.error == SerializationError::WriteError);
    REQUIRE(result.error_message.find("Cannot open JSON file for writing") != std::string::npos);

    // Binary write should still complete successfully before JSON failure is returned.
    REQUIRE(fs::exists(fs::path(base_path.string() + ".bin")));
}

TEST_CASE("Snapshot Helpers - capture and restore particle system", "[serialization][helpers]")
{
    ParticleSystem system;
    Material material_a;
    material_a.mass = 2.5f;
    material_a.restitution = 0.4f;
    material_a.friction = 0.6f;
    material_a.linear_damping = 0.03f;
    material_a.angular_damping = 0.04f;
    material_a.drag_coefficient = 0.2f;
    system.spawn(Vec3f(1.0f, 2.0f, 3.0f), Vec3f(-1.0f, 0.5f, 0.25f), material_a, 8.0f, 0.6f);
    system.particles().back().acceleration = Vec3f(0.1f, 0.2f, 0.3f);
    system.particles().back().force_accumulator = Vec3f(1.0f, 0.0f, -1.0f);

    Material material_b;
    material_b.mass = 1.0f;
    material_b.restitution = 0.2f;
    material_b.friction = 0.5f;
    material_b.linear_damping = 0.01f;
    material_b.angular_damping = 0.02f;
    material_b.drag_coefficient = 0.05f;
    system.spawn(Vec3f(-2.0f, 0.0f, 1.0f), Vec3f(0.5f, -0.5f, 1.0f), material_b, -1.0f, 0.4f);
    system.particles().back().active = false;

    const PhysicsSnapshot captured = SnapshotHelpers::capture_particle_system(system, 12, 0.2, 77);
    REQUIRE(captured.particles.size() == 1);

    ParticleSystem restored;
    REQUIRE(SnapshotHelpers::restore_particle_system(captured, restored));
    const PhysicsSnapshot recaptured = SnapshotHelpers::capture_particle_system(restored, 12, 0.2, 77);

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(captured, recaptured, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("Snapshot Helpers - capture and restore rigid body system", "[serialization][helpers][rigid-body]")
{
    RigidBodySystem system;

    auto sphere_shape = std::make_shared<SphereShape>(0.5f, Vec3f(0.1f, 0.0f, 0.0f));
    Material sphere_material;
    sphere_material.restitution = 0.6f;
    sphere_material.friction = 0.4f;
    sphere_material.linear_damping = 0.02f;
    sphere_material.angular_damping = 0.03f;
    sphere_material.drag_coefficient = 0.05f;
    const auto sphere_id = system.spawn_body(Vec3f(1.0f, 2.0f, 3.0f), Quatf(), sphere_shape, 2.0f, sphere_material);

    auto *sphere_body = system.get_body(sphere_id);
    REQUIRE(sphere_body != nullptr);
    sphere_body->velocity = Vec3f(0.5f, -0.25f, 1.0f);
    sphere_body->force_accumulator = Vec3f(0.1f, 0.2f, 0.3f);
    sphere_body->angular_velocity = Vec3f(0.0f, 1.0f, 0.0f);
    sphere_body->torque_accumulator = Vec3f(0.05f, 0.0f, -0.02f);
    sphere_body->lifetime = 7.5f;

    auto box_shape = std::make_shared<BoxShape>(Vec3f(0.25f, 0.5f, 0.75f), Vec3f(0.0f, 0.1f, 0.0f));
    Material box_material;
    box_material.restitution = 0.2f;
    box_material.friction = 0.8f;
    const auto box_id = system.spawn_body(Vec3f(-2.0f, 1.0f, 0.0f), Quatf(), box_shape, 3.0f, box_material);
    auto *box_body = system.get_body(box_id);
    REQUIRE(box_body != nullptr);
    box_body->velocity = Vec3f(-0.5f, 0.0f, 0.25f);

    const PhysicsSnapshot captured = SnapshotHelpers::capture_rigid_body_system(system, 25, 0.4, 1234);
    REQUIRE(captured.rigid_bodies.size() == 2);

    RigidBodySystem restored;
    REQUIRE(SnapshotHelpers::restore_rigid_body_system(captured, restored));
    const PhysicsSnapshot recaptured = SnapshotHelpers::capture_rigid_body_system(restored, 25, 0.4, 1234);

    REQUIRE(recaptured.rigid_bodies.size() == captured.rigid_bodies.size());
    for (size_t index = 0; index < captured.rigid_bodies.size(); ++index)
    {
        const auto &expected = captured.rigid_bodies[index];
        const auto &actual = recaptured.rigid_bodies[index];

        REQUIRE((expected.position - actual.position).length() <= 1e-6f);
        REQUIRE((expected.velocity - actual.velocity).length() <= 1e-6f);
        REQUIRE((expected.angular_velocity - actual.angular_velocity).length() <= 1e-6f);
        REQUIRE(expected.shape_type == actual.shape_type);
        REQUIRE(expected.id == actual.id);
        REQUIRE(expected.mass == Catch::Approx(actual.mass));
        REQUIRE(expected.restitution == Catch::Approx(actual.restitution));
        REQUIRE(expected.friction == Catch::Approx(actual.friction));
    }
}

TEST_CASE("Golden Serialization Fixture - current schema loads", "[serialization][golden]")
{
    const std::string golden_path = get_golden_dir() + "/serialization/particle_snapshot_v1_1.json";

    PhysicsSnapshot loaded;
    const auto result = SnapshotHelpers::load_golden(golden_path, loaded);
    REQUIRE(result.is_success());
    REQUIRE(loaded.schema_version == current_schema_version());
    REQUIRE(loaded.particles.size() == 2);
    REQUIRE(loaded.particles[0].drag_coefficient == Catch::Approx(0.15f));
}

TEST_CASE("Golden Serialization Fixture - legacy schema remains compatible", "[serialization][golden][compat]")
{
    SerializationFixture fixture;
    const std::string golden_path = get_golden_dir() + "/serialization/particle_snapshot_v1_0.json";

    PhysicsSnapshot loaded;
    const auto result = SnapshotHelpers::load_golden(golden_path, loaded);
    REQUIRE(result.is_success());
    REQUIRE(loaded.schema_version == current_schema_version());

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(
        fixture.expected_legacy_snapshot(), loaded, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("Serialization - validate golden exists", "[serialization][golden]")
{
    const std::string golden_path = get_golden_dir() + "/serialization/particle_snapshot_v1_1.json";
    REQUIRE(SnapshotHelpers::validate_golden_exists(golden_path, current_schema_version()));
}

TEST_CASE("Snapshot Helpers - update_golden emits audit event", "[serialization][golden][audit]")
{
    SerializationFixture fixture;
    const PhysicsSnapshot snapshot = fixture.create_snapshot(2);
    const std::string golden_path = (fixture.temp_dir / "audit_event_snapshot.json").string();
    std::vector<std::string> audit_events;

    ScopedSnapshotAuditSink audit_scope([&audit_events](const std::string &message)
                                        { audit_events.push_back(message); });

    const auto update_result =
        SnapshotHelpers::update_golden(snapshot, golden_path, "regenerate serialization fixture");
    REQUIRE(update_result.is_success());
    REQUIRE(audit_events.size() == 1);
    REQUIRE(audit_events[0].find("event=snapshot.update_golden") != std::string::npos);
    REQUIRE(audit_events[0].find("status=success") != std::string::npos);
    REQUIRE(audit_events[0].find("regenerate serialization fixture") != std::string::npos);
    REQUIRE(audit_events[0].find(golden_path) != std::string::npos);
}

TEST_CASE("Serialization - describe snapshot", "[serialization][util]")
{
    SerializationFixture fixture;
    const PhysicsSnapshot snapshot = fixture.create_snapshot(5);
    const std::string description = SnapshotSerializer::describe_snapshot(snapshot);

    REQUIRE(description.find("schema_version") != std::string::npos);
    REQUIRE(description.find("particles") != std::string::npos);
    REQUIRE(description.find("frame_number") != std::string::npos);
}

TEST_CASE("Golden Serialization Fixture - N-2 legacy schema migrates with defaults",
          "[serialization][golden][compat][migration]")
{
    SerializationFixture fixture;
    const std::string golden_path = get_golden_dir() + "/serialization/particle_snapshot_v0_9.json";

    PhysicsSnapshot loaded;
    const auto result = SnapshotHelpers::load_golden(golden_path, loaded);
    REQUIRE(result.is_success());
    REQUIRE(loaded.schema_version == current_schema_version());
    REQUIRE(SnapshotSerializer::can_migrate(SchemaVersion{0, 9, 0}, current_schema_version()));

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(
        fixture.expected_n2_legacy_snapshot(), loaded, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("Binary migration: format v1 -> v2", "[serialization][golden][compat][migration][binary]")
{
    SerializationFixture fixture;
    const auto expected = fixture.expected_legacy_snapshot();
    const std::string fixture_path = get_golden_dir() + "/serialization/snapshot_v1_format1.bin";

    if (!fs::exists(fixture_path))
    {
        fs::create_directories(fs::path(fixture_path).parent_path());
        const auto write_result = write_legacy_format_v1_fixture(fixture_path, expected);
        REQUIRE(write_result.is_success());
    }

    PhysicsSnapshot loaded;
    const auto load_result = SnapshotSerializer::load_binary(fixture_path, loaded);
    REQUIRE(load_result.is_success());

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(expected, loaded, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("Binary migration: legacy v2 raw fixture remains compatible",
          "[serialization][golden][compat][migration][binary]")
{
    SerializationFixture fixture;
    const auto expected = fixture.create_snapshot_with_rigid_bodies();
    const std::string fixture_path = (fixture.temp_dir / "snapshot_v2_legacy_raw.bin").string();

    const auto write_result = write_legacy_format_v2_raw_fixture(fixture_path, expected);
    REQUIRE(write_result.is_success());

    PhysicsSnapshot loaded;
    const auto load_result = SnapshotSerializer::load_binary(fixture_path, loaded);
    REQUIRE(load_result.is_success());

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(expected, loaded, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("Serialization - unsupported forward-major schema fails with clear error",
          "[serialization][schema][migration]")
{
    SerializationFixture fixture;
    const std::string file_path = (fixture.temp_dir / "unsupported_schema.json").string();

    std::ofstream file(file_path);
    REQUIRE(file.is_open());
    file << R"({
    "schema_version": { "major": 2, "minor": 0, "patch": 0 },
    "metadata": {
        "frame_number": 1,
        "simulated_time": 0.0166667,
        "timestep": 0.0166667,
        "rng_seed": 99
    },
    "particles": []
})";
    file.close();

    PhysicsSnapshot loaded;
    const auto result = SnapshotSerializer::load_json(file_path, loaded);
    REQUIRE_FALSE(result.is_success());
    REQUIRE(result.error == SerializationError::SchemaVersionMismatch);
    REQUIRE(result.error_message.find("No migration path") != std::string::npos);
}

void ensure_golden_dir(const std::string &dir)
{
    fs::create_directories(dir);
}

TEST_CASE("Serialization scene golden - particle snapshot", "[serialization][golden][scene]")
{
    const std::string golden_root = get_golden_dir() + "/serialization/scenes";
    const std::string golden_file = golden_root + "/particle_scene_gravity.json";
    ensure_golden_dir(golden_root);

    ParticleSystem system;
    Material material;
    material.mass = 1.5f;
    material.restitution = 0.25f;
    material.friction = 0.4f;
    material.linear_damping = 0.02f;
    material.angular_damping = 0.02f;
    material.drag_coefficient = 0.01f;

    system.spawn(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), material, -1.0f, 0.5f);
    system.spawn(Vec3f(-1.0f, 3.0f, 0.0f), Vec3f(0.5f, -0.2f, 0.0f), material, -1.0f, 0.4f);

    const float dt = 1.0f / 120.0f;
    constexpr int frame_count = 120;
    for (int frame = 0; frame < frame_count; ++frame)
    {
        system.update(dt);
    }

    const PhysicsSnapshot current_snapshot =
        SnapshotHelpers::capture_particle_system(system, frame_count, frame_count * dt, 20260310);

#ifdef GOLDEN_CAPTURE_MODE
    const auto save_result = SnapshotHelpers::save_golden(current_snapshot, golden_file);
    INFO(save_result.to_string());
    REQUIRE(save_result.is_success());
    SUCCEED("Particle scene golden captured");
#else
    PhysicsSnapshot golden_snapshot;
    const auto load_result = SnapshotHelpers::load_golden(golden_file, golden_snapshot);
    REQUIRE(load_result.is_success());

    std::string diff_report;
    REQUIRE(
        SnapshotHelpers::snapshots_equal_with_tolerance(golden_snapshot, current_snapshot, 1e-6f, 1e-6f, &diff_report));
#endif
}

TEST_CASE("Serialization scene golden - rigid body snapshot", "[serialization][golden][scene][rigid-body]")
{
    const std::string golden_root = get_golden_dir() + "/serialization/scenes";
    const std::string golden_file = golden_root + "/rigid_scene_pair.json";
    ensure_golden_dir(golden_root);

    RigidBodySystem system;
    auto sphere_shape = std::make_shared<SphereShape>(0.45f, Vec3f(0.0f));
    auto box_shape = std::make_shared<BoxShape>(Vec3f(0.4f, 0.2f, 0.3f), Vec3f(0.0f, 0.05f, 0.0f));

    Material sphere_material;
    sphere_material.restitution = 0.5f;
    sphere_material.friction = 0.3f;

    Material box_material;
    box_material.restitution = 0.2f;
    box_material.friction = 0.7f;

    const auto sphere_id = system.spawn_body(Vec3f(-1.0f, 2.0f, 0.0f), Quatf(), sphere_shape, 2.0f, sphere_material);
    const auto box_id = system.spawn_body(Vec3f(1.0f, 1.0f, 0.0f), Quatf(), box_shape, 3.5f, box_material);

    auto *sphere_body = system.get_body(sphere_id);
    auto *box_body = system.get_body(box_id);
    REQUIRE(sphere_body != nullptr);
    REQUIRE(box_body != nullptr);
    sphere_body->velocity = Vec3f(1.0f, -0.25f, 0.0f);
    sphere_body->angular_velocity = Vec3f(0.0f, 1.0f, 0.1f);
    box_body->velocity = Vec3f(-0.5f, 0.1f, 0.0f);
    box_body->angular_velocity = Vec3f(0.0f, -0.5f, 0.0f);

    const float dt = 1.0f / 120.0f;
    constexpr int frame_count = 90;
    for (int frame = 0; frame < frame_count; ++frame)
    {
        system.update(dt);
    }

    const PhysicsSnapshot current_snapshot =
        SnapshotHelpers::capture_rigid_body_system(system, frame_count, frame_count * dt, 20260310);

#ifdef GOLDEN_CAPTURE_MODE
    const auto save_result = SnapshotHelpers::save_golden(current_snapshot, golden_file);
    INFO(save_result.to_string());
    REQUIRE(save_result.is_success());
    SUCCEED("Rigid body scene golden captured");
#else
    PhysicsSnapshot golden_snapshot;
    const auto load_result = SnapshotHelpers::load_golden(golden_file, golden_snapshot);
    REQUIRE(load_result.is_success());

    REQUIRE(golden_snapshot.frame_number == current_snapshot.frame_number);
    REQUIRE(golden_snapshot.rigid_bodies.size() == current_snapshot.rigid_bodies.size());
    for (size_t index = 0; index < golden_snapshot.rigid_bodies.size(); ++index)
    {
        const auto &expected = golden_snapshot.rigid_bodies[index];
        const auto &actual = current_snapshot.rigid_bodies[index];

        REQUIRE((expected.position - actual.position).length() <= 1e-6f);
        REQUIRE((expected.velocity - actual.velocity).length() <= 1e-6f);
        REQUIRE((expected.angular_velocity - actual.angular_velocity).length() <= 1e-6f);
        REQUIRE(expected.shape_type == actual.shape_type);
        REQUIRE(expected.id == actual.id);
        REQUIRE(expected.mass == Catch::Approx(actual.mass));
        REQUIRE(expected.restitution == Catch::Approx(actual.restitution));
        REQUIRE(expected.friction == Catch::Approx(actual.friction));
    }
#endif
}
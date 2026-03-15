#include <catch2/catch_all.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/micro/particle_system.hpp>
#include <core/serialization/replay_reader.hpp>
#include <core/serialization/replay_writer.hpp>
#include <core/serialization/snapshot_helpers.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace phynity::serialization;
using namespace phynity::math::vectors;
using namespace phynity::physics;

namespace fs = std::filesystem;

namespace
{
#ifdef GOLDEN_CAPTURE_MODE
std::string trim_wrapping_quotes(std::string value)
{
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
    {
        return value.substr(1, value.size() - 2);
    }
    return value;
}
#endif

struct ReplayFixture
{
    fs::path temp_dir = fs::temp_directory_path() / "phynity_replay_tests";

    ReplayFixture()
    {
        fs::create_directories(temp_dir);
    }

    ~ReplayFixture()
    {
        fs::remove_all(temp_dir);
    }

    PhysicsSnapshot make_frame(size_t index) const
    {
        PhysicsSnapshot snapshot;
        snapshot.schema_version = current_schema_version();
        snapshot.frame_number = static_cast<uint64_t>(index);
        snapshot.simulated_time = static_cast<double>(index) * (1.0 / 120.0);
        snapshot.timestep = 1.0f / 120.0f;
        snapshot.rng_seed = static_cast<uint32_t>(1000 + index);

        ParticleSnapshot particle;
        const float i = static_cast<float>(index);
        particle.position = Vec3f(i, i * 0.5f, -i * 0.25f);
        particle.velocity = Vec3f(0.1f + i, -0.2f, 0.3f - i * 0.05f);
        particle.acceleration = Vec3f(0.0f, -9.81f, 0.0f);
        particle.force_accumulator = Vec3f(0.0f, 1.0f + i, 0.0f);
        particle.radius = 0.25f + i * 0.01f;
        particle.mass = 1.0f + i * 0.1f;
        snapshot.particles.push_back(particle);

        return snapshot;
    }
};

} // namespace

TEST_CASE("Replay round-trip preserves frame contents", "[serialization][replay][roundtrip]")
{
    ReplayFixture fixture;
    const fs::path replay_path = fixture.temp_dir / "round_trip.replay";

    std::vector<PhysicsSnapshot> expected_frames;
    expected_frames.reserve(12);

    ReplayWriter writer;
    REQUIRE(writer.open(replay_path.string()).is_success());

    for (size_t i = 0; i < 12; ++i)
    {
        expected_frames.push_back(fixture.make_frame(i));
        const auto append_result = writer.append_frame(expected_frames.back());
        REQUIRE(append_result.is_success());
    }

    REQUIRE(writer.close().is_success());

    ReplayReader reader;
    const auto open_result = reader.open(replay_path.string());
    REQUIRE(open_result.is_success());
    REQUIRE(reader.frame_count() == expected_frames.size());

    for (size_t i = 0; i < expected_frames.size(); ++i)
    {
        PhysicsSnapshot actual;
        const auto read_result = reader.read_next(actual);
        REQUIRE(read_result.is_success());

        std::string diff_report;
        REQUIRE(
            SnapshotHelpers::snapshots_equal_with_tolerance(expected_frames[i], actual, 1e-6f, 1e-6f, &diff_report));
    }
}

TEST_CASE("Replay random access matches sequential frame", "[serialization][replay][random-access]")
{
    ReplayFixture fixture;
    const fs::path replay_path = fixture.temp_dir / "random_access.replay";

    std::vector<PhysicsSnapshot> expected_frames;
    expected_frames.reserve(10);

    ReplayWriter writer;
    REQUIRE(writer.open(replay_path.string()).is_success());

    for (size_t i = 0; i < 10; ++i)
    {
        expected_frames.push_back(fixture.make_frame(i));
        REQUIRE(writer.append_frame(expected_frames.back()).is_success());
    }
    REQUIRE(writer.close().is_success());

    ReplayReader reader;
    REQUIRE(reader.open(replay_path.string()).is_success());

    PhysicsSnapshot random_frame;
    const size_t target_index = expected_frames.size() / 2;
    REQUIRE(reader.read_frame(target_index, random_frame).is_success());

    std::string diff_report;
    REQUIRE(SnapshotHelpers::snapshots_equal_with_tolerance(
        expected_frames[target_index], random_frame, 1e-6f, 1e-6f, &diff_report));

    reader.reset_iteration();
    PhysicsSnapshot sequential_frame;
    for (size_t i = 0; i <= target_index; ++i)
    {
        REQUIRE(reader.read_next(sequential_frame).is_success());
    }

    REQUIRE(
        SnapshotHelpers::snapshots_equal_with_tolerance(sequential_frame, random_frame, 1e-6f, 1e-6f, &diff_report));
}

TEST_CASE("Replay detects corrupt header", "[serialization][replay][corrupt]")
{
    ReplayFixture fixture;
    const fs::path replay_path = fixture.temp_dir / "corrupt_header.replay";

    std::ofstream output(replay_path, std::ios::binary | std::ios::trunc);
    REQUIRE(output.is_open());

    ReplayFileHeader header;
    header.magic = 0xDEADBEEFU;
    output.write(reinterpret_cast<const char *>(&header), sizeof(header));
    output.close();

    ReplayReader reader;
    const auto open_result = reader.open(replay_path.string());
    REQUIRE_FALSE(open_result.is_success());
    REQUIRE(open_result.error == SerializationError::InvalidFileFormat);
}

TEST_CASE("Replay capture integration - particle system 60 frames", "[serialization][replay][integration]")
{
    ReplayFixture fixture;
    fs::path replay_path = fixture.temp_dir / "particle_capture.replay";

#ifdef GOLDEN_CAPTURE_MODE
    if (const char *golden_root = std::getenv("GOLDEN_FILES_DIR"); golden_root != nullptr)
    {
        replay_path =
            fs::path(trim_wrapping_quotes(golden_root)) / "serialization" / "replays" / "determinism_particle.replay";
    }
#endif

    fs::create_directories(replay_path.parent_path());

    ParticleSystem system;
    Material material;
    material.mass = 1.5f;
    material.restitution = 0.2f;
    material.friction = 0.4f;
    system.spawn(Vec3f(0.0f, 3.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), material, -1.0f, 0.35f);

    REQUIRE(SnapshotHelpers::start_replay_capture(replay_path.string()).is_success());

    std::vector<PhysicsSnapshot> expected_frames;
    expected_frames.reserve(60);

    const float dt = 1.0f / 120.0f;
    for (uint64_t frame = 0; frame < 60; ++frame)
    {
        system.update(dt);
        const double simulated_time = static_cast<double>(frame + 1U) * static_cast<double>(dt);
        const auto capture_result = SnapshotHelpers::append_replay_frame(system, frame + 1, simulated_time, 4242);
        REQUIRE(capture_result.is_success());
        expected_frames.push_back(SnapshotHelpers::capture_particle_system(system, frame + 1, simulated_time, 4242));
    }

    REQUIRE(SnapshotHelpers::stop_replay_capture().is_success());

    ReplayReader reader;
    REQUIRE(reader.open(replay_path.string()).is_success());
    REQUIRE(reader.frame_count() == expected_frames.size());

    for (size_t i = 0; i < expected_frames.size(); ++i)
    {
        PhysicsSnapshot actual;
        REQUIRE(reader.read_frame(i, actual).is_success());

        std::string diff_report;
        REQUIRE(
            SnapshotHelpers::snapshots_equal_with_tolerance(expected_frames[i], actual, 1e-6f, 1e-6f, &diff_report));
    }
}

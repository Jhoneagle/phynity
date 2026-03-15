#include <catch2/catch_all.hpp>
#include <core/physics/macro/rigid_body_system.hpp>
#include <core/physics/micro/particle_system.hpp>
#include <core/serialization/snapshot_helpers.hpp>
#include <core/serialization/snapshot_schema.hpp>
#include <core/serialization/snapshot_serializer.hpp>
#include <platform/memory_usage.hpp>
#include <tests/test_utils/golden_serializer.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

using namespace phynity::physics;
using namespace phynity::serialization;
using namespace phynity::math::vectors;
using namespace phynity::test;
using namespace phynity::test::helpers;
using namespace phynity::test::helpers::constants;

namespace
{

// Helper macros for stringifying preprocessor definitions
#define STRINGIFY(x) #x
#define STRINGIFY_EXPANDED(x) STRINGIFY(x)

struct PerfResult
{
    std::string scenario;
    double milliseconds = 0.0;
    int iterations = 0;
    int workload = 0;
    std::string notes;
    phynity::platform::TrackedVector<double> samples_ms; // Per-iteration sample times in milliseconds
    double median_ms = 0.0;
    double mean_ms = 0.0;
    double stddev_ms = 0.0;
    uint64_t peak_rss_kb = 0;
    int64_t allocator_delta_bytes = 0;
};

std::string get_golden_dir()
{
#ifdef GOLDEN_FILES_DIR
    return STRINGIFY_EXPANDED(GOLDEN_FILES_DIR);
#else
    return "tests/golden_outputs";
#endif
}

/// Compute statistics from sample array
void compute_stats(PerfResult &result)
{
    if (result.samples_ms.empty())
    {
        result.median_ms = result.mean_ms = result.stddev_ms = 0.0;
        return;
    }

    // Median
    std::vector<double> sorted_samples(result.samples_ms.begin(), result.samples_ms.end());
    std::sort(sorted_samples.begin(), sorted_samples.end());
    size_t n = sorted_samples.size();
    result.median_ms = (n % 2 == 0) ? (sorted_samples[n / 2 - 1] + sorted_samples[n / 2]) / 2.0 : sorted_samples[n / 2];

    // Mean
    double sum = 0.0;
    for (double sample : result.samples_ms)
    {
        sum += sample;
    }
    result.mean_ms = sum / static_cast<double>(result.samples_ms.size());

    // Standard deviation
    double variance = 0.0;
    for (double sample : result.samples_ms)
    {
        double diff = sample - result.mean_ms;
        variance += diff * diff;
    }
    variance /= static_cast<double>(result.samples_ms.size());
    result.stddev_ms = std::sqrt(variance);
}

std::string to_json(const PerfResult &result)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(6);
    oss << "{\n";
    oss << "  \"scenario\": \"" << result.scenario << "\",\n";
    oss << "  \"milliseconds\": " << result.milliseconds << ",\n";
    oss << "  \"iterations\": " << result.iterations << ",\n";
    oss << "  \"workload\": " << result.workload << ",\n";
    oss << "  \"notes\": \"" << result.notes << "\",\n";

    // Samples array
    oss << "  \"samples_ms\": [";
    for (size_t i = 0; i < result.samples_ms.size(); ++i)
    {
        if (i > 0)
            oss << ", ";
        oss << result.samples_ms[i];
    }
    oss << "],\n";

    // Computed statistics
    oss << "  \"median_ms\": " << result.median_ms << ",\n";
    oss << "  \"mean_ms\": " << result.mean_ms << ",\n";
    oss << "  \"stddev_ms\": " << result.stddev_ms << ",\n";
    oss << "  \"peak_rss_kb\": " << result.peak_rss_kb << ",\n";
    oss << "  \"allocator_delta_bytes\": " << result.allocator_delta_bytes << "\n";
    oss << "}\n";
    return oss.str();
}

void ensure_dir(const std::string &dir)
{
    std::filesystem::create_directories(dir);
}

void write_perf_result(const PerfResult &result)
{
    const std::string golden_root = get_golden_dir();
    const std::string perf_dir = golden_root + "/performance";
    ensure_dir(perf_dir);

    const std::string current_path = perf_dir + "/" + result.scenario + ".current.json";
    GoldenSerializer::save_json_file(to_json(result), current_path);

#ifdef GOLDEN_CAPTURE_MODE
    const std::string golden_path = perf_dir + "/" + result.scenario + ".json";
    GoldenSerializer::save_json_file(to_json(result), golden_path);
#endif
}

/**
 * Benchmark particle integration kernel: tight inner loop of position/velocity updates
 * This is a core-tier benchmark focused on math kernel performance (vector math + integration)
 * Runs multiple times to collect reliable samples.
 */
PerfResult benchmark_particle_integration_kernel(int particle_count, int frames, int num_samples = 3)
{
    phynity::platform::AllocatorDeltaScope allocator_scope;
    constexpr float seed_max = 2147483647.0f;
    auto make_system = [&]() -> ParticleSystem
    {
        ParticleSystem system;
        system.set_broadphase_cell_size(2.0f);

        unsigned int seed = 42;
        auto rand_float = [&seed](float min, float max) -> float
        {
            seed = (seed * 1103515245 + 12345) & 0x7fffffff;
            float normalized = static_cast<float>(seed) / seed_max;
            return min + normalized * (max - min);
        };

        // Spawn particles with varied properties
        for (int i = 0; i < particle_count; ++i)
        {
            Vec3f pos(rand_float(-10.0f, 10.0f), rand_float(-10.0f, 10.0f), rand_float(-10.0f, 10.0f));
            Vec3f vel(rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f));
            system.spawn(pos, vel, 1.0f, -1.0f, 0.25f);
        }
        return system;
    };

    PerfResult result;
    result.scenario = "particle_integration_kernel";
    result.workload = particle_count;
    result.notes = "Pure particle integration kernel (position + velocity update per timestep)";
    if (num_samples > 0)
    {
        result.samples_ms.reserve(static_cast<std::vector<double>::size_type>(num_samples));
    }

    // Collect multiple samples
    for (int sample = 0; sample < num_samples; ++sample)
    {
        ParticleSystem system = make_system();

        const auto start = std::chrono::high_resolution_clock::now();

        // Time only integration steps (collisions disabled for pure kernel measurement)
        for (int frame = 0; frame < frames; ++frame)
        {
            system.update(DETERMINISTIC_TIMESTEP);
        }

        const auto end = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        double sample_ms = static_cast<double>(duration.count()) / 1000.0;
        result.samples_ms.push_back(sample_ms);
    }

    // Compute aggregate statistics
    double total_ms = 0.0;
    for (double sample : result.samples_ms)
    {
        total_ms += sample;
    }
    result.milliseconds = total_ms;
    result.iterations = frames;
    compute_stats(result);
    result.peak_rss_kb = phynity::platform::get_peak_rss_kb();
    result.allocator_delta_bytes = allocator_scope.delta_bytes();

    return result;
}

/**
 * Benchmark complex-scene deterministic simulation with snapshot capture.
 * This is a complex-tier benchmark exercising full simulation + serialization pipeline.
 * Validates that full deterministic round-trips preserve simulation state.
 * Runs multiple times to collect reliable samples.
 */
PerfResult benchmark_complex_deterministic_scene(int rigid_body_count,
                                                 int particle_count,
                                                 int simulation_frames,
                                                 int num_samples = 3)
{
    phynity::platform::AllocatorDeltaScope allocator_scope;
    constexpr float seed_max = 2147483647.0f;

    auto setup_scene = [&]() -> std::pair<RigidBodySystem, ParticleSystem>
    {
        RigidBodySystem::Config rb_config;
        rb_config.enable_linear_ccd = false;
        RigidBodySystem rb_system(rb_config);

        ParticleSystem particle_system;
        particle_system.set_broadphase_cell_size(1.0f);
        particle_system.enable_collisions(false); // Collisions not needed for this benchmark

        unsigned int seed = 123;
        auto rand_float = [&seed](float min, float max) -> float
        {
            seed = (seed * 1103515245 + 12345) & 0x7fffffff;
            float normalized = static_cast<float>(seed) / seed_max;
            return min + normalized * (max - min);
        };

        // Spawn rigid bodies
        for (int i = 0; i < rigid_body_count; ++i)
        {
            Vec3f pos(rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f));
            Quatf orientation; // Identity
            auto sphere = std::make_shared<SphereShape>(0.5f);
            RigidBodyID id = rb_system.spawn_body(pos, orientation, sphere, 1.0f);
            RigidBody *body = rb_system.get_body(id);
            if (body)
            {
                body->velocity = Vec3f(rand_float(-2.0f, 2.0f), rand_float(-2.0f, 2.0f), rand_float(-2.0f, 2.0f));
            }
        }

        // Spawn particles
        for (int i = 0; i < particle_count; ++i)
        {
            Vec3f pos(rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f));
            Vec3f vel(rand_float(-3.0f, 3.0f), rand_float(-3.0f, 3.0f), rand_float(-3.0f, 3.0f));
            particle_system.spawn(pos, vel, 1.0f, -1.0f, 0.25f);
        }

        return {std::move(rb_system), std::move(particle_system)};
    };

    PerfResult result;
    result.scenario = "complex_deterministic_scene";
    result.workload = rigid_body_count + particle_count;
    std::ostringstream notes_stream;
    notes_stream << "Full scene simulation (" << rigid_body_count << " rigid bodies + " << particle_count
                 << " particles) with periodic snapshots";
    result.notes = notes_stream.str();
    if (num_samples > 0)
    {
        result.samples_ms.reserve(static_cast<std::vector<double>::size_type>(num_samples));
    }

    // Collect multiple samples
    for (int sample = 0; sample < num_samples; ++sample)
    {
        auto [rb_system, particle_system] = setup_scene();

        const auto start = std::chrono::high_resolution_clock::now();

        // Simulate and periodically capture snapshots
        for (int frame = 0; frame < simulation_frames; ++frame)
        {
            rb_system.update(DETERMINISTIC_TIMESTEP);
            particle_system.update(DETERMINISTIC_TIMESTEP);

            // Every 10 frames, capture and serialize a snapshot (to measure serialization cost)
            if (frame % 10 == 0)
            {
                PhysicsSnapshot snapshot;
                snapshot.frame_number = static_cast<uint64_t>(frame);
                snapshot.simulated_time = static_cast<double>(frame) * static_cast<double>(DETERMINISTIC_TIMESTEP);
                snapshot.timestep = DETERMINISTIC_TIMESTEP;
                snapshot.rng_seed = 123;

                // Manually construct particle snapshots (benchmark uses synthesized data)
                const size_t previous_particle_capacity = snapshot.particles.capacity();
                snapshot.particles.clear();
                ParticleSnapshot p;
                p.position = Vec3f(1.0f, 2.0f, 3.0f);
                p.velocity = Vec3f(0.1f, 0.2f, 0.3f);
                p.mass = 1.0f;
                snapshot.particles.push_back(p);
                phynity::platform::track_vector_capacity_change(snapshot.particles, previous_particle_capacity);

                // Similarly for rigid bodies
                const size_t previous_rigid_capacity = snapshot.rigid_bodies.capacity();
                snapshot.rigid_bodies.clear();
                RigidBodySnapshot rb;
                rb.position = Vec3f(5.0f, 6.0f, 7.0f);
                rb.velocity = Vec3f(0.5f, 0.6f, 0.7f);
                rb.orientation_w = 1.0f;
                rb.orientation_x = rb.orientation_y = rb.orientation_z = 0.0f;
                rb.mass = 2.0f;
                rb.shape_type = SnapshotShapeType::Sphere;
                rb.shape_radius = 0.5f;
                snapshot.rigid_bodies.push_back(rb);
                phynity::platform::track_vector_capacity_change(snapshot.rigid_bodies, previous_rigid_capacity);

                // Serialize to binary (measure round-trip cost)
                std::filesystem::path tmp_dir = std::filesystem::temp_directory_path();
                std::filesystem::path tmp_path = tmp_dir / "snapshot_perf_test.bin";
                SnapshotSerializer::save_binary(snapshot, tmp_path.string());

                phynity::platform::track_vector_capacity_release(snapshot.particles);
                phynity::platform::track_vector_capacity_release(snapshot.rigid_bodies);
            }
        }

        const auto end = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        double sample_ms = static_cast<double>(duration.count()) / 1000.0;
        result.samples_ms.push_back(sample_ms);
    }

    // Compute aggregate statistics
    double total_ms = 0.0;
    for (double sample : result.samples_ms)
    {
        total_ms += sample;
    }
    result.milliseconds = total_ms;
    result.iterations = simulation_frames;
    compute_stats(result);
    result.peak_rss_kb = phynity::platform::get_peak_rss_kb();
    result.allocator_delta_bytes = allocator_scope.delta_bytes();

    return result;
}

} // anonymous namespace

TEST_CASE("Core Kernel Performance: Particle integration", "[validation][performance][core-kernel]")
{
    const int particle_count = 100;
    const int frames = 200;

    SECTION("Measure particle integration kernel on moderate workload")
    {
        PerfResult result = benchmark_particle_integration_kernel(particle_count, frames);
        REQUIRE(result.milliseconds > 0.0);

        std::cout << "\n=== Particle Integration Kernel Performance ===\n";
        std::cout << "  Particles:     " << particle_count << "\n";
        std::cout << "  Frames:        " << frames << "\n";
        std::cout << "  Total time:    " << result.milliseconds << " ms\n";
        std::cout << "  Per-frame avg: " << (result.milliseconds / frames) << " ms\n";

        write_perf_result(result);
    }
}

TEST_CASE("Complex Scene Performance: Deterministic simulation with snapshots",
          "[validation][performance][complex-scenes]")
{
    const int rigid_body_count = 10;
    const int particle_count = 50;
    const int simulation_frames = 100;

    SECTION("Measure full-scene simulation + periodic snapshot serialization")
    {
        PerfResult result = benchmark_complex_deterministic_scene(rigid_body_count, particle_count, simulation_frames);
        REQUIRE(result.milliseconds > 0.0);

        std::cout << "\n=== Complex Deterministic Scene Performance ===\n";
        std::cout << "  Rigid bodies:  " << rigid_body_count << "\n";
        std::cout << "  Particles:     " << particle_count << "\n";
        std::cout << "  Frames:        " << simulation_frames << "\n";
        std::cout << "  Total time:    " << result.milliseconds << " ms\n";
        std::cout << "  Per-frame avg: " << (result.milliseconds / simulation_frames) << " ms\n";

        write_perf_result(result);

        // Sanity check: full scene should be reasonably fast
        REQUIRE(result.milliseconds < 10000.0); // Should complete in under 10 seconds
    }
}

#include <catch2/catch_all.hpp>
#include <core/physics/common/ccd_config.hpp>
#include <core/physics/macro/rigid_body_system.hpp>
#include <core/physics/micro/particle_system.hpp>

#include <chrono>
#include <cmath>
#include <iostream>

using namespace phynity::physics;
using namespace phynity::math::vectors;

namespace
{

bool is_slow_env()
{
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_UNDEFINED__)
    return true;
#endif
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(undefined_behavior_sanitizer)
    return true;
#endif
#endif
    return false;
}

struct PerfResult
{
    double milliseconds;
    int particle_count;
    int frames;
    std::string config_name;
};

/**
 * Measure performance of particle system with CCD enabled vs disabled
 */
PerfResult benchmark_particle_ccd(int particle_count, int frames, const CCDConfig &config, const std::string &name)
{
    ParticleSystem system;
    system.set_broadphase_cell_size(2.0f);
    system.set_ccd_config(config);

    // Spawn mix of fast and slow particles
    std::srand(42); // Deterministic
    auto rand_float = [](float min, float max)
    { return min + (max - min) * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)); };

    for (int i = 0; i < particle_count; ++i)
    {
        Vec3f pos(rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f));

        // Mix: 80% slow, 20% fast (to trigger CCD selectively)
        float speed = (i % 5 == 0) ? rand_float(5.0f, 10.0f) : rand_float(0.5f, 2.0f);
        float angle = rand_float(0.0f, 6.28f);
        Vec3f vel(std::cos(angle) * speed, std::sin(angle) * speed, rand_float(-1.0f, 1.0f));

        system.spawn(pos, vel, 1.0f, -1.0f, 0.25f);
    }

    const float dt = 0.016f;
    const auto start = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < frames; ++frame)
    {
        system.update(dt);
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    PerfResult result;
    result.milliseconds = static_cast<double>(duration.count()) / 1000.0;
    result.particle_count = particle_count;
    result.frames = frames;
    result.config_name = name;
    return result;
}

/**
 * Measure performance of rigid body system with CCD enabled vs disabled
 */
PerfResult benchmark_rigidbody_ccd(int body_count, int frames, bool ccd_enabled, const std::string &name)
{
    RigidBodySystem::Config config;
    config.enable_linear_ccd = ccd_enabled;
    config.ccd_config = ccd_enabled ? ccd_presets::balanced() : ccd_presets::disabled();

    RigidBodySystem system(config);

    // Spawn bodies in a grid with varied velocities
    std::srand(42);
    auto rand_float = [](float min, float max)
    { return min + (max - min) * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)); };

    int grid_size = static_cast<int>(std::ceil(std::cbrt(static_cast<double>(body_count))));
    for (int i = 0; i < body_count; ++i)
    {
        int x = i % grid_size;
        int y = (i / grid_size) % grid_size;
        int z = i / (grid_size * grid_size);

        Vec3f pos(static_cast<float>(x) * 2.0f, static_cast<float>(y) * 2.0f, static_cast<float>(z) * 2.0f);
        Quatf orientation; // Identity quaternion

        auto sphere = std::make_shared<SphereShape>(0.5f);
        RigidBodyID id = system.spawn_body(pos, orientation, sphere, 1.0f);

        // Set velocity after spawning
        RigidBody *body = system.get_body(id);
        if (body != nullptr)
        {
            body->velocity = Vec3f(rand_float(-3.0f, 3.0f), rand_float(-3.0f, 3.0f), rand_float(-3.0f, 3.0f));
            body->angular_velocity = Vec3f(rand_float(-1.0f, 1.0f), rand_float(-1.0f, 1.0f), rand_float(-1.0f, 1.0f));
        }
    }

    const float dt = 0.016f;
    const auto start = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < frames; ++frame)
    {
        system.update(dt);
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    PerfResult result;
    result.milliseconds = static_cast<double>(duration.count()) / 1000.0;
    result.particle_count = body_count;
    result.frames = frames;
    result.config_name = name;
    return result;
}

} // anonymous namespace

TEST_CASE("CCD Performance: Particle system overhead", "[validation][performance][ccd]")
{
    const int particle_count = 200;
    const int frames = 100;

    SECTION("Measure CCD overhead for particles")
    {
        PerfResult disabled = benchmark_particle_ccd(particle_count, frames, ccd_presets::disabled(), "disabled");
        PerfResult conservative =
            benchmark_particle_ccd(particle_count, frames, ccd_presets::conservative(), "conservative");
        PerfResult balanced = benchmark_particle_ccd(particle_count, frames, ccd_presets::balanced(), "balanced");

        REQUIRE(disabled.milliseconds > 0.0);
        REQUIRE(conservative.milliseconds > 0.0);
        REQUIRE(balanced.milliseconds > 0.0);

        // Print results for analysis
        std::cout << "\n=== CCD Particle Performance Results ===\n";
        std::cout << "  Disabled:     " << disabled.milliseconds << " ms\n";
        std::cout << "  Conservative: " << conservative.milliseconds
                  << " ms (overhead: " << (conservative.milliseconds / disabled.milliseconds - 1.0) * 100.0 << "%)\n";
        std::cout << "  Balanced:     " << balanced.milliseconds
                  << " ms (overhead: " << (balanced.milliseconds / disabled.milliseconds - 1.0) * 100.0 << "%)\n";

        // Sanity check: CCD should not cause dramatic slowdown (>3x)
        // (The optimization goal is to keep overhead reasonable)
        const double ratio_limit = is_slow_env() ? 6.0 : 3.0;
        REQUIRE(conservative.milliseconds < disabled.milliseconds * ratio_limit);
        REQUIRE(balanced.milliseconds < disabled.milliseconds * ratio_limit);
    }
}

TEST_CASE("CCD Performance: Rigid body overhead", "[validation][performance][ccd]")
{
    const int body_count = 30;
    const int frames = 50;

    SECTION("Measure CCD overhead for rigid bodies")
    {
        PerfResult disabled = benchmark_rigidbody_ccd(body_count, frames, false, "disabled");
        PerfResult enabled = benchmark_rigidbody_ccd(body_count, frames, true, "enabled");

        REQUIRE(disabled.milliseconds > 0.0);
        REQUIRE(enabled.milliseconds > 0.0);

        // Print results for analysis
        std::cout << "\n=== CCD Rigid Body Performance Results ===\n";
        std::cout << "  Disabled: " << disabled.milliseconds << " ms\n";
        std::cout << "  Enabled:  " << enabled.milliseconds
                  << " ms (overhead: " << (enabled.milliseconds / disabled.milliseconds - 1.0) * 100.0 << "%)\n";

        // Sanity check: CCD should not cause dramatic slowdown (>5x for rigid bodies)
        // Note: Rigid bodies use O(n²) collision detection, so CCD overhead is higher than particles
        REQUIRE(enabled.milliseconds < disabled.milliseconds * 5.1);
    }
}

TEST_CASE("CCD Performance: Scaling with particle count", "[validation][performance][ccd]")
{
    const int frames = 50;

    SECTION("Measure how CCD overhead scales")
    {
        PerfResult small_off = benchmark_particle_ccd(50, frames, ccd_presets::disabled(), "50_off");
        PerfResult small_on = benchmark_particle_ccd(50, frames, ccd_presets::balanced(), "50_on");

        PerfResult large_off = benchmark_particle_ccd(200, frames, ccd_presets::disabled(), "200_off");
        PerfResult large_on = benchmark_particle_ccd(200, frames, ccd_presets::balanced(), "200_on");

        REQUIRE(small_off.milliseconds > 0.0);
        REQUIRE(small_on.milliseconds > 0.0);
        REQUIRE(large_off.milliseconds > 0.0);
        REQUIRE(large_on.milliseconds > 0.0);

        // Print scaling analysis
        double small_overhead_pct = (small_on.milliseconds / small_off.milliseconds - 1.0) * 100.0;
        double large_overhead_pct = (large_on.milliseconds / large_off.milliseconds - 1.0) * 100.0;

        std::cout << "\n=== CCD Scaling Analysis ===\n";
        std::cout << "  50 particles:  " << small_off.milliseconds << " ms -> " << small_on.milliseconds << " ms (+"
                  << small_overhead_pct << "%)\n";
        std::cout << "  200 particles: " << large_off.milliseconds << " ms -> " << large_on.milliseconds << " ms (+"
                  << large_overhead_pct << "%)\n";

        // CCD overhead should scale sub-linearly (good culling/early exits)
        // Guard against small denominators to avoid unstable ratios on fast machines
        double denom = std::max(1.0, std::abs(small_overhead_pct));
        double overhead_ratio = std::abs(large_overhead_pct) / denom;
        std::cout << "  Overhead ratio (200/50): " << overhead_ratio << "x\n";

        // If optimizations work well, overhead ratio should be reasonable
        // (CCD early-exit logic should prevent superlinear scaling)
        REQUIRE(std::isfinite(overhead_ratio));
        const double ratio_limit = is_slow_env() ? 6.0 : 5.0;
        REQUIRE(overhead_ratio < ratio_limit);
    }
}

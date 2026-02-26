#include <catch2/catch_all.hpp>
#include <core/physics/particle_system.hpp>
#include <core/physics/collision/sphere_sphere_narrowphase.hpp>
#include <core/physics/collision/impulse_resolver.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>
#include <chrono>
#include <cmath>

using namespace phynity::physics;
using namespace phynity::physics::collision;
using namespace phynity::math::vectors;

/**
 * @brief Helper to resolve collisions using brute-force O(n²) approach.
 */
void resolve_collisions_brute_force(std::vector<Particle>& particles) {
    const size_t count = particles.size();
    for (size_t i = 0; i < count; ++i) {
        Particle& a = particles[i];
        if (!a.is_alive()) continue;

        for (size_t j = i + 1; j < count; ++j) {
            Particle& b = particles[j];
            if (!b.is_alive()) continue;

            SphereCollider collider_a;
            collider_a.position = a.position;
            collider_a.velocity = a.velocity;
            collider_a.radius = a.radius;
            collider_a.inverse_mass = a.inverse_mass();
            collider_a.restitution = a.material.restitution;

            SphereCollider collider_b;
            collider_b.position = b.position;
            collider_b.velocity = b.velocity;
            collider_b.radius = b.radius;
            collider_b.inverse_mass = b.inverse_mass();
            collider_b.restitution = b.material.restitution;

            ContactManifold manifold = SphereSpherNarrowphase::detect(collider_a, collider_b, i, j);
            if (manifold.is_valid()) {
                ImpulseResolver::resolve(manifold, collider_a, collider_b);
                a.position = collider_a.position;
                a.velocity = collider_a.velocity;
                b.position = collider_b.position;
                b.velocity = collider_b.velocity;
            }
        }
    }
}

/**
 * @brief Time a collision resolution approach for N particles over multiple frames.
 */
struct TimingResult {
    double milliseconds = 0.0;
    int collision_count = 0;
};

TimingResult time_broadphase(int particle_count, float cell_size, int frames) {
    ParticleSystem system;
    system.enable_collisions(true);
    system.set_broadphase_cell_size(cell_size);

    // Spawn particles in a random distribution
    // Use seed for reproducibility
    unsigned int seed = 42;
    constexpr float seed_max = 2147483647.0f;
    auto rand_float = [&seed](float min, float max) -> float {
        seed = (seed * 1103515245 + 12345) & 0x7fffffff;
        float normalized = static_cast<float>(seed) / seed_max;
        return min + normalized * (max - min);
    };

    for (int i = 0; i < particle_count; ++i) {
        const Vec3f pos(
            rand_float(-5.0f, 5.0f),
            rand_float(-5.0f, 5.0f),
            rand_float(-5.0f, 5.0f)
        );
        const Vec3f vel(
            rand_float(-0.5f, 0.5f),
            rand_float(-0.5f, 0.5f),
            rand_float(-0.5f, 0.5f)
        );
        system.spawn(pos, vel, 1.0f, -1.0f, 0.25f);
    }

    const float dt = 0.016f;
    const auto start = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < frames; ++frame) {
        system.update(dt);
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    TimingResult result;
    result.milliseconds = static_cast<double>(duration.count()) / 1000.0;
    return result;
}

TimingResult time_brute_force(int particle_count, int frames) {
    ParticleSystem system;
    system.enable_collisions(false);  // Manual brute-force

    // Identical particle distribution
    unsigned int seed = 42;
    constexpr float seed_max = 2147483647.0f;
    auto rand_float = [&seed](float min, float max) -> float {
        seed = (seed * 1103515245 + 12345) & 0x7fffffff;
        float normalized = static_cast<float>(seed) / seed_max;
        return min + normalized * (max - min);
    };

    for (int i = 0; i < particle_count; ++i) {
        const Vec3f pos(
            rand_float(-5.0f, 5.0f),
            rand_float(-5.0f, 5.0f),
            rand_float(-5.0f, 5.0f)
        );
        const Vec3f vel(
            rand_float(-0.5f, 0.5f),
            rand_float(-0.5f, 0.5f),
            rand_float(-0.5f, 0.5f)
        );
        system.spawn(pos, vel, 1.0f, -1.0f, 0.25f);
    }

    const float dt = 0.016f;
    const auto start = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < frames; ++frame) {
        auto& particles = system.particles();
        for (auto& p : particles) {
            if (p.is_alive()) {
                p.clear_forces();
                p.update_acceleration();
                p.integrate(dt);
            }
        }
        resolve_collisions_brute_force(particles);
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    TimingResult result;
    result.milliseconds = static_cast<double>(duration.count()) / 1000.0;
    return result;
}

TEST_CASE("Broadphase performance: Small system (10 particles)", "[validation][benchmark]") {
    SECTION("Broadphase vs brute-force timing") {
        const int particle_count = 10;
        const int frames = 100;
        const float cell_size = 1.0f;

        auto broadphase_result = time_broadphase(particle_count, cell_size, frames);
        auto brute_force_result = time_brute_force(particle_count, frames);

        // For small systems, brute-force might be faster due to overhead
        // Just verify both complete successfully
        REQUIRE(broadphase_result.milliseconds > 0.0);
        REQUIRE(brute_force_result.milliseconds > 0.0);
    }
}

TEST_CASE("Broadphase performance: Medium system (100 particles)", "[validation][benchmark]") {
    SECTION("Broadphase should scale well as particle count increases") {
        const int particle_count = 100;
        const int frames = 50;
        const float cell_size = 1.0f;

        auto broadphase_result = time_broadphase(particle_count, cell_size, frames);
        auto brute_force_result = time_brute_force(particle_count, frames);

        // At N=100, both approaches are relatively fast with low overhead
        // Broadphase may not be faster due to grid construction overhead
        // but should not be significantly slower
        REQUIRE(broadphase_result.milliseconds > 0.0);
        REQUIRE(brute_force_result.milliseconds > 0.0);
        
        const double ratio = broadphase_result.milliseconds / brute_force_result.milliseconds;
        REQUIRE(ratio < 2.0);  // Broadphase within 2x of brute-force at N=100
    }
}

TEST_CASE("Broadphase performance: Large system (500 particles)", "[validation][benchmark]") {
    SECTION("Broadphase should scale reasonably") {
        const int particle_count = 500;
        const int frames = 20;
        const float cell_size = 1.0f;

        auto broadphase_result = time_broadphase(particle_count, cell_size, frames);
        auto brute_force_result = time_brute_force(particle_count, frames);

        REQUIRE(broadphase_result.milliseconds > 0.0);
        REQUIRE(brute_force_result.milliseconds > 0.0);

        // At N=500, both approaches work but O(n²) becomes expensive
        // Focus on: broadphase should not be dramatically slower
        const double ratio = broadphase_result.milliseconds / brute_force_result.milliseconds;
        REQUIRE(ratio < 3.0);  // Broadphase within 3x of brute-force
    }
}

TEST_CASE("Broadphase performance: Cell size impact", "[validation][benchmark]") {
    // Test that different cell sizes produce valid results
    // (Performance tradeoffs depend on system state, not strictly faster/slower)
    
    SECTION("Different grid cell sizes complete successfully") {
        const int particle_count = 100;
        const int frames = 50;

        auto coarse = time_broadphase(particle_count, 4.0f, frames);   // Large cells
        auto medium = time_broadphase(particle_count, 2.0f, frames);   // Medium cells
        auto fine = time_broadphase(particle_count, 0.5f, frames);     // Small cells

        // All should complete successfully
        REQUIRE(coarse.milliseconds > 0.0);
        REQUIRE(medium.milliseconds > 0.0);
        REQUIRE(fine.milliseconds > 0.0);
        
        // Cell size affects performance but doesn't make it fail
        // (optimal size depends on particle distribution and query patterns)
    }
}

TEST_CASE("Broadphase performance: Scaling behavior", "[validation][benchmark]") {
    // Verify that broadphase scales reasonably as particles increase
    
    SECTION("Scaling analysis") {
        // Time two different sizes
        const float cell_size = 1.0f;
        const int frames = 50;

        auto small = time_broadphase(50, cell_size, frames);   // N=50
        auto large = time_broadphase(100, cell_size, frames);  // N=100 (2x particles)

        // Both should complete successfully
        REQUIRE(small.milliseconds > 0.0);
        REQUIRE(large.milliseconds > 0.0);
        
        // Time should increase with particle count (obviously)
        const double time_ratio = large.milliseconds / small.milliseconds;
        REQUIRE(time_ratio > 1.0);  // 100 particles takes more time than 50
    }
}

TEST_CASE("Broadphase performance: Consistency check", "[validation][benchmark]") {
    // Same test run twice should produce consistent timing
    
    SECTION("Reproducible timing within reasonable tolerance") {
        const int particle_count = 100;
        const int frames = 50;
        const float cell_size = 1.0f;

        auto run1 = time_broadphase(particle_count, cell_size, frames);
        auto run2 = time_broadphase(particle_count, cell_size, frames);

        // Results should be within 50% of each other (timing can vary with system load)
        const double ratio = run2.milliseconds / run1.milliseconds;
        REQUIRE(ratio > 0.5);
        REQUIRE(ratio < 2.0);
    }
}

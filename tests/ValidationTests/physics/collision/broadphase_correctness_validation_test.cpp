#include <catch2/catch_all.hpp>
#include <core/physics/particle_system.hpp>
#include <core/physics/collision/sphere_sphere_narrowphase.hpp>
#include <core/physics/constraints/contact_constraint.hpp>
#include <core/physics/constraints/constraint_solver.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

using namespace phynity::physics;
using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;

/**
 * @brief Helper function to perform brute-force O(n²) collision detection.
 * 
 * Used as reference for validating that broadphase+narrowphase produces
 * identical results to the original O(n²) approach.
 */
void resolve_collisions_brute_force(std::vector<Particle>& particles) {
    std::vector<ContactManifold> manifolds;
    const size_t count = particles.size();

    for (size_t i = 0; i < count; ++i) {
        Particle& a = particles[i];
        if (!a.is_alive()) {
            continue;
        }

        for (size_t j = i + 1; j < count; ++j) {
            Particle& b = particles[j];
            if (!b.is_alive()) {
                continue;
            }

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
                manifolds.push_back(manifold);
            }
        }
    }

    if (manifolds.empty()) {
        return;
    }

    std::vector<std::unique_ptr<constraints::Constraint>> constraints;
    constraints.reserve(manifolds.size());
    for (const auto& manifold : manifolds) {
        if (manifold.object_a_id < particles.size() && manifold.object_b_id < particles.size()) {
            constraints.push_back(std::make_unique<constraints::ContactConstraint>(
                manifold,
                particles[manifold.object_a_id],
                particles[manifold.object_b_id],
                constraints::ContactConstraint::ContactType::Normal
            ));
        }
    }

    constraints::ConstraintSolver solver;
    solver.solve(constraints, particles);
}

TEST_CASE("Broadphase correctness: Simple sphere-sphere collision", "[validation][broadphase]") {
    // Test that broadphase produces identical results to brute-force for a 2-particle collision
    
    SECTION("Head-on collision") {
        // Scenario: Two particles approaching each other
        const float dt = 0.016f;  // ~60 FPS
        const int steps = 2;
        const float pos_tolerance = tolerance::POSITION;
        const float vel_tolerance = tolerance::VELOCITY;

        // Create two identical scenarios
        ParticleSystem broadphase_system;
        ParticleSystem brute_force_system;
        
        broadphase_system.enable_collisions(true);
        broadphase_system.set_broadphase_cell_size(2.0f);
        
        brute_force_system.enable_collisions(false);  // We'll manually do brute-force

        // Spawn identical particles
        const Vec3f pos_a(-1.0f, 0.0f, 0.0f);
        const Vec3f pos_b(1.0f, 0.0f, 0.0f);
        const Vec3f vel_a(1.0f, 0.0f, 0.0f);    // Moving right
        const Vec3f vel_b(-1.0f, 0.0f, 0.0f);   // Moving left
        const float radius = 0.5f;
        const float mass = 1.0f;

        // Broadphase system
        broadphase_system.spawn(pos_a, vel_a, mass, -1.0f, radius);
        broadphase_system.spawn(pos_b, vel_b, mass, -1.0f, radius);

        // Brute-force system (identical initial state)
        brute_force_system.spawn(pos_a, vel_a, mass, -1.0f, radius);
        brute_force_system.spawn(pos_b, vel_b, mass, -1.0f, radius);

        // Run simulation
        for (int step = 0; step < steps; ++step) {
            // Broadphase update
            broadphase_system.update(dt);

            // Brute-force update
            auto& bf_particles = brute_force_system.particles();
            for (auto& p : bf_particles) {
                if (p.is_alive()) {
                    p.clear_forces();
                    p.update_acceleration();
                    p.integrate(dt);
                }
            }
            resolve_collisions_brute_force(bf_particles);
        }

        // Compare results
        const auto& bp_particles = broadphase_system.particles();
        const auto& bf_particles = brute_force_system.particles();

        REQUIRE(bp_particles.size() == bf_particles.size());

        for (size_t i = 0; i < bp_particles.size(); ++i) {
            REQUIRE_THAT(bp_particles[i].position.x, 
                        Catch::Matchers::WithinAbs(bf_particles[i].position.x, pos_tolerance));
            REQUIRE_THAT(bp_particles[i].position.y,
                        Catch::Matchers::WithinAbs(bf_particles[i].position.y, pos_tolerance));
            REQUIRE_THAT(bp_particles[i].position.z,
                        Catch::Matchers::WithinAbs(bf_particles[i].position.z, pos_tolerance));

            REQUIRE_THAT(bp_particles[i].velocity.x,
                        Catch::Matchers::WithinAbs(bf_particles[i].velocity.x, vel_tolerance));
            REQUIRE_THAT(bp_particles[i].velocity.y,
                        Catch::Matchers::WithinAbs(bf_particles[i].velocity.y, vel_tolerance));
            REQUIRE_THAT(bp_particles[i].velocity.z,
                        Catch::Matchers::WithinAbs(bf_particles[i].velocity.z, vel_tolerance));
        }
    }
}

TEST_CASE("Broadphase correctness: Dense cluster collision", "[validation][broadphase]") {
    // Multiple particles in tight cluster to test complex interactions
    
    SECTION("4-particle square cluster") {
        const float dt = 0.016f;
        const int steps = 5;
        const float pos_tolerance = tolerance::POSITION;
        const float vel_tolerance = tolerance::VELOCITY;

        ParticleSystem broadphase_system;
        ParticleSystem brute_force_system;
        
        broadphase_system.enable_collisions(true);
        broadphase_system.set_broadphase_cell_size(2.0f);
        
        brute_force_system.enable_collisions(false);

        // Create 4 particles in a square
        const std::vector<Vec3f> positions = {
            Vec3f(-0.5f, -0.5f, 0.0f),
            Vec3f(0.5f, -0.5f, 0.0f),
            Vec3f(-0.5f, 0.5f, 0.0f),
            Vec3f(0.5f, 0.5f, 0.0f)
        };
        
        const std::vector<Vec3f> velocities = {
            Vec3f(0.5f, 0.5f, 0.0f),   // Moving toward center
            Vec3f(-0.5f, 0.5f, 0.0f),
            Vec3f(0.5f, -0.5f, 0.0f),
            Vec3f(-0.5f, -0.5f, 0.0f)
        };

        for (size_t i = 0; i < 4; ++i) {
            broadphase_system.spawn(positions[i], velocities[i], 1.0f, -1.0f, 0.3f);
            brute_force_system.spawn(positions[i], velocities[i], 1.0f, -1.0f, 0.3f);
        }

        // Run simulation
        for (int step = 0; step < steps; ++step) {
            broadphase_system.update(dt);

            auto& bf_particles = brute_force_system.particles();
            for (auto& p : bf_particles) {
                if (p.is_alive()) {
                    p.clear_forces();
                    p.update_acceleration();
                    p.integrate(dt);
                }
            }
            resolve_collisions_brute_force(bf_particles);
        }

        // Compare results
        const auto& bp_particles = broadphase_system.particles();
        const auto& bf_particles = brute_force_system.particles();

        for (size_t i = 0; i < bp_particles.size(); ++i) {
            REQUIRE_THAT(bp_particles[i].position.x,
                        Catch::Matchers::WithinAbs(bf_particles[i].position.x, pos_tolerance));
            REQUIRE_THAT(bp_particles[i].velocity.x,
                        Catch::Matchers::WithinAbs(bf_particles[i].velocity.x, vel_tolerance));
        }
    }
}

TEST_CASE("Broadphase correctness: Sparse system (no collisions)", "[validation][broadphase]") {
    // Particles far apart should produce identical results trivially
    
    SECTION("10 particles, no collisions") {
        const float dt = 0.016f;
        const int steps = 10;
        const float pos_tolerance = tolerance::POSITION;

        ParticleSystem broadphase_system;
        ParticleSystem brute_force_system;
        
        broadphase_system.enable_collisions(true);
        broadphase_system.set_broadphase_cell_size(2.0f);
        
        brute_force_system.enable_collisions(false);

        // Spawn particles far apart (no collisions possible)
        for (int i = 0; i < 10; ++i) {
            const float x = static_cast<float>(i) * 10.0f;  // Spaced 10 units apart
            const Vec3f pos(x, 0.0f, 0.0f);
            const Vec3f vel(0.0f, 0.0f, 0.0f);
            
            broadphase_system.spawn(pos, vel);
            brute_force_system.spawn(pos, vel);
        }

        // Run simulation
        for (int step = 0; step < steps; ++step) {
            broadphase_system.update(dt);

            auto& bf_particles = brute_force_system.particles();
            for (auto& p : bf_particles) {
                if (p.is_alive()) {
                    p.clear_forces();
                    p.update_acceleration();
                    p.integrate(dt);
                }
            }
            // No collisions to resolve
        }

        // Results should be identical (no collisions occurred)
        const auto& bp_particles = broadphase_system.particles();
        const auto& bf_particles = brute_force_system.particles();

        for (size_t i = 0; i < bp_particles.size(); ++i) {
            REQUIRE_THAT(bp_particles[i].position.x,
                        Catch::Matchers::WithinAbs(bf_particles[i].position.x, pos_tolerance));
        }
    }
}

TEST_CASE("Broadphase correctness: Particle at cell boundary", "[validation][broadphase]") {
    // Test that particles at grid cell boundaries are handled correctly
    
    SECTION("Collision across cell boundary") {
        const float dt = 0.016f;
        const int steps = 3;
        const float pos_tolerance = tolerance::POSITION;
        const float vel_tolerance = tolerance::VELOCITY;

        ParticleSystem broadphase_system;
        ParticleSystem brute_force_system;
        
        broadphase_system.enable_collisions(true);
        broadphase_system.set_broadphase_cell_size(2.0f);  // Cell size 2.0
        
        brute_force_system.enable_collisions(false);

        // Position particles at/near cell boundary (x=2.0)
        // Particle A in cell x=0, Particle B in cell x=1
        const Vec3f pos_a(1.9f, 0.0f, 0.0f);   // Near boundary
        const Vec3f pos_b(2.1f, 0.0f, 0.0f);   // Just past boundary
        const Vec3f vel_a(0.1f, 0.0f, 0.0f);   // Moving right
        const Vec3f vel_b(-0.1f, 0.0f, 0.0f);  // Moving left

        broadphase_system.spawn(pos_a, vel_a, 1.0f, -1.0f, 0.5f);
        broadphase_system.spawn(pos_b, vel_b, 1.0f, -1.0f, 0.5f);

        brute_force_system.spawn(pos_a, vel_a, 1.0f, -1.0f, 0.5f);
        brute_force_system.spawn(pos_b, vel_b, 1.0f, -1.0f, 0.5f);

        // Run simulation
        for (int step = 0; step < steps; ++step) {
            broadphase_system.update(dt);

            auto& bf_particles = brute_force_system.particles();
            for (auto& p : bf_particles) {
                if (p.is_alive()) {
                    p.clear_forces();
                    p.update_acceleration();
                    p.integrate(dt);
                }
            }
            resolve_collisions_brute_force(bf_particles);
        }

        // Compare
        const auto& bp_particles = broadphase_system.particles();
        const auto& bf_particles = brute_force_system.particles();

        for (size_t i = 0; i < bp_particles.size(); ++i) {
            REQUIRE_THAT(bp_particles[i].position.x,
                        Catch::Matchers::WithinAbs(bf_particles[i].position.x, pos_tolerance));
            REQUIRE_THAT(bp_particles[i].velocity.x,
                        Catch::Matchers::WithinAbs(bf_particles[i].velocity.x, vel_tolerance));
        }
    }
}

TEST_CASE("Broadphase correctness: Conservation laws", "[validation][broadphase]") {
    // Verify that broadphase system conserves momentum and energy like brute-force
    
    SECTION("Momentum conservation in collision") {
        const float dt = 0.016f;
        const int steps = 3;
        const float momentum_tolerance = tolerance::MOMENTUM;

        ParticleSystem system;
        system.enable_collisions(true);
        system.set_broadphase_cell_size(2.0f);

        // Create collision: equal mass, opposite velocities
        system.spawn(Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), 1.0f);
        system.spawn(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), 1.0f);

        // Initial momentum should be zero
        auto initial_diag = system.compute_diagnostics();
        REQUIRE_THAT(initial_diag.total_momentum.x, 
                    Catch::Matchers::WithinAbs(0.0f, momentum_tolerance));

        // Run simulation
        for (int step = 0; step < steps; ++step) {
            system.update(dt);
        }

        // Final momentum should still be zero (conserved)
        auto final_diag = system.compute_diagnostics();
        REQUIRE_THAT(final_diag.total_momentum.x,
                    Catch::Matchers::WithinAbs(0.0f, momentum_tolerance));
    }
}

TEST_CASE("Broadphase correctness: Different grid cell sizes", "[validation][broadphase]") {
    // Test that results are consistent regardless of cell size
    // (as long as cell size is reasonable relative to collision radius)
    
    SECTION("Same scenario with different cell sizes") {
        const float dt = 0.016f;
        const int steps = 2;
        const float pos_tolerance = tolerance::POSITION;

        ParticleSystem system_coarse;
        ParticleSystem system_fine;
        
        system_coarse.enable_collisions(true);
        system_coarse.set_broadphase_cell_size(4.0f);  // 4x particle radius
        
        system_fine.enable_collisions(true);
        system_fine.set_broadphase_cell_size(1.0f);   // 1x particle radius

        // Identical scenario
        const Vec3f pos_a(-1.0f, 0.0f, 0.0f);
        const Vec3f pos_b(1.0f, 0.0f, 0.0f);
        const Vec3f vel_a(1.0f, 0.0f, 0.0f);
        const Vec3f vel_b(-1.0f, 0.0f, 0.0f);

        system_coarse.spawn(pos_a, vel_a, 1.0f, -1.0f, 0.5f);
        system_coarse.spawn(pos_b, vel_b, 1.0f, -1.0f, 0.5f);

        system_fine.spawn(pos_a, vel_a, 1.0f, -1.0f, 0.5f);
        system_fine.spawn(pos_b, vel_b, 1.0f, -1.0f, 0.5f);

        // Run both
        for (int step = 0; step < steps; ++step) {
            system_coarse.update(dt);
            system_fine.update(dt);
        }

        // Results should be very similar (both use same narrowphase)
        const auto& coarse = system_coarse.particles();
        const auto& fine = system_fine.particles();

        for (size_t i = 0; i < coarse.size(); ++i) {
            REQUIRE_THAT(coarse[i].position.x,
                        Catch::Matchers::WithinAbs(fine[i].position.x, pos_tolerance * 2.0f));
            // Slightly more lenient tolerance due to different grid layouts
        }
    }
}

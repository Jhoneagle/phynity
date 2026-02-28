#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/particle_system.hpp>
#include <core/physics/particle.hpp>
#include <core/physics/force_field.hpp>
#include <core/physics/collision/pgs_solver.hpp>
#include <core/math/vectors/vec3.hpp>

using namespace phynity::physics;
using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using Catch::Matchers::WithinAbs;

namespace {
    // Helper: Create standard test system
    ParticleSystem create_stable_system() {
        ParticleSystem system;
        system.set_solver_mode(ParticleSystem::SolverMode::PGS);
        system.enable_collisions(true);
        system.enable_constraints(true);
        
        PGSConfig pgs_config;
        pgs_config.max_iterations = 8;
        pgs_config.convergence_threshold = 1e-5f;
        pgs_config.friction_coefficient = 0.5f;
        system.set_pgs_config(pgs_config);
        
        return system;
    }

    // Helper: Compute total energy (PE + KE)
    float compute_total_energy(const ParticleSystem& system, float gravity = 9.81f) {
        float total_pe = 0.0f;
        float total_ke = 0.0f;
        
        const auto& particles = system.particles();
        for (size_t i = 0; i < system.particleCount(); ++i) {
            float mass = particles[i].material.mass;
            if (mass > 0.0f) {
                total_pe += mass * gravity * particles[i].position.y;
                total_ke += 0.5f * mass * particles[i].velocity.squaredLength();
            }
        }
        return total_pe + total_ke;
    }

    // Helper: Wait for settling (velocity drops below threshold)
    void wait_until_settled(ParticleSystem& system, float velocity_threshold = 0.05f, 
                           float max_time = 10.0f, float dt = 1.0f / 60.0f) {
        int max_frames = static_cast<int>(max_time / dt);
        bool any_moving = true;
        
        for (int i = 0; i < max_frames && any_moving; ++i) {
            system.update(dt);
            
            any_moving = false;
            for (const auto& p : system.particles()) {
                if (p.is_alive() && p.velocity.length() > velocity_threshold) {
                    any_moving = true;
                    break;
                }
            }
        }
    }
}

// ============================================================================
// BASELINE MEASUREMENT: What does the solver actually deliver?
// ============================================================================

TEST_CASE("Stacking: 2-particle baseline measurement", "[stacking][validation]") {
    // PURPOSE: Establish golden baseline for 2-particle stack
    // This measures what the solver ACTUALLY does, not what we wish it would do
    
    auto system = create_stable_system();
    Material mat{1.0f, 0.5f, 0.3f};
    
    // Ground (immovable)
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    size_t ground_idx = system.particleCount() - 1;
    system.particles()[ground_idx].material.mass = 0.0f;
    
    // Top particle
    system.spawn(Vec3f(0.0f, 2.5f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    size_t top_idx = system.particleCount() - 1;
    
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
    // Simulate until settled
    float dt = 1.0f / 60.0f;
    wait_until_settled(system, 0.05f, 5.0f, dt);
    
    float final_height = system.particles()[top_idx].position.y;
    float final_velocity = system.particles()[top_idx].velocity.length();
    
    // Record measurements
    INFO("Baseline: Final height = " << final_height);
    INFO("Baseline: Final velocity = " << final_velocity);
    
    SECTION("Particle settles within reasonable bounds") {
        // ✅ Conservative check: Must be above 1.0 (not underground) and below 2.5 (initial position)
        REQUIRE(final_height > 1.0f);
        REQUIRE(final_height < 2.5f);
    }
    
    SECTION("Velocity drops to near-zero") {
        // ✅ After settling, velocity should be small (but not zero due to numerical noise)
        REQUIRE(final_velocity < 0.2f);  // Loose tolerance for iterative solver
    }
    
    SECTION("Height remains stable over longer simulation") {
        float height_before = final_height;
        
        // Simulate 1 more second
        for (int i = 0; i < 60; ++i) {
            system.update(dt);
        }
        
        float height_after = system.particles()[top_idx].position.y;
        
        // ✅ Once settled, height should stay approximately the same
        // (allows reasonable numerical drift)
        float drift = std::abs(height_after - height_before);
        REQUIRE(drift < 0.3f);  // Allow up to 0.3 units drift
    }
}

// ============================================================================
// CONSERVATION LAWS: Must hold for ANY physical simulation
// ============================================================================

TEST_CASE("Stacking: Energy monotonicity (never increase)", "[stacking][validation]") {
    // PURPOSE: Validate fundamental physics: energy dissipates or stays constant
    // This is system-independent and solver-independent
    
    auto system = create_stable_system();
    Material mat{1.0f, 0.5f, 0.3f};
    
    // Ground
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    size_t ground_idx = system.particleCount() - 1;
    system.particles()[ground_idx].material.mass = 0.0f;
    
    // Two falling particles
    system.spawn(Vec3f(0.5f, 3.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    system.spawn(Vec3f(-0.5f, 3.5f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
    float dt = 1.0f / 60.0f;
    float prev_energy = compute_total_energy(system);
    int energy_violations = 0;
    
    SECTION("Total energy never increases") {
        for (int i = 0; i < 600; ++i) {  // 10 seconds
            system.update(dt);
            
            float current_energy = compute_total_energy(system);
            
            // Allow tiny numerical error (1e-3 per frame)
            if (current_energy > prev_energy + 0.001f) {
                energy_violations++;
            }
            
            prev_energy = current_energy;
        }
        
        // ✅ Energy should almost never increase, allow a few numerical blips
        REQUIRE(energy_violations < 10);  // Less than 1% of frames
    }
}

TEST_CASE("Stacking: 3-particle tower settling", "[stacking][validation]") {
    // PURPOSE: Validate stacking of 3 particles
    // Uses loose tolerances since solver is approximate
    
    auto system = create_stable_system();
    Material mat{1.0f, 0.5f, 0.3f};
    
    // Ground
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    std::vector<size_t> indices;
    indices.push_back(system.particleCount() - 1);
    system.particles()[indices[0]].material.mass = 0.0f;
    
    // Two more particles
    system.spawn(Vec3f(0.0f, 2.5f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    indices.push_back(system.particleCount() - 1);
    
    system.spawn(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    indices.push_back(system.particleCount() - 1);
    
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
    wait_until_settled(system, 0.05f, 5.0f);
    
    SECTION("Particles maintain vertical alignment") {
        // ✅ Should stay roughly aligned (x, z near zero)
        float max_x_drift = 0.0f;
        for (size_t i = 1; i < indices.size(); ++i) {
            max_x_drift = std::max(max_x_drift, std::abs(system.particles()[indices[i]].position.x));
        }
        REQUIRE(max_x_drift < 0.3f);
    }
    
    SECTION("Particles settle with low velocities") {
        // ✅ After settling, all should have v < 0.2
        for (size_t i = 1; i < indices.size(); ++i) {
            float v = system.particles()[indices[i]].velocity.length();
            REQUIRE(v < 0.2f);
        }
    }
    
    SECTION("Heights form monotonic stack") {
        // ✅ Each particle should be above the previous one
        for (size_t i = 1; i < indices.size(); ++i) {
            float h_curr = system.particles()[indices[i]].position.y;
            float h_prev = system.particles()[indices[i-1]].position.y;
            
            REQUIRE(h_curr > h_prev);  // Strictly above previous
        }
    }
}

// ============================================================================
// CONSTRAINT PROPERTY: Distance preservation
// ============================================================================

TEST_CASE("Stacking: Fixed constraint maintains distance", "[stacking][validation]") {
    // PURPOSE: Validate that constraints actually preserve distance
    // Loose tolerance but must hold consistently
    
    auto system = create_stable_system();
    Material mat{1.0f, 0.5f, 0.3f};
    
    // Ground
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 1.0f);
    size_t ground_idx = system.particleCount() - 1;
    system.particles()[ground_idx].material.mass = 0.0f;
    
    // Two particles with constraint
    system.spawn(Vec3f(-1.0f, 2.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 0.5f);
    size_t p1_idx = system.particleCount() - 1;
    
    system.spawn(Vec3f(1.0f, 2.0f, 0.0f), Vec3f(0.0f), mat, -1.0f, 0.5f);
    size_t p2_idx = system.particleCount() - 1;
    
    float initial_distance = (system.particles()[p2_idx].position - 
                             system.particles()[p1_idx].position).length();
    
    system.add_fixed_constraint(p1_idx, p2_idx);
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    
    wait_until_settled(system);
    
    SECTION("Constraint distance remains stable") {
        float max_drift = 0.0f;
        
        // Check distance over 60 more frames
        for (int i = 0; i < 60; ++i) {
            system.update(1.0f / 60.0f);
            
            Vec3f pos1 = system.particles()[p1_idx].position;
            Vec3f pos2 = system.particles()[p2_idx].position;
            float current_distance = (pos2 - pos1).length();
            float drift = std::abs(current_distance - initial_distance);
            
            max_drift = std::max(max_drift, drift);
        }
        
        // ✅ Constraint should maintain distance with reasonable precision
        REQUIRE(max_drift < 0.20f);  // Allow 0.2 unit drift max
    }
}

// ============================================================================
// COMPLEX SCENARIOS: Loose bounds for current solver behavior
// ============================================================================

TEST_CASE("Stacking: 5-particle tower (loose bounds)", "[stacking][validation]") {
    // PURPOSE: Validate that the simulation remains stable for a taller stack
    // Expectations are intentionally loose to match current solver capabilities
    
    auto system = create_stable_system();
    Material mat{1.0f, 0.5f, 0.3f};
    
    // Just verify basic properties, not exact heights
    std::vector<size_t> indices;
    
    // Ground
    system.spawn(Vec3f(0, 0, 0), Vec3f(0, 0, 0), mat, -1.0f, 1.0f);
    indices.push_back(system.particleCount() - 1);
    system.particles()[indices[0]].material.mass = 0.0f;
    
    // Stack 4 more
    for (int i = 1; i < 5; ++i) {
        system.spawn(Vec3f(0, static_cast<float>(i) * 2.5f, 0), Vec3f(0, 0, 0), mat, -1.0f, 1.0f);
        indices.push_back(system.particleCount() - 1);
    }
    
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0, -9.81f, 0)));
    
    wait_until_settled(system, 0.1f, 10.0f);  // Longer settling time needed
    
    SECTION("Positions remain finite and bounded") {
        for (size_t i = 1; i < indices.size(); ++i) {
            const Vec3f pos = system.particles()[indices[i]].position;
            REQUIRE(std::isfinite(pos.x));
            REQUIRE(std::isfinite(pos.y));
            REQUIRE(std::isfinite(pos.z));
            REQUIRE(std::abs(pos.x) < 200.0f);
            REQUIRE(std::abs(pos.y) < 200.0f);
            REQUIRE(std::abs(pos.z) < 200.0f);
        }
    }
    
    SECTION("Velocities remain bounded") {
        for (size_t i = 1; i < indices.size(); ++i) {
            const float speed = system.particles()[indices[i]].velocity.length();
            REQUIRE(std::isfinite(speed));
            REQUIRE(speed < 200.0f);
        }
    }
}

TEST_CASE("Stacking: Pyramid structure (loose bounds)", "[stacking][validation]") {
    // PURPOSE: Complex multi-contact scenario with loose stability checks
    
    auto system = create_stable_system();
    Material mat{1.0f, 0.5f, 0.3f};
    
    // Just load and simulate, no strict assertions
    system.spawn(Vec3f(0, -0.5f, 0), Vec3f(0, 0, 0), mat, -1.0f, 1.5f);
    size_t ground_idx = system.particleCount() - 1;
    system.particles()[ground_idx].material.mass = 0.0f;
    
    float spacing = 1.1f;
    
    // 3x3 base layer
    for (int x = -1; x <= 1; ++x) {
        for (int z = -1; z <= 1; ++z) {
            system.spawn(
                Vec3f(static_cast<float>(x) * spacing, 1.0f, static_cast<float>(z) * spacing),
                Vec3f(0, 0, 0), mat, -1.0f, 0.5f
            );
        }
    }
    
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0, -9.81f, 0)));
    
    wait_until_settled(system, 0.15f, 10.0f);
    
    SECTION("System remains stable (sanity check only)") {
        for (const auto& p : system.particles()) {
            REQUIRE(std::isfinite(p.position.x));
            REQUIRE(std::isfinite(p.position.y));
            REQUIRE(std::isfinite(p.position.z));
            REQUIRE(std::abs(p.position.x) < 1000.0f);
            REQUIRE(std::abs(p.position.y) < 1000.0f);
            REQUIRE(std::abs(p.position.z) < 1000.0f);
        }
    }
}

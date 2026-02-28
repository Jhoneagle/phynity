#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/micro/particle_system.hpp>
#include <core/physics/collision/contact/contact_cache.hpp>

using namespace phynity::physics;
using namespace phynity::physics::collision;
using namespace phynity::math::vectors;

TEST_CASE("ContactCache - Stacked particles persistence", "[collision][contact_cache][golden]") {
    ParticleSystem system;
    system.enable_collisions(true);
    system.set_broadphase_cell_size(2.0f);
    
    // Create two particles stacked vertically (they will be in contact)
    // Particle 0: on the ground (static - very heavy)
    system.spawn(
        Vec3f(0.0f, 0.0f, 0.0f),  // position
        Vec3f(0.0f, 0.0f, 0.0f),  // velocity
        1000.0f,                   // mass (heavy = nearly static)
        -1.0f,                     // lifetime (infinite)
        1.0f                       // radius
    );
    
    // Particle 1: on top, will settle onto particle 0
    system.spawn(
        Vec3f(0.0f, 1.8f, 0.0f),  // position (just touching radius-to-radius)
        Vec3f(0.0f, 0.0f, 0.0f),  // velocity
        1.0f,                      // mass
        -1.0f,                     // lifetime (infinite)
        1.0f                       // radius
    );
    
    // Add gravity
    system.add_force_field(std::make_unique<phynity::physics::GravityField>(
        Vec3f(0.0f, -9.8f, 0.0f)  // gravity
    ));
    
    SECTION("Contacts persist across multiple frames") {
        const float dt = 0.016f;  // ~60 FPS
        
        // Simulate for several frames
        // The two particles should remain in contact after initial settling
        for (int frame = 0; frame < 10; ++frame) {
            system.update(dt);
        }
        
        // After settling, particles should be in stable contact
        // We can't directly access the contact cache from ParticleSystem,
        // but we can verify the system is stable (positions don't explode)
        const auto& particles = system.particles();
        REQUIRE(particles.size() == 2);
        REQUIRE(particles[0].is_alive());
        REQUIRE(particles[1].is_alive());
        
        // Verify particles are still close (in contact)
        float distance = (particles[1].position - particles[0].position).length();
        REQUIRE(distance < 2.5f);  // Should be around 2.0 (sum of radii)
    }
    
    SECTION("Determinism - Same initial conditions produce same results") {
        ParticleSystem system1;
        ParticleSystem system2;
        
        // Setup identical systems
        for (auto* sys : {&system1, &system2}) {
            sys->enable_collisions(true);
            sys->set_broadphase_cell_size(2.0f);
            
            sys->spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1000.0f, -1.0f, 1.0f);
            sys->spawn(Vec3f(0.0f, 1.8f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f, -1.0f, 1.0f);
            
            sys->add_force_field(std::make_unique<phynity::physics::GravityField>(
                Vec3f(0.0f, -9.8f, 0.0f)
            ));
        }
        
        const float dt = 0.016f;
        const int num_frames = 15;
        
        // Run both systems
        for (int frame = 0; frame < num_frames; ++frame) {
            system1.update(dt);
            system2.update(dt);
        }
        
        // Verify identical final states
        const auto& particles1 = system1.particles();
        const auto& particles2 = system2.particles();
        
        REQUIRE(particles1.size() == particles2.size());
        
        for (size_t i = 0; i < particles1.size(); ++i) {
            // Check positions match (within floating point tolerance)
            float pos_diff = (particles1[i].position - particles2[i].position).length();
            REQUIRE(pos_diff < 1e-5f);
            
            // Check velocities match
            float vel_diff = (particles1[i].velocity - particles2[i].velocity).length();
            REQUIRE(vel_diff < 1e-5f);
        }
    }
}

TEST_CASE("ContactCache - Multiple simultaneous contacts", "[collision][contact_cache][golden]") {
    ParticleSystem system;
    system.enable_collisions(true);
    system.set_broadphase_cell_size(3.0f);
    
    // Create a 3-particle stack with very heavy bottom particle for stability
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 10000.0f, -1.0f, 1.0f);  // Bottom (very heavy)
    system.spawn(Vec3f(0.0f, 1.9f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 100.0f, -1.0f, 1.0f);    // Middle
    system.spawn(Vec3f(0.0f, 3.8f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f, -1.0f, 1.0f);      // Top (light)
    
    system.add_force_field(std::make_unique<phynity::physics::GravityField>(
        Vec3f(0.0f, -9.8f, 0.0f)
    ));
    
    SECTION("Stack settles without exploding") {
        const float dt = 0.016f;
        
        // Simulate for a shorter period (stacks can be unstable without PGS solver)
        for (int frame = 0; frame < 30; ++frame) {  // ~0.5 seconds at 60 fps
            system.update(dt);
        }
        
        // Verify particles haven't exploded to unrealistic positions
        const auto& particles = system.particles();
        REQUIRE(particles.size() == 3);
        
        // Check no particle has exploded far away from origin
        for (const auto& p : particles) {
            REQUIRE(std::abs(p.position.x) < 10.0f);
            REQUIRE(p.position.y > -10.0f);  // Allow some settling/penetration
            REQUIRE(p.position.y < 15.0f);
            REQUIRE(std::abs(p.position.z) < 10.0f);
        }
        
        // Verify bottom particle hasn't moved much (it's very heavy)
        REQUIRE(std::abs(particles[0].position.y) < 2.0f);
    }
}

TEST_CASE("ContactCache - Contact appears and disappears", "[collision][contact_cache][golden]") {
    ParticleSystem system;
    system.enable_collisions(true);
    system.set_broadphase_cell_size(3.0f);
    
    // Create two particles that will collide and separate
    system.spawn(
        Vec3f(-5.0f, 0.0f, 0.0f),  // Left particle
        Vec3f(2.0f, 0.0f, 0.0f),   // Moving right
        1.0f, -1.0f, 1.0f
    );
    
    system.spawn(
        Vec3f(5.0f, 0.0f, 0.0f),   // Right particle
        Vec3f(-2.0f, 0.0f, 0.0f),  // Moving left
        1.0f, -1.0f, 1.0f
    );
    
    SECTION("Particles collide and separate") {
        const float dt = 0.016f;
        
        Vec3f initial_momentum = system.particles()[0].velocity * system.particles()[0].material.mass +
                                  system.particles()[1].velocity * system.particles()[1].material.mass;
        
        // Simulate collision
        for (int frame = 0; frame < 60; ++frame) {
            system.update(dt);
        }
        
        // Verify particles are still alive
        REQUIRE(system.particles().size() == 2);
        
        // After collision with restitution, particles should have separated
        float final_distance = (system.particles()[0].position - system.particles()[1].position).length();
        REQUIRE(final_distance > 2.0f);  // Should be more than sum of radii
        
        // Verify momentum is roughly conserved (no external forces)
        Vec3f final_momentum = system.particles()[0].velocity * system.particles()[0].material.mass +
                               system.particles()[1].velocity * system.particles()[1].material.mass;
        
        float momentum_diff = (final_momentum - initial_momentum).length();
        REQUIRE(momentum_diff < 0.5f);  // Allow some numerical error
    }
}

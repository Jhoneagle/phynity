#include <core/physics/particle_system.hpp>
#include <iostream>
#include <iomanip>

using phynity::physics::ParticleSystem;
using phynity::math::vectors::Vec3f;

int main() {
    std::cout << "=== Phynity Phase 0 - Simple Particle System ===" << std::endl;

    ParticleSystem system;
    
    // Spawn some particles
    system.spawn(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f), 1.0f);
    system.spawn(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), 1.0f);
    system.spawn(Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f, 0.0f, 1.0f), 1.0f);
    
    std::cout << "Spawned " << system.particleCount() << " particles at y=5" << std::endl;

    // Simulation parameters
    const float dt = 0.016f;  // ~60 FPS
    const float gravity = -9.81f;
    const int steps = 300;  // ~5 seconds
    
    // Run simulation
    for (int i = 0; i < steps; ++i) {
        // Apply gravity
        system.applyGravity(Vec3f(0.0f, gravity, 0.0f));
        
        // Step simulation
        system.step(dt);
        
        // Print status every 60 frames (~1 second)
        if (i % 60 == 0) {
            float time = static_cast<float>(i) * dt;
            std::cout << std::fixed << std::setprecision(2)
                      << "t=" << time << "s, particles=" << system.particleCount();
            
            if (system.particleCount() > 0) {
                const auto& particles = system.particles();
                std::cout << ", first particle at " << particles[0].position;
            }
            std::cout << std::endl;
        }
    }

    std::cout << "Simulation complete. Final particle count: " << system.particleCount() << std::endl;
    return 0;
}

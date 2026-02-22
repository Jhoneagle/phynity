#include "physics_context.hpp"
#include "demo_scenarios.hpp"
#include <iostream>
#include <iomanip>
#include <memory>
#include <vector>

using phynity::app::PhysicsContext;
using phynity::app::scenarios::Scenario;
using phynity::math::vectors::Vec3f;

/// Simple demo runner that executes a scenario for a specified duration
void run_scenario(std::unique_ptr<Scenario> scenario, float duration_seconds = 5.0f) {
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "Scenario: " << scenario->name() << std::endl;
    std::cout << "Description: " << scenario->description() << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    // Create physics context with default config (60 FPS)
    PhysicsContext context;
    
    // Setup the scenario
    scenario->setup(context);
    
    std::cout << "Initial particle count: " << context.particle_count() << std::endl;
    std::cout << "Initial force fields: " << context.force_field_count() << std::endl;
    std::cout << "Running for " << duration_seconds << " seconds...\n" << std::endl;

    // Simulation loop
    const float dt = 0.016f;  // ~60 FPS frame time
    const int frames = static_cast<int>(duration_seconds / dt);
    
    for (int frame = 0; frame < frames; ++frame) {
        // Update physics with frame time
        context.update(dt);
        
        // Call scenario step callback if needed
        scenario->step_callback(context, dt);
        
        // Print diagnostics every 60 frames (~1 second)
        if (frame % 60 == 0) {
            float elapsed_time = static_cast<float>(frame) * dt;
            auto diag = context.diagnostics();
            
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "t=" << std::setw(5) << elapsed_time << "s | "
                      << "particles=" << std::setw(3) << diag.particle_count << " | "
                      << "KE=" << std::setw(8) << diag.total_kinetic_energy << "J | "
                      << "p_y=" << std::setw(8) << diag.total_momentum.y << std::endl;
        }
    }

    std::cout << "\nFinal simulation state:" << std::endl;
    context.print_diagnostics();
}

int main() {
    std::cout << "=== Phynity Phase 2 - Application Integration ===" << std::endl;
    std::cout << "Physics Context with Demo Scenarios\n" << std::endl;

    // Create a collection of scenarios to demonstrate
    std::vector<std::unique_ptr<Scenario>> scenarios;
    
    scenarios.push_back(std::make_unique<phynity::app::scenarios::GravityWell>());
    scenarios.push_back(std::make_unique<phynity::app::scenarios::ParticleSpread>());
    scenarios.push_back(std::make_unique<phynity::app::scenarios::ProjectileMotion>());
    scenarios.push_back(std::make_unique<phynity::app::scenarios::DragInteraction>());
    scenarios.push_back(std::make_unique<phynity::app::scenarios::LowGravity>());
    scenarios.push_back(std::make_unique<phynity::app::scenarios::ZeroGravity>());

    std::cout << "Available Scenarios:" << std::endl;
    for (size_t i = 0; i < scenarios.size(); ++i) {
        std::cout << "  " << (i + 1) << ". " << scenarios[i]->name() << std::endl;
    }

    // Run a subset of scenarios for the demo
    // In a full application, user would select which scenario to run
    std::cout << "\nRunning selected scenarios (5 seconds each)...\n" << std::endl;

    // Run Gravity Well
    run_scenario(std::make_unique<phynity::app::scenarios::GravityWell>(), 3.0f);

    // Run Projectile Motion
    run_scenario(std::make_unique<phynity::app::scenarios::ProjectileMotion>(), 3.0f);

    // Run Drag Interaction
    run_scenario(std::make_unique<phynity::app::scenarios::DragInteraction>(), 5.0f);

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "All scenarios completed successfully!" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    return 0;
}

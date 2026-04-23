#include "demo_scenarios.hpp"
#include "physics_context.hpp"

#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

using phynity::app::PhysicsContext;
using phynity::app::scenarios::Scenario;
using phynity::math::vectors::Vec3f;

/// Simple demo runner that executes a scenario for a specified duration
void run_scenario(std::unique_ptr<Scenario> scenario, float duration_seconds = 5.0f)
{
    std::cout << "\n" << std::string(70, '=') << '\n';
    std::cout << "Scenario: " << scenario->name() << '\n';
    std::cout << "Description: " << scenario->description() << '\n';
    std::cout << std::string(70, '=') << '\n';

    // Create physics context with default config (60 FPS)
    PhysicsContext context;

    // Setup the scenario
    scenario->setup(context);

    auto initial_diag = context.diagnostics();
    std::cout << "Initial particles: " << initial_diag.particle_count << '\n';
    std::cout << "Initial rigid bodies: " << initial_diag.body_count << '\n';
    std::cout << "Initial constraints: " << initial_diag.constraint_count << '\n';
    std::cout << "Initial force fields: " << context.force_field_count() << '\n';
    std::cout << "Running for " << duration_seconds << " seconds...\n\n";

    // Simulation loop
    const float dt = 0.016f; // ~60 FPS frame time
    const int frames = static_cast<int>(duration_seconds / dt);

    for (int frame = 0; frame < frames; ++frame)
    {
        // Update physics with frame time
        context.update(dt);

        // Call scenario step callback if needed
        scenario->step_callback(context, dt);

        // Print diagnostics every 60 frames (~1 second)
        if (frame % 60 == 0)
        {
            float elapsed_time = static_cast<float>(frame) * dt;
            auto diag = context.diagnostics();

            std::cout << std::fixed << std::setprecision(3);
            std::cout << "t=" << std::setw(5) << elapsed_time << "s | ";

            if (diag.particle_count > 0)
            {
                std::cout << "particles=" << std::setw(3) << diag.particle_count << " | ";
            }
            if (diag.body_count > 0)
            {
                std::cout << "bodies=" << std::setw(3) << diag.body_count << " | ";
            }

            std::cout << "KE=" << std::setw(8) << diag.total_kinetic_energy << "J | "
                      << "p_y=" << std::setw(8) << diag.total_momentum.y << '\n';
        }
    }

    std::cout << "\nFinal simulation state:\n";
    context.print_diagnostics();
}

int main()
{
    try
    {
        std::cout << "=== Phynity Physics Sandbox ===\n";
        std::cout << "Demo Scenarios\n\n";

        // Build scenario registry
        struct ScenarioEntry
        {
            const char *name;
            std::unique_ptr<Scenario> (*create)();
            float duration;
        };

        // clang-format off
        std::vector<ScenarioEntry> registry = {
            // Particle scenarios
            {"Gravity Well",            []() -> std::unique_ptr<Scenario> { return std::make_unique<phynity::app::scenarios::GravityWell>(); },              3.0f},
            {"Projectile Motion",       []() -> std::unique_ptr<Scenario> { return std::make_unique<phynity::app::scenarios::ProjectileMotion>(); },         3.0f},
            {"Drag Interaction",        []() -> std::unique_ptr<Scenario> { return std::make_unique<phynity::app::scenarios::DragInteraction>(); },          5.0f},
            // Rigid body scenarios
            {"Box Stacking",            []() -> std::unique_ptr<Scenario> { return std::make_unique<phynity::app::scenarios::BoxStacking>(); },              3.0f},
            {"Tower Topple",            []() -> std::unique_ptr<Scenario> { return std::make_unique<phynity::app::scenarios::TowerTopple>(); },              7.0f},
            {"Hinge Door",              []() -> std::unique_ptr<Scenario> { return std::make_unique<phynity::app::scenarios::HingeDoor>(); },                5.0f},
        };
        // clang-format on

        std::cout << "Available Scenarios:\n";
        for (size_t i = 0; i < registry.size(); ++i)
        {
            std::cout << "  " << (i + 1) << ". " << registry[i].name << '\n';
        }

        std::cout << "\nRunning all scenarios...\n";

        for (auto &entry : registry)
        {
            run_scenario(entry.create(), entry.duration);
        }

        std::cout << "\n" << std::string(70, '=') << '\n';
        std::cout << "All scenarios completed successfully!\n";
        std::cout << std::string(70, '=') << '\n';

        return 0;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Fatal error: " << ex.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "Fatal error: unknown exception\n";
    }

    return 1;
}

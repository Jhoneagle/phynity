#include "demo_scenarios.hpp"
#include "physics_context.hpp"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#if PHYNITY_HAS_RENDER
#include "sandbox_app.hpp"
#endif

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
        context.update(dt);
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

void run_headless()
{
    using namespace phynity::app::scenarios;

    std::cout << "=== Phynity Physics Sandbox (headless) ===\n\n";

    struct Entry
    {
        std::unique_ptr<Scenario> (*create)();
        float duration;
    };

    // clang-format off
    Entry entries[] = {
        {[] () -> std::unique_ptr<Scenario> { return std::make_unique<GravityWell>(); },              3.0f},
        {[] () -> std::unique_ptr<Scenario> { return std::make_unique<ProjectileMotion>(); },         3.0f},
        {[] () -> std::unique_ptr<Scenario> { return std::make_unique<DragInteraction>(); },          5.0f},
        {[] () -> std::unique_ptr<Scenario> { return std::make_unique<BoxStacking>(); },              3.0f},
        {[] () -> std::unique_ptr<Scenario> { return std::make_unique<TowerTopple>(); },              7.0f},
        {[] () -> std::unique_ptr<Scenario> { return std::make_unique<HingeDoor>(); },                5.0f},
    };
    // clang-format on

    for (auto &entry : entries)
    {
        run_scenario(entry.create(), entry.duration);
    }

    std::cout << "\n" << std::string(70, '=') << '\n';
    std::cout << "All scenarios completed successfully!\n";
    std::cout << std::string(70, '=') << '\n';
}

int main(int argc, char *argv[])
{
    try
    {
        bool headless = false;
        for (int i = 1; i < argc; ++i)
        {
            if (std::strcmp(argv[i], "--headless") == 0)
            {
                headless = true;
            }
        }

#if PHYNITY_HAS_RENDER
        if (!headless)
        {
            phynity::app::SandboxApp app;
            app.run();
            return 0;
        }
#else
        (void) headless;
#endif

        run_headless();
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

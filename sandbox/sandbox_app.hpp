#pragma once

#include "demo_scenarios.hpp"
#include "physics_context.hpp"

#include <core/diagnostics/frame_profiler.hpp>
#include <render/debug_hud.hpp>
#include <render/imgui_context.hpp>
#include <render/window.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace phynity::app
{

/// Interactive windowed sandbox application.
/// Owns the window, ImGui context, and physics context.
/// Provides scenario selection and runs the main render loop.
class SandboxApp
{
public:
    SandboxApp();

    /// Run the main loop until window is closed
    void run();

private:
    struct ScenarioEntry
    {
        std::string name;
        std::function<std::unique_ptr<scenarios::Scenario>()> create;
    };

    render::Window window_;
    render::ImGuiContext imgui_context_;
    PhysicsContext physics_context_;

    std::vector<ScenarioEntry> scenario_registry_;
    int selected_scenario_ = 0;
    int active_scenario_ = -1;
    std::unique_ptr<scenarios::Scenario> current_scenario_;

    double last_frame_time_ = 0.0;
    bool show_imgui_demo_ = false;

    // Debug HUD
    render::DebugHUD debug_hud_;
    diagnostics::FrameProfiler frame_profiler_{120};

    void register_scenarios();
    void load_scenario(int index);
    void draw_ui();
    void draw_scenario_panel();
    render::DebugHUD::State build_hud_state(float dt) const;
};

} // namespace phynity::app

#include "sandbox_app.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>

namespace phynity::app
{

SandboxApp::SandboxApp()
    : window_(1280, 720, "Phynity Sandbox"), imgui_context_(window_), last_frame_time_(window_.get_time())
{
    register_scenarios();
}

void SandboxApp::register_scenarios()
{
    // Particle scenarios
    scenario_registry_.push_back(
        {"Gravity Well", [] { return std::make_unique<scenarios::GravityWell>(); }});
    scenario_registry_.push_back(
        {"Particle Spread", [] { return std::make_unique<scenarios::ParticleSpread>(); }});
    scenario_registry_.push_back(
        {"Projectile Motion", [] { return std::make_unique<scenarios::ProjectileMotion>(); }});
    scenario_registry_.push_back(
        {"Drag Interaction", [] { return std::make_unique<scenarios::DragInteraction>(); }});
    scenario_registry_.push_back(
        {"Orbit Stability", [] { return std::make_unique<scenarios::OrbitStability>(); }});
    scenario_registry_.push_back(
        {"Multi-Particle Collision", [] { return std::make_unique<scenarios::MultiParticleCollision>(); }});
    scenario_registry_.push_back(
        {"Low Gravity", [] { return std::make_unique<scenarios::LowGravity>(); }});
    scenario_registry_.push_back(
        {"Zero Gravity", [] { return std::make_unique<scenarios::ZeroGravity>(); }});
    scenario_registry_.push_back(
        {"High Drag", [] { return std::make_unique<scenarios::HighDrag>(); }});

    // Rigid body scenarios
    scenario_registry_.push_back(
        {"Box Stacking", [] { return std::make_unique<scenarios::BoxStacking>(); }});
    scenario_registry_.push_back(
        {"Tower Topple", [] { return std::make_unique<scenarios::TowerTopple>(); }});
    scenario_registry_.push_back(
        {"Hinge Door", [] { return std::make_unique<scenarios::HingeDoor>(); }});
}

void SandboxApp::load_scenario(int index)
{
    if (index < 0 || index >= static_cast<int>(scenario_registry_.size()))
    {
        return;
    }

    // Reset physics context
    physics_context_ = PhysicsContext();

    // Create and setup new scenario
    current_scenario_ = scenario_registry_[static_cast<size_t>(index)].create();
    current_scenario_->setup(physics_context_);
    active_scenario_ = index;
}

void SandboxApp::run()
{
    while (!window_.should_close())
    {
        window_.poll_events();

        // Compute delta time
        double current_time = window_.get_time();
        auto dt = static_cast<float>(current_time - last_frame_time_);
        last_frame_time_ = current_time;

        // Clamp dt to prevent spiral-of-death on window drag/resize
        dt = std::min(dt, 0.1f);

        // Update physics
        if (current_scenario_)
        {
            physics_context_.update(dt);
            current_scenario_->step_callback(physics_context_, dt);
        }

        // Render
        int fb_width = 0;
        int fb_height = 0;
        window_.get_framebuffer_size(fb_width, fb_height);
        glViewport(0, 0, fb_width, fb_height);
        glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        imgui_context_.begin_frame();

        // Toggle ImGui demo with F2
        if (ImGui::IsKeyPressed(ImGuiKey_F2))
        {
            show_imgui_demo_ = !show_imgui_demo_;
        }
        if (show_imgui_demo_)
        {
            ImGui::ShowDemoWindow(&show_imgui_demo_);
        }

        draw_ui();

        imgui_context_.end_frame();
        window_.swap_buffers();
    }
}

void SandboxApp::draw_ui()
{
    draw_scenario_panel();
}

void SandboxApp::draw_scenario_panel()
{
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 300), ImGuiCond_FirstUseEver);

    ImGui::Begin("Scenarios");

    // Scenario dropdown
    if (ImGui::BeginCombo("Scenario", scenario_registry_[static_cast<size_t>(selected_scenario_)].name.c_str()))
    {
        for (int i = 0; i < static_cast<int>(scenario_registry_.size()); ++i)
        {
            bool is_selected = (selected_scenario_ == i);
            if (ImGui::Selectable(scenario_registry_[static_cast<size_t>(i)].name.c_str(), is_selected))
            {
                selected_scenario_ = i;
            }
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    // Load / Reset buttons
    if (ImGui::Button("Load Scenario"))
    {
        load_scenario(selected_scenario_);
    }
    ImGui::SameLine();
    if (active_scenario_ >= 0 && ImGui::Button("Reset"))
    {
        load_scenario(active_scenario_);
    }

    ImGui::Separator();

    // Show current diagnostics
    if (current_scenario_)
    {
        ImGui::Text("Active: %s", scenario_registry_[static_cast<size_t>(active_scenario_)].name.c_str());

        auto diag = physics_context_.diagnostics();
        ImGui::Text("Particles: %zu", diag.particle_count);
        ImGui::Text("Rigid Bodies: %zu", diag.body_count);
        ImGui::Text("Constraints: %zu", diag.constraint_count);
        ImGui::Text("Total KE: %.3f J", static_cast<double>(diag.total_kinetic_energy));
        ImGui::Text("Momentum: [%.2f, %.2f, %.2f]",
                     static_cast<double>(diag.total_momentum.x),
                     static_cast<double>(diag.total_momentum.y),
                     static_cast<double>(diag.total_momentum.z));
    }
    else
    {
        ImGui::TextWrapped("Select a scenario and click 'Load Scenario' to begin.");
    }

    ImGui::End();
}

} // namespace phynity::app

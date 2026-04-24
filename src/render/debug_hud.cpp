#include "debug_hud.hpp"

#include <imgui.h>

namespace phynity::render
{

void DebugHUD::draw(const State &state)
{
    if (!visible_)
    {
        return;
    }

    draw_performance_panel(state);
    draw_physics_panel(state);
    draw_simulation_panel(state);
}

void DebugHUD::draw_performance_panel(const State &state)
{
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);

    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

    if (ImGui::Begin("Performance", nullptr, flags))
    {
        ImGui::Text("FPS: %.1f", static_cast<double>(state.fps));
        ImGui::Text("Frame Time: %.2f ms", static_cast<double>(state.frame_time_ms));
        ImGui::Text("  Min: %.2f  Max: %.2f  Avg: %.2f ms",
                    static_cast<double>(state.frame_time_min_ms),
                    static_cast<double>(state.frame_time_max_ms),
                    static_cast<double>(state.frame_time_avg_ms));
        ImGui::Text("Physics Steps: %llu", static_cast<unsigned long long>(state.physics_step_count));

        if (state.zone_count > 0)
        {
            ImGui::Separator();
            ImGui::Text("Profiler Zones:");

            if (ImGui::BeginTable("zones", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV))
            {
                ImGui::TableSetupColumn("Zone", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Avg", ImGuiTableColumnFlags_WidthFixed, 65.0f);
                ImGui::TableSetupColumn("Max", ImGuiTableColumnFlags_WidthFixed, 65.0f);
                ImGui::TableHeadersRow();

                for (size_t i = 0; i < state.zone_count; ++i)
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(state.zones[i].name);
                    ImGui::TableNextColumn();
                    ImGui::Text("%.0f us", static_cast<double>(state.zones[i].avg_us));
                    ImGui::TableNextColumn();
                    ImGui::Text("%.0f us", static_cast<double>(state.zones[i].max_us));
                }
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();
}

void DebugHUD::draw_physics_panel(const State &state)
{
    const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowPos(ImVec2(viewport_size.x - 290.0f, 10.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);

    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

    if (ImGui::Begin("Physics", nullptr, flags))
    {
        if (state.particle_count > 0)
        {
            ImGui::Text("Particles: %zu", state.particle_count);
        }
        if (state.body_count > 0)
        {
            ImGui::Text("Rigid Bodies: %zu", state.body_count);
        }
        if (state.constraint_count > 0)
        {
            ImGui::Text("Constraints: %zu", state.constraint_count);
        }

        ImGui::Separator();
        ImGui::Text("Kinetic Energy: %.3f J", static_cast<double>(state.total_ke));
        if (state.body_count > 0)
        {
            ImGui::Text("  Linear KE: %.3f J", static_cast<double>(state.total_linear_ke));
            ImGui::Text("  Angular KE: %.3f J", static_cast<double>(state.total_angular_ke));
        }

        ImGui::Separator();
        ImGui::Text("Momentum: [%.2f, %.2f, %.2f]",
                    static_cast<double>(state.total_momentum_x),
                    static_cast<double>(state.total_momentum_y),
                    static_cast<double>(state.total_momentum_z));
        if (state.body_count > 0)
        {
            ImGui::Text("Angular Mom: [%.2f, %.2f, %.2f]",
                        static_cast<double>(state.total_angular_momentum_x),
                        static_cast<double>(state.total_angular_momentum_y),
                        static_cast<double>(state.total_angular_momentum_z));
        }
    }
    ImGui::End();
}

void DebugHUD::draw_simulation_panel(const State &state)
{
    const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowPos(ImVec2(10.0f, viewport_size.y - 110.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);

    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

    if (ImGui::Begin("Simulation", nullptr, flags))
    {
        ImGui::Text("Time: %.3f s", static_cast<double>(state.simulated_time));
        ImGui::Text("Timestep: %.4f s", static_cast<double>(state.timestep_size));
        ImGui::Text("Determinism: %s", state.determinism_enabled ? "ON" : "OFF");
        ImGui::Text("Workers: %u", state.worker_count);
    }
    ImGui::End();
}

} // namespace phynity::render

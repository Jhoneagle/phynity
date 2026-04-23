#include "timeline_scrubber.hpp"

#include <imgui.h>

namespace phynity::render
{

TimelineScrubber::Action TimelineScrubber::draw(const State &state)
{
    Action action;

    const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    const float bar_height = 60.0f;
    ImGui::SetNextWindowPos(ImVec2(0.0f, viewport_size.y - bar_height), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(viewport_size.x, bar_height), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.8f);

    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                       ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                       ImGuiWindowFlags_NoSavedSettings;

    if (!ImGui::Begin("##timeline_scrubber", nullptr, flags))
    {
        ImGui::End();
        return action;
    }

    // Left section: transport controls
    if (state.is_paused)
    {
        if (ImGui::Button("Play"))
        {
            action.type = ActionType::Play;
        }
    }
    else
    {
        if (ImGui::Button("Pause"))
        {
            action.type = ActionType::Pause;
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("<<"))
    {
        action.type = ActionType::StepBackward;
    }

    ImGui::SameLine();
    if (ImGui::Button(">>"))
    {
        action.type = ActionType::StepForward;
    }

    ImGui::SameLine();
    if (state.is_paused)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
        ImGui::Text("PAUSED");
        ImGui::PopStyleColor();
        ImGui::SameLine();
    }

    // Center: frame slider
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 200.0f);
    int frame = static_cast<int>(state.current_frame);
    int max_frame = state.timeline_size > 0 ? static_cast<int>(state.timeline_size) - 1 : 0;
    if (ImGui::SliderInt("##frame", &frame, 0, max_frame, "Frame %d"))
    {
        if (frame >= 0 && static_cast<size_t>(frame) != state.current_frame)
        {
            action.type = ActionType::SeekToFrame;
            action.target_frame = static_cast<size_t>(frame);
        }
    }

    // Right section: speed selector and time display
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    static const float speeds[] = {0.25f, 0.5f, 1.0f, 2.0f, 4.0f};
    static const char *speed_labels[] = {"0.25x", "0.5x", "1x", "2x", "4x"};
    int current_speed_idx = 2; // default 1x
    for (int i = 0; i < 5; ++i)
    {
        if (state.speed_multiplier >= speeds[i] - 0.01f && state.speed_multiplier <= speeds[i] + 0.01f)
        {
            current_speed_idx = i;
        }
    }

    if (ImGui::BeginCombo("##speed", speed_labels[current_speed_idx], ImGuiComboFlags_NoArrowButton))
    {
        for (int i = 0; i < 5; ++i)
        {
            bool selected = (i == current_speed_idx);
            if (ImGui::Selectable(speed_labels[i], selected))
            {
                action.type = ActionType::SetSpeed;
                action.target_speed = speeds[i];
            }
        }
        ImGui::EndCombo();
    }

    ImGui::SameLine();
    ImGui::Text("%.2fs", static_cast<double>(state.simulated_time));

    // Keyboard shortcuts
    if (!ImGui::GetIO().WantTextInput)
    {
        if (ImGui::IsKeyPressed(ImGuiKey_Space))
        {
            action.type = state.is_paused ? ActionType::Play : ActionType::Pause;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow) && state.is_paused)
        {
            action.type = ActionType::StepBackward;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow) && state.is_paused)
        {
            action.type = ActionType::StepForward;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket))
        {
            int new_idx = (current_speed_idx > 0) ? current_speed_idx - 1 : 0;
            action.type = ActionType::SetSpeed;
            action.target_speed = speeds[new_idx];
        }
        if (ImGui::IsKeyPressed(ImGuiKey_RightBracket))
        {
            int new_idx = (current_speed_idx < 4) ? current_speed_idx + 1 : 4;
            action.type = ActionType::SetSpeed;
            action.target_speed = speeds[new_idx];
        }
    }

    ImGui::End();
    return action;
}

} // namespace phynity::render

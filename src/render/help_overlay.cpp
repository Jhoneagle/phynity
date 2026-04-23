#include "help_overlay.hpp"

#include <imgui.h>

namespace phynity::render
{

void HelpOverlay::draw()
{
    if (!visible_)
    {
        return;
    }

    const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowPos(ImVec2(viewport_size.x * 0.5f, viewport_size.y * 0.5f), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(360, 380), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Keyboard Shortcuts", &visible_))
    {
        ImGui::Text("General");
        ImGui::Separator();
        ImGui::BulletText("F1 - Toggle this help");
        ImGui::BulletText("F2 - Toggle ImGui demo window");
        ImGui::BulletText("F3 - Toggle debug HUD");

        ImGui::Spacing();
        ImGui::Text("Timeline");
        ImGui::Separator();
        ImGui::BulletText("Space - Play / Pause");
        ImGui::BulletText("Left Arrow - Step backward (paused)");
        ImGui::BulletText("Right Arrow - Step forward (paused)");
        ImGui::BulletText("[ / ] - Decrease / Increase speed");

        ImGui::Spacing();
        ImGui::Text("Camera");
        ImGui::Separator();
        ImGui::BulletText("Left Drag - Orbit");
        ImGui::BulletText("Right Drag - Pan");
        ImGui::BulletText("Scroll - Zoom");
        ImGui::BulletText("Home - Reset camera");

        ImGui::Spacing();
        ImGui::Text("Selection");
        ImGui::Separator();
        ImGui::BulletText("Left Click - Select body (opens inspector)");
        ImGui::BulletText("Click empty space - Deselect");
    }
    ImGui::End();
}

} // namespace phynity::render

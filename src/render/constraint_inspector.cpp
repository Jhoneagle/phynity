#include "constraint_inspector.hpp"

#include <cstdio>
#include <imgui.h>

namespace phynity::render
{

void ConstraintInspector::draw(const ConstraintInspectorData &data, bool &open)
{
    char title[64];
    std::snprintf(title, sizeof(title), "Constraint #%zu###constraint_%zu", data.index, data.index);

    ImGui::SetNextWindowSize(ImVec2(280, 150), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin(title, &open))
    {
        ImGui::End();
        return;
    }

    ImGui::Text("Type: %s", data.type_name.c_str());
    ImGui::Text("Active: %s", data.active ? "yes" : "no");
    ImGui::Text("Error: %.6f", static_cast<double>(data.error));

    ImGui::End();
}

} // namespace phynity::render

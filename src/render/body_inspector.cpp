#include "body_inspector.hpp"

#include <cstdio>
#include <imgui.h>

namespace phynity::render
{

namespace
{
const char *shape_type_name(int type)
{
    switch (type)
    {
        case 0:
            return "Sphere";
        case 1:
            return "Box";
        case 2:
            return "Capsule";
        default:
            return "Unknown";
    }
}
} // namespace

BodyInspectorEdit BodyInspector::draw(const BodyInspectorData &data, bool is_paused, bool &open)
{
    BodyInspectorEdit edit;

    char title[64];
    std::snprintf(title, sizeof(title), "Body #%d###body_%d", data.id, data.id);

    ImGui::SetNextWindowSize(ImVec2(340, 500), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin(title, &open))
    {
        ImGui::End();
        return edit;
    }

    ImGui::Text(
        "ID: %d  Active: %s  Lifetime: %.2f", data.id, data.active ? "yes" : "no", static_cast<double>(data.lifetime));

    // Transform section
    if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen))
    {
        float pos[3] = {data.position.x, data.position.y, data.position.z};
        if (is_paused)
        {
            if (ImGui::DragFloat3("Position", pos, 0.1f))
            {
                edit.new_position = Vec3f(pos[0], pos[1], pos[2]);
            }
        }
        else
        {
            ImGui::Text("Position: [%.3f, %.3f, %.3f]",
                        static_cast<double>(data.position.x),
                        static_cast<double>(data.position.y),
                        static_cast<double>(data.position.z));
        }

        ImGui::Text("Orientation: [%.3f, %.3f, %.3f, %.3f]",
                    static_cast<double>(data.orientation.w),
                    static_cast<double>(data.orientation.x),
                    static_cast<double>(data.orientation.y),
                    static_cast<double>(data.orientation.z));
        ImGui::Text("Euler (deg): [%.1f, %.1f, %.1f]",
                    static_cast<double>(data.euler_angles.x),
                    static_cast<double>(data.euler_angles.y),
                    static_cast<double>(data.euler_angles.z));
    }

    // Dynamics section
    if (ImGui::CollapsingHeader("Dynamics", ImGuiTreeNodeFlags_DefaultOpen))
    {
        float vel[3] = {data.velocity.x, data.velocity.y, data.velocity.z};
        if (is_paused)
        {
            if (ImGui::DragFloat3("Velocity", vel, 0.1f))
            {
                edit.new_velocity = Vec3f(vel[0], vel[1], vel[2]);
            }
        }
        else
        {
            ImGui::Text("Velocity: [%.3f, %.3f, %.3f]",
                        static_cast<double>(data.velocity.x),
                        static_cast<double>(data.velocity.y),
                        static_cast<double>(data.velocity.z));
        }

        ImGui::Text("Force: [%.3f, %.3f, %.3f]",
                    static_cast<double>(data.force_accumulator.x),
                    static_cast<double>(data.force_accumulator.y),
                    static_cast<double>(data.force_accumulator.z));

        float mass = data.mass;
        if (is_paused && data.mass > 0.0f)
        {
            if (ImGui::DragFloat("Mass", &mass, 0.1f, 0.01f, 1000.0f))
            {
                edit.new_mass = mass;
            }
        }
        else
        {
            ImGui::Text(
                "Mass: %.3f kg  (inv: %.6f)", static_cast<double>(data.mass), static_cast<double>(data.inv_mass));
        }

        ImGui::Separator();

        float ang_vel[3] = {data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z};
        if (is_paused)
        {
            if (ImGui::DragFloat3("Ang Velocity", ang_vel, 0.1f))
            {
                edit.new_angular_velocity = Vec3f(ang_vel[0], ang_vel[1], ang_vel[2]);
            }
        }
        else
        {
            ImGui::Text("Ang Velocity: [%.3f, %.3f, %.3f]",
                        static_cast<double>(data.angular_velocity.x),
                        static_cast<double>(data.angular_velocity.y),
                        static_cast<double>(data.angular_velocity.z));
        }

        ImGui::Text("Torque: [%.3f, %.3f, %.3f]",
                    static_cast<double>(data.torque_accumulator.x),
                    static_cast<double>(data.torque_accumulator.y),
                    static_cast<double>(data.torque_accumulator.z));
    }

    // Shape section
    if (ImGui::CollapsingHeader("Shape"))
    {
        ImGui::Text("Type: %s", shape_type_name(data.shape_type));
        ImGui::Text("Dimensions: [%.3f, %.3f, %.3f]",
                    static_cast<double>(data.shape_dimensions.x),
                    static_cast<double>(data.shape_dimensions.y),
                    static_cast<double>(data.shape_dimensions.z));
        ImGui::Text("Bounding Radius: %.3f", static_cast<double>(data.bounding_radius));
    }

    // Material section
    if (ImGui::CollapsingHeader("Material"))
    {
        ImGui::Text("Restitution: %.3f", static_cast<double>(data.restitution));
        ImGui::Text("Friction: %.3f", static_cast<double>(data.friction));
        ImGui::Text("Linear Damping: %.4f", static_cast<double>(data.linear_damping));
        ImGui::Text("Angular Damping: %.4f", static_cast<double>(data.angular_damping));
    }

    // Energy section
    if (ImGui::CollapsingHeader("Energy"))
    {
        ImGui::Text("Linear KE: %.4f J", static_cast<double>(data.linear_ke));
        ImGui::Text("Angular KE: %.4f J", static_cast<double>(data.angular_ke));
        ImGui::Text("Total KE: %.4f J", static_cast<double>(data.total_ke));
        ImGui::Text("Lin Momentum: [%.3f, %.3f, %.3f]",
                    static_cast<double>(data.linear_momentum.x),
                    static_cast<double>(data.linear_momentum.y),
                    static_cast<double>(data.linear_momentum.z));
        ImGui::Text("Ang Momentum: [%.3f, %.3f, %.3f]",
                    static_cast<double>(data.angular_momentum.x),
                    static_cast<double>(data.angular_momentum.y),
                    static_cast<double>(data.angular_momentum.z));
    }

    ImGui::End();
    return edit;
}

} // namespace phynity::render

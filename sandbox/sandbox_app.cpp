#include "sandbox_app.hpp"

#include <core/physics/shapes/box.hpp>
#include <core/physics/shapes/capsule.hpp>
#include <core/physics/shapes/sphere.hpp>

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <cstring>

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
        frame_profiler_.begin_frame();

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
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Handle camera input
        handle_camera_input();

        // Draw 3D scene
        if (current_scenario_)
        {
            auto scene_state = build_scene_state();
            scene_renderer_.draw(camera_, scene_state, fb_width, fb_height);
            handle_picking(fb_width, fb_height);
        }

        imgui_context_.begin_frame();

        // Keyboard shortcuts
        if (ImGui::IsKeyPressed(ImGuiKey_F2))
        {
            show_imgui_demo_ = !show_imgui_demo_;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F3))
        {
            debug_hud_.toggle();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Home))
        {
            camera_.reset();
        }

        if (show_imgui_demo_)
        {
            ImGui::ShowDemoWindow(&show_imgui_demo_);
        }

        draw_ui();

        // Draw HUD
        auto hud_state = build_hud_state(dt);
        debug_hud_.draw(hud_state);

        imgui_context_.end_frame();
        window_.swap_buffers();

        frame_profiler_.end_frame();
    }
}

void SandboxApp::draw_ui()
{
    draw_scenario_panel();

    // Timeline scrubber
    if (current_scenario_)
    {
        render::TimelineScrubber::State ts_state;
        ts_state.timeline_size = physics_context_.timeline_size();
        ts_state.current_frame = physics_context_.current_frame_index();
        ts_state.is_paused = physics_context_.is_paused();
        ts_state.speed_multiplier = physics_context_.speed();
        ts_state.simulated_time = physics_context_.timestep_statistics().accumulated_time;

        auto action = timeline_scrubber_.draw(ts_state);

        switch (action.type)
        {
            case render::TimelineScrubber::ActionType::Play:
                physics_context_.resume();
                break;
            case render::TimelineScrubber::ActionType::Pause:
                physics_context_.pause();
                break;
            case render::TimelineScrubber::ActionType::StepForward:
                physics_context_.step_forward();
                break;
            case render::TimelineScrubber::ActionType::StepBackward:
                physics_context_.step_backward();
                break;
            case render::TimelineScrubber::ActionType::SeekToFrame:
                physics_context_.pause();
                physics_context_.seek_to_frame(action.target_frame);
                break;
            case render::TimelineScrubber::ActionType::SetSpeed:
                physics_context_.set_speed(action.target_speed);
                break;
            case render::TimelineScrubber::ActionType::None:
                break;
        }
    }
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

render::SceneRenderer::State SandboxApp::build_scene_state() const
{
    render::SceneRenderer::State state;

    // Rigid bodies
    const auto &bodies = physics_context_.rigid_body_system().bodies();
    state.bodies.reserve(bodies.size());
    for (const auto &rb : bodies)
    {
        render::SceneRenderer::BodyVisual vis;
        vis.position = rb.position;
        vis.orientation = rb.orientation;
        vis.id = rb.id;
        vis.selected = (selected_body_id_.has_value() && selected_body_id_.value() == rb.id);

        if (rb.shape)
        {
            switch (rb.shape->get_type())
            {
                case phynity::physics::shapes::ShapeType::Sphere:
                    vis.type = render::SceneRenderer::ShapeType::Sphere;
                    vis.dimensions =
                        Vec3f(static_cast<const phynity::physics::shapes::SphereShape *>(rb.shape.get())->radius,
                              0.0f,
                              0.0f);
                    break;
                case phynity::physics::shapes::ShapeType::Box:
                    vis.type = render::SceneRenderer::ShapeType::Box;
                    vis.dimensions =
                        static_cast<const phynity::physics::shapes::BoxShape *>(rb.shape.get())->half_extents;
                    break;
                case phynity::physics::shapes::ShapeType::Capsule:
                {
                    vis.type = render::SceneRenderer::ShapeType::Capsule;
                    const auto *cap = static_cast<const phynity::physics::shapes::CapsuleShape *>(rb.shape.get());
                    vis.dimensions = Vec3f(cap->radius, cap->half_height, 0.0f);
                    break;
                }
                default:
                    vis.type = render::SceneRenderer::ShapeType::Unknown;
                    break;
            }
        }

        state.bodies.push_back(vis);
    }

    // Particles
    const auto &particles = physics_context_.particle_system().particles();
    state.particles.reserve(particles.size());
    for (const auto &p : particles)
    {
        render::SceneRenderer::ParticleVisual vis;
        vis.position = p.position;
        vis.radius = p.radius;
        state.particles.push_back(vis);
    }

    // Constraints (draw lines between connected body pairs)
    const auto &constraints = physics_context_.rigid_body_system().get_constraints();
    for (const auto &c : constraints)
    {
        if (!c)
            continue;
        // Constraints store body pointers; we approximate by showing a line between bodies
        // This is a simplified visualization
        render::SceneRenderer::ConstraintVisual vis;
        vis.pos_a = Vec3f(0.0f); // Will be filled if we can access body pointers
        vis.pos_b = Vec3f(0.0f);
        state.constraints.push_back(vis);
    }

    return state;
}

void SandboxApp::handle_picking(int fb_width, int fb_height)
{
    // Only pick on left-click when ImGui doesn't want the mouse
    if (ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::GetIO().KeyCtrl)
    {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        auto scene_state = build_scene_state();
        auto result =
            render::pick_body(camera_, mouse_pos.x, mouse_pos.y, fb_width, fb_height, scene_state);

        if (result.has_value())
        {
            selected_body_id_ = result->body_id;
        }
        else
        {
            selected_body_id_.reset();
        }
    }
}

void SandboxApp::handle_camera_input()
{
    // Don't handle camera when ImGui wants the mouse
    if (ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    ImVec2 mouse_pos = ImGui::GetMousePos();

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        camera_.on_mouse_button(0, true, mouse_pos.x, mouse_pos.y);
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {
        camera_.on_mouse_button(0, false, mouse_pos.x, mouse_pos.y);
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        camera_.on_mouse_button(1, true, mouse_pos.x, mouse_pos.y);
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
    {
        camera_.on_mouse_button(1, false, mouse_pos.x, mouse_pos.y);
    }

    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) || ImGui::IsMouseDragging(ImGuiMouseButton_Right))
    {
        camera_.on_mouse_move(mouse_pos.x, mouse_pos.y);
    }

    float scroll = ImGui::GetIO().MouseWheel;
    if (scroll != 0.0f)
    {
        camera_.on_scroll(scroll);
    }
}

render::DebugHUD::State SandboxApp::build_hud_state(float dt) const
{
    render::DebugHUD::State state;

    // Performance
    state.fps = (dt > 0.0f) ? 1.0f / dt : 0.0f;
    state.frame_time_ms = dt * 1000.0f;
    state.frame_time_min_ms = static_cast<float>(frame_profiler_.get_min_frame_time(60)) / 1000.0f;
    state.frame_time_max_ms = static_cast<float>(frame_profiler_.get_max_frame_time(60)) / 1000.0f;
    state.frame_time_avg_ms = static_cast<float>(frame_profiler_.get_average_frame_time(60)) / 1000.0f;

    // Physics diagnostics
    auto diag = physics_context_.diagnostics();
    state.particle_count = diag.particle_count;
    state.body_count = diag.body_count;
    state.constraint_count = diag.constraint_count;
    state.total_ke = diag.total_kinetic_energy;
    state.total_linear_ke = diag.total_linear_ke;
    state.total_angular_ke = diag.total_angular_ke;
    state.total_momentum_x = diag.total_momentum.x;
    state.total_momentum_y = diag.total_momentum.y;
    state.total_momentum_z = diag.total_momentum.z;
    state.total_angular_momentum_x = diag.total_angular_momentum.x;
    state.total_angular_momentum_y = diag.total_angular_momentum.y;
    state.total_angular_momentum_z = diag.total_angular_momentum.z;

    // Simulation state
    auto ts_stats = physics_context_.timestep_statistics();
    state.physics_step_count = ts_stats.total_steps;
    state.simulated_time = ts_stats.accumulated_time;
    state.timestep_size = physics_context_.target_timestep();
    state.determinism_enabled = physics_context_.config().use_determinism;
    state.worker_count = physics_context_.config().job_workers;

    // Profiler zones — pick the key high-level zones
    static const char *zone_names[] = {
        "RigidBodySystem::update",
        "RigidBodySystem::collisions",
        "RigidBodySystem::constraint_solving",
        "integration",
        "collision_resolution",
        "apply_force_fields",
    };

    state.zone_count = 0;
    for (const char *name : zone_names)
    {
        if (state.zone_count >= render::DebugHUD::MAX_ZONES)
        {
            break;
        }
        auto stats = frame_profiler_.get_zone_stats(name, 60);
        if (stats.call_count > 0)
        {
            auto &zone = state.zones[state.zone_count];
            zone.name = name;
            zone.avg_us = static_cast<float>(stats.average_duration_us());
            zone.max_us = static_cast<float>(stats.max_duration_us);
            ++state.zone_count;
        }
    }

    return state;
}

} // namespace phynity::app

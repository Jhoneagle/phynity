#pragma once

#include <cstddef>
#include <cstdint>

namespace phynity::render
{

/// Debug HUD rendering on-screen overlays with physics and performance data.
/// Uses plain data structs to stay decoupled from core physics types.
class DebugHUD
{
public:
    static constexpr size_t MAX_ZONES = 16;

    struct ZoneInfo
    {
        const char *name = "";
        float avg_us = 0.0f;
        float max_us = 0.0f;
    };

    struct State
    {
        // Performance
        float fps = 0.0f;
        float frame_time_ms = 0.0f;
        float frame_time_min_ms = 0.0f;
        float frame_time_max_ms = 0.0f;
        float frame_time_avg_ms = 0.0f;

        // Physics objects
        size_t particle_count = 0;
        size_t body_count = 0;
        size_t constraint_count = 0;

        // Energy and momentum
        float total_ke = 0.0f;
        float total_linear_ke = 0.0f;
        float total_angular_ke = 0.0f;
        float total_momentum_x = 0.0f;
        float total_momentum_y = 0.0f;
        float total_momentum_z = 0.0f;
        float total_angular_momentum_x = 0.0f;
        float total_angular_momentum_y = 0.0f;
        float total_angular_momentum_z = 0.0f;

        // Simulation
        uint64_t physics_step_count = 0;
        float simulated_time = 0.0f;
        float timestep_size = 0.0f;
        bool determinism_enabled = false;
        uint32_t worker_count = 0;

        // Profiler zones
        ZoneInfo zones[MAX_ZONES] = {};
        size_t zone_count = 0;
    };

    /// Draw the HUD panels. Call between ImGui begin_frame/end_frame.
    void draw(const State &state);

    /// Toggle HUD visibility
    void toggle()
    {
        visible_ = !visible_;
    }

    bool is_visible() const
    {
        return visible_;
    }

private:
    bool visible_ = true;

    void draw_performance_panel(const State &state);
    void draw_physics_panel(const State &state);
    void draw_simulation_panel(const State &state);
};

} // namespace phynity::render

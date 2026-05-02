#pragma once

#include <cstddef>

namespace phynity::render
{

/// Timeline scrubber widget for simulation transport controls.
class TimelineScrubber
{
public:
    struct State
    {
        size_t timeline_size = 0;
        size_t current_frame = 0;
        bool is_paused = false;
        float speed_multiplier = 1.0f;
        float simulated_time = 0.0f;
    };

    enum class ActionType
    {
        None,
        Play,
        Pause,
        StepForward,
        StepBackward,
        SeekToFrame,
        SetSpeed,
    };

    struct Action
    {
        ActionType type = ActionType::None;
        size_t target_frame = 0;
        float target_speed = 1.0f;
    };

    /// Draw the scrubber widget. Returns the action requested by the user.
    Action draw(const State &state);
};

} // namespace phynity::render

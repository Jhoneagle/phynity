#pragma once

#include "schedule_recorder.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace phynity::jobs
{

/// Loads a recorded schedule for deterministic replay.
/// In the new system, replay schedules are consumed by the JobSystem's
/// deterministic mode to execute tasks in the exact recorded order.
class ScheduleReplayer
{
public:
    bool load(const std::string &path);

    [[nodiscard]] bool has_frame(uint32_t frame_index) const;

    /// Get the recorded task execution order for a given frame.
    /// Returns the task records sorted by start_order.
    [[nodiscard]] const std::vector<ScheduleRecorder::TaskRecord> *frame_tasks(uint32_t frame_index) const;

    [[nodiscard]] uint32_t frame_count() const
    {
        return static_cast<uint32_t>(frames_.size());
    }

    void clear();

private:
    std::vector<ScheduleRecorder::FrameRecord> frames_;
    std::unordered_map<uint32_t, size_t> frame_index_map_; // frame_index -> frames_ index
};

} // namespace phynity::jobs

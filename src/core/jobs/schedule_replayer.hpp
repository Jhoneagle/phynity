#pragma once

#include "schedule_recorder.hpp"
#include "task_schedule.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace phynity::jobs
{

/// Loads a recorded schedule and produces TaskSchedules that replay
/// the exact execution order from the recording.
class ScheduleReplayer
{
public:
    bool load(const std::string &path);

    [[nodiscard]] bool has_frame(uint32_t frame_index) const;

    /// Produce a TaskSchedule where tasks are sequenced in the exact
    /// recorded execution order. All tasks get tier 0 (serial replay).
    [[nodiscard]] TaskSchedule replay_schedule(uint32_t frame_index) const;

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

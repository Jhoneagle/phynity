#pragma once

#include "task_id.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace phynity::jobs
{

/// Records task execution order per frame for deterministic replay.
/// Thread-safe: record_task_start/end can be called from worker threads.
class ScheduleRecorder
{
public:
    struct TaskRecord
    {
        uint32_t task_id;
        uint32_t worker_index;
        uint32_t start_order; // global execution sequence
    };

    struct FrameRecord
    {
        uint32_t frame_index;
        std::vector<TaskRecord> tasks;
    };

    void begin_frame(uint32_t frame_index);
    void record_task_start(TaskId id, uint32_t worker_index);
    void end_frame();

    [[nodiscard]] const std::vector<FrameRecord> &frames() const
    {
        return frames_;
    }

    bool save(const std::string &path) const;
    void clear();

private:
    std::vector<FrameRecord> frames_;
    FrameRecord current_frame_{};
    std::atomic<uint32_t> execution_counter_{0};
    std::mutex record_mutex_;
    std::atomic<bool> in_frame_{false};
};

} // namespace phynity::jobs

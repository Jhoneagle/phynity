#include "schedule_recorder.hpp"
#include "schedule_schema.hpp"

#include <fstream>

namespace phynity::jobs
{

void ScheduleRecorder::begin_frame(uint32_t frame_index)
{
    current_frame_ = FrameRecord{};
    current_frame_.frame_index = frame_index;
    execution_counter_.store(0, std::memory_order_relaxed);
    in_frame_ = true;
}

void ScheduleRecorder::record_task_start(TaskId id, uint32_t worker_index)
{
    if (!in_frame_)
    {
        return;
    }

    uint32_t order = execution_counter_.fetch_add(1, std::memory_order_acq_rel);

    std::lock_guard<std::mutex> lock(record_mutex_);
    current_frame_.tasks.push_back({.task_id = id.value, .worker_index = worker_index, .start_order = order});
}

void ScheduleRecorder::end_frame()
{
    if (!in_frame_)
    {
        return;
    }

    in_frame_ = false;

    // Sort by start_order for consistent output
    auto &tasks = current_frame_.tasks;
    std::sort(tasks.begin(), tasks.end(),
              [](const TaskRecord &a, const TaskRecord &b) { return a.start_order < b.start_order; });

    frames_.push_back(std::move(current_frame_));
}

bool ScheduleRecorder::save(const std::string &path) const
{
    std::ofstream out(path, std::ios::binary);
    if (!out)
    {
        return false;
    }

    out.write(reinterpret_cast<const char *>(&schedule_file_magic), sizeof(schedule_file_magic));
    out.write(reinterpret_cast<const char *>(&schedule_format_version), sizeof(schedule_format_version));
    uint32_t frame_count = static_cast<uint32_t>(frames_.size());
    out.write(reinterpret_cast<const char *>(&frame_count), sizeof(frame_count));

    for (const auto &frame : frames_)
    {
        out.write(reinterpret_cast<const char *>(&frame.frame_index), sizeof(frame.frame_index));
        uint32_t task_count = static_cast<uint32_t>(frame.tasks.size());
        out.write(reinterpret_cast<const char *>(&task_count), sizeof(task_count));

        // Bulk write: TaskRecord is a POD of three uint32_t fields
        if (task_count > 0)
        {
            out.write(reinterpret_cast<const char *>(frame.tasks.data()),
                      static_cast<std::streamsize>(task_count * sizeof(TaskRecord)));
        }
    }

    return out.good();
}

void ScheduleRecorder::clear()
{
    frames_.clear();
    current_frame_ = FrameRecord{};
    execution_counter_.store(0, std::memory_order_relaxed);
    in_frame_ = false;
}

} // namespace phynity::jobs

#include "schedule_replayer.hpp"
#include "schedule_schema.hpp"

#include <fstream>

namespace phynity::jobs
{

bool ScheduleReplayer::load(const std::string &path)
{
    clear();

    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        return false;
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t frame_count = 0;

    in.read(reinterpret_cast<char *>(&magic), sizeof(magic));
    in.read(reinterpret_cast<char *>(&version), sizeof(version));
    in.read(reinterpret_cast<char *>(&frame_count), sizeof(frame_count));

    if (!in.good() || magic != schedule_file_magic || version != schedule_format_version)
    {
        return false;
    }

    for (uint32_t f = 0; f < frame_count; ++f)
    {
        ScheduleRecorder::FrameRecord frame{};

        in.read(reinterpret_cast<char *>(&frame.frame_index), sizeof(frame.frame_index));
        uint32_t task_count = 0;
        in.read(reinterpret_cast<char *>(&task_count), sizeof(task_count));

        if (!in.good())
        {
            return false;
        }

        // Bulk read: TaskRecord is a POD of three uint32_t fields
        frame.tasks.resize(task_count);
        if (task_count > 0)
        {
            in.read(reinterpret_cast<char *>(frame.tasks.data()),
                    static_cast<std::streamsize>(task_count * sizeof(ScheduleRecorder::TaskRecord)));
        }

        if (!in.good() && !in.eof())
        {
            return false;
        }

        frame_index_map_[frame.frame_index] = frames_.size();
        frames_.push_back(std::move(frame));
    }

    return true;
}

bool ScheduleReplayer::has_frame(uint32_t frame_index) const
{
    return frame_index_map_.contains(frame_index);
}

TaskSchedule ScheduleReplayer::replay_schedule(uint32_t frame_index) const
{
    TaskSchedule schedule;

    auto it = frame_index_map_.find(frame_index);
    if (it == frame_index_map_.end())
    {
        return schedule;
    }

    const auto &frame = frames_[it->second];

    // Tasks are already sorted by start_order (ScheduleRecorder sorts on end_frame).
    // Each task gets its own tier to force serial execution in recorded order.
    schedule.entries.reserve(frame.tasks.size());
    for (uint32_t i = 0; i < frame.tasks.size(); ++i)
    {
        schedule.entries.push_back(
            {.id = TaskId{frame.tasks[i].task_id}, .tier = i, .sequence = i});
    }
    schedule.tier_count = static_cast<uint32_t>(frame.tasks.size());

    return schedule;
}

void ScheduleReplayer::clear()
{
    frames_.clear();
    frame_index_map_.clear();
}

} // namespace phynity::jobs

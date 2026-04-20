#pragma once

#include "job_handle.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace phynity::jobs
{

enum class SchedulingMode : uint8_t
{
    Concurrent,
    Deterministic,
    DeterministicReplay // Replay a previously recorded schedule for bit-identical results
};

struct JobSystemConfig
{
    uint32_t worker_count = 0; // 0 = use hardware concurrency
    SchedulingMode mode = SchedulingMode::Concurrent;
    uint32_t queue_capacity = 1024;
};

using JobFn = std::function<void()>;

class JobSystem
{
public:
    JobSystem() = default;
    explicit JobSystem(const JobSystemConfig &config);

    JobSystem(const JobSystem &) = delete;
    JobSystem &operator=(const JobSystem &) = delete;

    void start(const JobSystemConfig &config);
    void shutdown();

    [[nodiscard]] bool is_running() const noexcept;
    [[nodiscard]] uint32_t worker_count() const noexcept;
    [[nodiscard]] SchedulingMode scheduling_mode() const noexcept;

    JobHandle submit(JobFn job);
    JobHandle submit_to_worker(uint32_t worker_index, JobFn job);
    void wait(JobHandle handle);
    void wait_all(std::span<const JobHandle> handles);

    template <typename IndexFn> void parallel_for(uint32_t begin, uint32_t end, uint32_t grain, IndexFn &&fn)
    {
        if (begin >= end)
        {
            return;
        }

        uint32_t count = end - begin;
        uint32_t effective_grain = (grain > 0) ? grain : count;

        // Serial fallback: system not running, small range, or deterministic/replay mode
        if (!is_running() || count <= effective_grain || scheduling_mode() == SchedulingMode::Deterministic ||
            scheduling_mode() == SchedulingMode::DeterministicReplay)
        {
            for (uint32_t i = begin; i < end; ++i)
            {
                fn(i);
            }
            return;
        }

        // Divide [begin, end) into chunks of size `grain` and submit each as a job
        std::vector<JobHandle> handles;
        handles.reserve((count + effective_grain - 1) / effective_grain);

        for (uint32_t chunk_begin = begin; chunk_begin < end; chunk_begin += effective_grain)
        {
            uint32_t chunk_end = std::min(chunk_begin + effective_grain, end);
            handles.push_back(submit(
                [chunk_begin, chunk_end, &fn]()
                {
                    for (uint32_t i = chunk_begin; i < chunk_end; ++i)
                    {
                        fn(i);
                    }
                }));
        }

        wait_all(handles);
    }

private:
    JobSystemConfig config_{};
};

} // namespace phynity::jobs

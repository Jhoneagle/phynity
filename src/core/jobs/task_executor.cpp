#include "task_executor.hpp"

#include <cassert>
#include <limits>
#include <vector>

namespace phynity::jobs
{

TaskExecutor::TaskExecutor(JobSystem &job_system) : job_system_(job_system)
{
}

void TaskExecutor::execute(const TaskGraph &graph)
{
    assert(graph.validate());
    auto schedule = TaskScheduler::build_schedule(graph);
    execute(schedule, graph);
}

void TaskExecutor::execute(const TaskSchedule &schedule, const TaskGraph &graph)
{
    if (schedule.entries.empty())
    {
        return;
    }

    if (!job_system_.is_running() || job_system_.scheduling_mode() == SchedulingMode::Deterministic ||
        job_system_.scheduling_mode() == SchedulingMode::DeterministicReplay)
    {
        execute_serial(schedule, graph);
        return;
    }

    execute_concurrent(schedule, graph);
}

void TaskExecutor::execute_serial(const TaskSchedule &schedule, const TaskGraph &graph)
{
    for (const auto &entry : schedule.entries)
    {
        const auto &desc = graph.descriptor(entry.id);
        if (desc.fn)
        {
            desc.fn();
        }
    }
}

void TaskExecutor::execute_concurrent(const TaskSchedule &schedule, const TaskGraph &graph)
{
    // Process one tier at a time: submit all tasks in the tier, then wait_all
    uint32_t current_tier = 0;
    std::vector<JobHandle> handles;

    for (const auto &entry : schedule.entries)
    {
        // When we move to a new tier, wait for all tasks in the previous tier
        if (entry.tier != current_tier)
        {
            job_system_.wait_all(handles);
            handles.clear();
            current_tier = entry.tier;
        }

        const auto &desc = graph.descriptor(entry.id);
        if (desc.fn)
        {
            if (desc.affinity_hint != std::numeric_limits<uint32_t>::max())
            {
                handles.push_back(job_system_.submit_to_worker(desc.affinity_hint, desc.fn));
            }
            else
            {
                handles.push_back(job_system_.submit(desc.fn));
            }
        }
    }

    // Wait for the final tier
    if (!handles.empty())
    {
        job_system_.wait_all(handles);
    }
}

} // namespace phynity::jobs

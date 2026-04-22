#include "task_scheduler.hpp"

#include <algorithm>
#include <cassert>

namespace phynity::jobs
{

TaskSchedule TaskScheduler::build_schedule(const TaskGraph &graph)
{
    const uint32_t n = graph.task_count();
    TaskSchedule schedule;

    if (n == 0)
    {
        return schedule;
    }

    // Mutable copy of predecessor counts for Kahn's algorithm
    std::vector<uint32_t> in_degree(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        in_degree[i] = graph.predecessor_count(TaskId{i});
    }

    // Seed the first wavefront with all roots, sorted by TaskId for determinism
    std::vector<TaskId> current_tier;
    for (uint32_t i = 0; i < n; ++i)
    {
        if (in_degree[i] == 0)
        {
            current_tier.push_back(TaskId{i});
        }
    }
    std::sort(current_tier.begin(), current_tier.end());

    schedule.entries.reserve(n);
    uint32_t tier = 0;
    uint32_t sequence = 0;

    while (!current_tier.empty())
    {
        std::vector<TaskId> next_tier;

        for (const auto &id : current_tier)
        {
            schedule.entries.push_back({.id = id, .tier = tier, .sequence = sequence++});

            for (const auto &child : graph.dependents(id))
            {
                assert(in_degree[child.value] > 0);
                --in_degree[child.value];
                if (in_degree[child.value] == 0)
                {
                    next_tier.push_back(child);
                }
            }
        }

        // Sort next tier by TaskId for deterministic ordering
        std::sort(next_tier.begin(), next_tier.end());

        current_tier = std::move(next_tier);
        ++tier;
    }

    schedule.tier_count = tier;

    // Verify all tasks were scheduled (no cycles missed by caller)
    assert(schedule.entries.size() == n);

    return schedule;
}

} // namespace phynity::jobs

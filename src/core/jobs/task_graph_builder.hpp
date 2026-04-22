#pragma once

#include "task_graph.hpp"
#include "task_id.hpp"

#include <cassert>
#include <cstdint>
#include <functional>
#include <vector>

namespace phynity::jobs
{

/// Partition range for a single partition index within [0, total_count).
struct PartitionRange
{
    uint32_t start;
    uint32_t end;
};

/// Compute the index range for partition p of partition_count over total_count items.
inline PartitionRange partition_range(uint32_t total_count, uint32_t partition_count, uint32_t p)
{
    return {(total_count * p) / partition_count, (total_count * (p + 1)) / partition_count};
}

/// Factory that creates a JobFn for a given partition range.
/// Signature: (uint32_t start, uint32_t end) -> JobFn
using PartitionFnFactory = std::function<JobFn(uint32_t start, uint32_t end)>;

/// Add a partitioned tier of tasks to a TaskGraph.
///
/// Creates partition_count tasks, each operating on a disjoint range of [0, item_count).
/// Each task gets an affinity_hint matching its partition index for cache locality.
/// If prev_tier is non-empty, adds dependencies from each prev_tier[p] to new_tier[p]
/// (same-partition chaining for data locality).
///
/// @return Vector of TaskIds for the new tier.
// NOLINTBEGIN(clang-analyzer-cplusplus.NewDeleteLeaks) — std::function ownership transfers into TaskGraph
inline std::vector<TaskId> add_partitioned_tier(TaskGraph &graph,
                                                uint32_t item_count,
                                                uint32_t partition_count,
                                                const std::vector<TaskId> &prev_tier,
                                                const PartitionFnFactory &fn_factory,
                                                const char *debug_name)
{
    assert(prev_tier.empty() || prev_tier.size() == partition_count);

    std::vector<TaskId> tier_ids;
    tier_ids.reserve(partition_count);

    for (uint32_t p = 0; p < partition_count; ++p)
    {
        auto [start, end] = partition_range(item_count, partition_count, p);
        auto id = graph.add_task({.fn = fn_factory(start, end), .affinity_hint = p, .debug_name = debug_name});

        if (!prev_tier.empty())
        {
            graph.add_dependency(prev_tier[p], id);
        }

        tier_ids.push_back(id);
    }

    return tier_ids;
}
// NOLINTEND(clang-analyzer-cplusplus.NewDeleteLeaks)

/// Add a single serial task that depends on all tasks in prev_tier (fan-in).
inline TaskId
add_serial_task_after(TaskGraph &graph, const std::vector<TaskId> &prev_tier, JobFn fn, const char *debug_name)
{
    auto id = graph.add_task({.fn = std::move(fn), .debug_name = debug_name});
    for (const auto &dep : prev_tier)
    {
        graph.add_dependency(dep, id);
    }
    return id;
}

} // namespace phynity::jobs

#pragma once

#include "task_graph.hpp"
#include "task_schedule.hpp"

namespace phynity::jobs
{

/// Produces a deterministic topological schedule from a TaskGraph.
/// Uses Kahn's algorithm (BFS) with stable tie-breaking by TaskId::value.
class TaskScheduler
{
public:
    /// Build a deterministic schedule from the graph.
    /// The graph must be a valid DAG (call graph.validate() first).
    [[nodiscard]] static TaskSchedule build_schedule(const TaskGraph &graph);
};

} // namespace phynity::jobs

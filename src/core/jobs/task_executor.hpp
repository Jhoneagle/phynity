#pragma once

#include "job_system.hpp"
#include "task_graph.hpp"
#include "task_schedule.hpp"
#include "task_scheduler.hpp"

namespace phynity::jobs
{

/// Executes a TaskGraph on a JobSystem using wavefront-based dispatch.
/// In concurrent mode, independent tasks within each tier run in parallel.
/// In deterministic mode, tasks execute serially in schedule sequence order.
class TaskExecutor
{
public:
    explicit TaskExecutor(JobSystem &job_system);

    /// Build a schedule from the graph and execute it.
    void execute(const TaskGraph &graph);

    /// Execute a pre-built schedule against its graph.
    void execute(const TaskSchedule &schedule, const TaskGraph &graph);

    /// Execute all tasks serially in schedule sequence order on the calling thread.
    /// Guarantees bit-identical results regardless of worker count.
    void execute_serial(const TaskSchedule &schedule, const TaskGraph &graph);

private:
    void execute_concurrent(const TaskSchedule &schedule, const TaskGraph &graph);

    JobSystem &job_system_;
};

} // namespace phynity::jobs

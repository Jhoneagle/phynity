#pragma once

#include "job_system.hpp"
#include "task_id.hpp"

#include <cassert>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace phynity::jobs
{

struct TaskDesc
{
    JobFn fn;
    uint32_t affinity_hint = std::numeric_limits<uint32_t>::max();
    const char *debug_name = nullptr;
};

/// Directed acyclic graph of tasks with explicit dependencies.
/// Tasks are added with add_task(), edges with add_dependency().
/// Call validate() to check for cycles before scheduling.
class TaskGraph
{
public:
    TaskId add_task(TaskDesc desc);

    /// Declare that `before` must finish before `after` can start.
    void add_dependency(TaskId before, TaskId after);

    /// Check the graph for cycles. Returns true if the graph is a valid DAG.
    [[nodiscard]] bool validate() const;

    void clear();

    [[nodiscard]] uint32_t task_count() const noexcept
    {
        return static_cast<uint32_t>(descriptors_.size());
    }

    [[nodiscard]] const TaskDesc &descriptor(TaskId id) const
    {
        assert(id.valid() && id.value < descriptors_.size());
        return descriptors_[id.value];
    }

    [[nodiscard]] std::span<const TaskId> dependents(TaskId id) const
    {
        assert(id.valid() && id.value < adjacency_.size());
        return adjacency_[id.value];
    }

    [[nodiscard]] uint32_t predecessor_count(TaskId id) const
    {
        assert(id.valid() && id.value < predecessor_counts_.size());
        return predecessor_counts_[id.value];
    }

    /// Return all tasks with zero predecessors.
    [[nodiscard]] std::vector<TaskId> roots() const;

private:
    std::vector<TaskDesc> descriptors_;
    std::vector<std::vector<TaskId>> adjacency_;
    std::vector<uint32_t> predecessor_counts_;
};

} // namespace phynity::jobs

#pragma once

#include "job.hpp"
#include "job_id.hpp"

#include <cassert>
#include <cstdint>
#include <functional>
#include <span>
#include <stack>
#include <vector>

namespace phynity::jobs
{

/// Factory that creates a JobDesc for a given partition range.
/// Signature: (uint32_t start, uint32_t end) -> JobDesc
using PartitionDescFactory = std::function<JobDesc(uint32_t start, uint32_t end)>;

/// Directed acyclic graph of jobs with explicit dependencies.
///
/// Replaces TaskGraph, TaskScheduler, and task_graph_builder from the old system.
/// Jobs are added with add(), edges with depend().
/// Submit the entire graph to JobSystem::submit_graph() for dependency-driven execution.
class JobGraph
{
public:
    /// Add a job to the graph. Returns a graph-local JobId (index only, generation=0).
    JobId add(JobDesc desc)
    {
        JobId id{static_cast<uint32_t>(descs_.size()), 0};
        descs_.push_back(std::move(desc));
        adjacency_.emplace_back();
        pred_counts_.push_back(0);
        return id;
    }

    /// Declare that `before` must finish before `after` can start.
    void depend(JobId before, JobId after)
    {
        assert(before.valid() && before.index < descs_.size());
        assert(after.valid() && after.index < descs_.size());
        assert(before.index != after.index);

        adjacency_[before.index].push_back(after);
        ++pred_counts_[after.index];
    }

    /// Check for cycles. Returns true if the graph is a valid DAG.
    [[nodiscard]] bool validate() const
    {
        if (descs_.empty())
        {
            return true;
        }

        const uint32_t n = size();
        std::vector<uint8_t> color(n, 0); // 0=white, 1=gray, 2=black
        std::stack<std::pair<uint32_t, uint32_t>> stack;

        for (uint32_t start = 0; start < n; ++start)
        {
            if (color[start] != 0)
            {
                continue;
            }

            stack.push({start, 0});
            color[start] = 1;

            while (!stack.empty())
            {
                auto &[node, child_idx] = stack.top();
                const auto &children = adjacency_[node];

                if (child_idx < children.size())
                {
                    uint32_t child = children[child_idx].index;
                    ++child_idx;

                    if (color[child] == 1)
                    {
                        return false; // cycle
                    }
                    if (color[child] == 0)
                    {
                        color[child] = 1;
                        stack.push({child, 0});
                    }
                }
                else
                {
                    color[node] = 2;
                    stack.pop();
                }
            }
        }

        return true;
    }

    void clear()
    {
        descs_.clear();
        adjacency_.clear();
        pred_counts_.clear();
    }

    [[nodiscard]] uint32_t size() const noexcept
    {
        return static_cast<uint32_t>(descs_.size());
    }

    [[nodiscard]] const JobDesc &desc(JobId id) const
    {
        assert(id.valid() && id.index < descs_.size());
        return descs_[id.index];
    }

    [[nodiscard]] std::span<const JobId> dependents(JobId id) const
    {
        assert(id.valid() && id.index < adjacency_.size());
        return adjacency_[id.index];
    }

    [[nodiscard]] uint32_t predecessor_count(JobId id) const
    {
        assert(id.valid() && id.index < pred_counts_.size());
        return pred_counts_[id.index];
    }

    [[nodiscard]] std::vector<JobId> roots() const
    {
        std::vector<JobId> result;
        for (uint32_t i = 0; i < pred_counts_.size(); ++i)
        {
            if (pred_counts_[i] == 0)
            {
                result.push_back(JobId{i, 0});
            }
        }
        return result;
    }

    // ========================================================================
    // Builder helpers (replace task_graph_builder.hpp free functions)
    // ========================================================================

    /// Compute the index range for partition p of partition_count over total_count items.
    static constexpr std::pair<uint32_t, uint32_t>
    partition_range(uint32_t total_count, uint32_t partition_count, uint32_t p)
    {
        return {(total_count * p) / partition_count, (total_count * (p + 1)) / partition_count};
    }

    /// Add a partitioned tier of jobs.
    ///
    /// Creates partition_count jobs, each operating on a disjoint range of [0, item_count).
    /// If prev_tier is non-empty, adds same-partition dependencies for data locality.
    ///
    /// @return Vector of JobIds for the new tier.
    std::vector<JobId> add_partitioned(uint32_t item_count,
                                       uint32_t partition_count,
                                       std::span<const JobId> prev_tier,
                                       const PartitionDescFactory &factory,
                                       const char *debug_name)
    {
        assert(prev_tier.empty() || prev_tier.size() == partition_count);

        std::vector<JobId> tier_ids;
        tier_ids.reserve(partition_count);

        for (uint32_t p = 0; p < partition_count; ++p)
        {
            auto [start, end] = partition_range(item_count, partition_count, p);
            auto desc = factory(start, end);

            // Override affinity_hint to match partition index for cache locality
            if (desc.affinity_hint == std::numeric_limits<uint16_t>::max())
            {
                desc.affinity_hint = static_cast<uint16_t>(p);
            }

            // Override debug_name if the factory didn't set one
            if (desc.debug_name == nullptr)
            {
                desc.debug_name = debug_name;
            }

            auto id = add(std::move(desc));

            if (!prev_tier.empty())
            {
                depend(prev_tier[p], id);
            }

            tier_ids.push_back(id);
        }

        return tier_ids;
    }

    /// Add a single serial job that depends on all jobs in prev_tier (fan-in).
    JobId add_serial_after(std::span<const JobId> prev_tier, JobDesc desc)
    {
        auto id = add(std::move(desc));
        for (const auto &dep : prev_tier)
        {
            depend(dep, id);
        }
        return id;
    }

private:
    std::vector<JobDesc> descs_;
    std::vector<std::vector<JobId>> adjacency_;
    std::vector<uint32_t> pred_counts_;
};

} // namespace phynity::jobs

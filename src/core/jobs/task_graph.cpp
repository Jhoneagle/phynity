#include "task_graph.hpp"

#include <algorithm>
#include <cassert>
#include <stack>

namespace phynity::jobs
{

TaskId TaskGraph::add_task(TaskDesc desc)
{
    TaskId id{static_cast<uint32_t>(descriptors_.size())};
    descriptors_.push_back(std::move(desc));
    adjacency_.emplace_back();
    predecessor_counts_.push_back(0);
    return id;
}

void TaskGraph::add_dependency(TaskId before, TaskId after)
{
    assert(before.valid() && before.value < descriptors_.size());
    assert(after.valid() && after.value < descriptors_.size());
    assert(before.value != after.value);

    adjacency_[before.value].push_back(after);
    ++predecessor_counts_[after.value];
}

bool TaskGraph::validate() const
{
    if (descriptors_.empty())
    {
        return true;
    }

    const uint32_t n = task_count();

    // DFS cycle detection: 0=white (unvisited), 1=gray (in-stack), 2=black (done)
    std::vector<uint8_t> color(n, 0);
    std::stack<std::pair<uint32_t, uint32_t>> stack; // (node, child_index)

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
                uint32_t child = children[child_idx].value;
                ++child_idx;

                if (color[child] == 1)
                {
                    return false; // back edge = cycle
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

void TaskGraph::clear()
{
    descriptors_.clear();
    adjacency_.clear();
    predecessor_counts_.clear();
}

std::vector<TaskId> TaskGraph::roots() const
{
    std::vector<TaskId> result;
    for (uint32_t i = 0; i < predecessor_counts_.size(); ++i)
    {
        if (predecessor_counts_[i] == 0)
        {
            result.push_back(TaskId{i});
        }
    }
    return result;
}

} // namespace phynity::jobs

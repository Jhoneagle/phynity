#include "constraint_graph.hpp"

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace phynity::physics::constraints
{

ConstraintColoring color_constraints(std::span<const Constraint *> constraints,
                                     std::span<const ConstraintBodyPair> body_pairs)
{
    const uint32_t n = static_cast<uint32_t>(constraints.size());
    ConstraintColoring result;

    if (n == 0)
    {
        return result;
    }

    // Build adjacency: two constraints conflict if they share a body pointer
    std::unordered_map<const void *, std::vector<uint32_t>> body_to_constraints;

    for (uint32_t i = 0; i < n; ++i)
    {
        if (body_pairs[i].body_a)
        {
            body_to_constraints[body_pairs[i].body_a].push_back(i);
        }
        if (body_pairs[i].body_b)
        {
            body_to_constraints[body_pairs[i].body_b].push_back(i);
        }
    }

    // Flat adjacency list (duplicates are harmless for greedy coloring)
    std::vector<std::vector<uint32_t>> adj(n);
    for (const auto &[body_ptr, indices] : body_to_constraints)
    {
        for (size_t a = 0; a < indices.size(); ++a)
        {
            for (size_t b = a + 1; b < indices.size(); ++b)
            {
                adj[indices[a]].push_back(indices[b]);
                adj[indices[b]].push_back(indices[a]);
            }
        }
    }

    // Greedy first-fit coloring (process in index order for determinism)
    result.color_of.resize(n, UINT32_MAX);
    uint32_t max_color = 0;
    std::vector<uint32_t> neighbor_colors; // reused across iterations

    for (uint32_t i = 0; i < n; ++i)
    {
        neighbor_colors.clear();
        for (uint32_t neighbor : adj[i])
        {
            if (result.color_of[neighbor] != UINT32_MAX)
            {
                neighbor_colors.push_back(result.color_of[neighbor]);
            }
        }
        std::sort(neighbor_colors.begin(), neighbor_colors.end());
        neighbor_colors.erase(std::unique(neighbor_colors.begin(), neighbor_colors.end()), neighbor_colors.end());

        // Assign the smallest unused color
        uint32_t color = 0;
        for (uint32_t nc : neighbor_colors)
        {
            if (nc != color)
            {
                break;
            }
            ++color;
        }
        result.color_of[i] = color;
        max_color = std::max(max_color, color);
    }

    result.color_count = max_color + 1;

    // Build groups
    result.groups.resize(result.color_count);
    for (uint32_t i = 0; i < n; ++i)
    {
        result.groups[result.color_of[i]].push_back(i);
    }

    return result;
}

} // namespace phynity::physics::constraints

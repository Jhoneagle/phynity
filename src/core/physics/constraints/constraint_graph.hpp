#pragma once

#include "constraint.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace phynity::physics::constraints
{

/// Result of graph coloring for parallel constraint solving.
/// Constraints within the same color group touch disjoint body sets
/// and can be solved in parallel.
struct ConstraintColoring
{
    uint32_t color_count = 0;
    std::vector<uint32_t> color_of; // color_of[constraint_index]
    std::vector<std::vector<uint32_t>> groups; // groups[color] = {constraint indices}
};

/// Extracts body pointer pairs from constraints for conflict detection.
/// Two constraints conflict (share an edge) if they reference the same body.
struct ConstraintBodyPair
{
    const void *body_a = nullptr;
    const void *body_b = nullptr;
};

/// Build a graph coloring of constraints for parallel solving.
/// Uses greedy first-fit coloring on constraints sorted by index.
///
/// @param constraints Span of constraint pointers (only active constraints).
/// @param body_pairs Body references for each constraint (same size as constraints).
///                   Used to detect conflicts: two constraints conflict if they share a body.
/// @return Coloring with color assignments and groups.
ConstraintColoring color_constraints(std::span<const Constraint *> constraints,
                                     std::span<const ConstraintBodyPair> body_pairs);

} // namespace phynity::physics::constraints

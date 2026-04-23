#pragma once

#include <core/serialization/snapshot_schema.hpp>

#include <cstddef>
#include <vector>

namespace phynity::serialization
{

/// Ring buffer of PhysicsSnapshot objects for simulation timeline.
/// Supports push, indexed access, and bounded history for pause/step/rewind.
class SimulationTimeline
{
public:
    explicit SimulationTimeline(size_t max_frames = 600);

    /// Add a snapshot. Overwrites oldest when full.
    void push(PhysicsSnapshot snapshot);

    /// Access snapshot by index (0 = oldest available).
    /// Returns nullptr if index is out of range.
    const PhysicsSnapshot *at(size_t index) const;

    /// Number of snapshots currently stored.
    size_t size() const
    {
        return count_;
    }

    /// Maximum number of snapshots that can be stored.
    size_t capacity() const
    {
        return buffer_.size();
    }

    /// Remove all stored snapshots.
    void clear();

    /// Truncate to keep only the first `new_size` snapshots (oldest).
    /// Used after seeking backward to discard future frames.
    void truncate(size_t new_size);

private:
    std::vector<PhysicsSnapshot> buffer_;
    size_t head_ = 0;  // Next write position
    size_t count_ = 0; // Number of valid entries
};

} // namespace phynity::serialization

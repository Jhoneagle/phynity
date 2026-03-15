#pragma once

#include <core/serialization/replay_schema.hpp>
#include <core/serialization/snapshot_serializer.hpp>
#include <platform/allocation_tracker.hpp>

#include <string>
#include <vector>

namespace phynity::serialization
{

class ReplayReader
{
public:
    ~ReplayReader()
    {
        phynity::platform::track_vector_capacity_release(index_);
    }

    SerializationResult open(const std::string &path);
    size_t frame_count() const;

    SerializationResult read_frame(size_t index, PhysicsSnapshot &snapshot) const;
    SerializationResult read_next(PhysicsSnapshot &snapshot);
    void reset_iteration();

private:
    std::string input_path_;
    bool is_open_ = false;
    size_t next_index_ = 0;
    ReplayFileHeader header_{};
    std::vector<ReplayFrameIndexEntry> index_;
};

} // namespace phynity::serialization

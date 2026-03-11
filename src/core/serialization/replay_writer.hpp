#pragma once

#include <core/serialization/replay_schema.hpp>
#include <core/serialization/snapshot_serializer.hpp>

#include <string>
#include <vector>

namespace phynity::serialization
{

class ReplayWriter
{
public:
    SerializationResult open(const std::string &path);
    SerializationResult append_frame(const PhysicsSnapshot &snapshot);
    SerializationResult close();

    bool is_open() const;

private:
    std::string output_path_;
    bool is_open_ = false;
    bool dt_locked_ = false;
    double frame_dt_ = 1.0 / 60.0;
    std::vector<std::vector<uint8_t>> frames_;
};

} // namespace phynity::serialization

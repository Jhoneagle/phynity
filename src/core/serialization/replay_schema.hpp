#pragma once

#include <core/serialization/snapshot_schema.hpp>

#include <cstdint>

namespace phynity::serialization
{

inline constexpr uint32_t replay_file_magic = 0x59504C52U; // 'RLPY'
inline constexpr uint32_t replay_format_version = 1;

struct ReplayFileHeader
{
    uint32_t magic = replay_file_magic;
    uint32_t format_version = replay_format_version;

    uint32_t schema_major = current_schema_version().major;
    uint32_t schema_minor = current_schema_version().minor;
    uint32_t schema_patch = current_schema_version().patch;

    uint32_t reserved = 0;

    uint64_t frame_count = 0;
    double frame_dt = 1.0 / 60.0;
};

struct ReplayFrameIndexEntry
{
    uint64_t offset = 0;
    uint64_t size = 0;
};

} // namespace phynity::serialization

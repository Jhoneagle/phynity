#pragma once

#include <cstdint>

namespace phynity::jobs
{

/// Binary format constants for schedule recording files.
constexpr uint32_t schedule_file_magic = 0x50485343;   // "PHSC"
constexpr uint32_t schedule_format_version = 1;

} // namespace phynity::jobs

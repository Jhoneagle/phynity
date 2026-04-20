#pragma once

#include "task_id.hpp"

#include <cstdint>
#include <vector>

namespace phynity::jobs
{

struct TaskSchedule
{
    struct Entry
    {
        TaskId id;
        uint32_t tier;     // wavefront number (0 = roots)
        uint32_t sequence; // global deterministic sequence number
    };

    std::vector<Entry> entries;
    uint32_t tier_count = 0;
};

} // namespace phynity::jobs

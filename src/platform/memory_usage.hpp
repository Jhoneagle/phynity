#pragma once

#include <platform/allocation_tracker.hpp>

#include <cstdint>
#include <fstream>
#include <string>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <psapi.h>
#include <windows.h>
#ifdef STRICT
#undef STRICT
#endif
#elif defined(__APPLE__)
#include <sys/resource.h>
#elif defined(__linux__)
#include <sys/resource.h>
#endif

namespace phynity::platform
{

inline uint64_t get_peak_rss_kb()
{
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS counters{};
    if (!GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)))
    {
        return 0;
    }
    return static_cast<uint64_t>(counters.PeakWorkingSetSize / 1024ULL);
#elif defined(__APPLE__)
    struct rusage usage
    {
    };
    if (getrusage(RUSAGE_SELF, &usage) != 0)
    {
        return 0;
    }
    // On macOS ru_maxrss is reported in bytes.
    return static_cast<uint64_t>(usage.ru_maxrss / 1024ULL);
#elif defined(__linux__)
    // Prefer VmHWM as peak RSS when available.
    std::ifstream status("/proc/self/status");
    if (status.is_open())
    {
        std::string line;
        while (std::getline(status, line))
        {
            if (line.rfind("VmHWM:", 0) == 0)
            {
                const auto first_digit = line.find_first_of("0123456789");
                if (first_digit == std::string::npos)
                {
                    return 0;
                }
                return static_cast<uint64_t>(std::stoull(line.substr(first_digit)));
            }
        }
    }

    struct rusage usage
    {
    };
    if (getrusage(RUSAGE_SELF, &usage) != 0)
    {
        return 0;
    }
    return static_cast<uint64_t>(usage.ru_maxrss);
#else
    return 0;
#endif
}

inline int64_t get_allocator_delta_bytes()
{
    return capture_allocator_usage().current_live_bytes;
}

} // namespace phynity::platform
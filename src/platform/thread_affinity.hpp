#pragma once

#include <cstdint>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__linux__)
#include <pthread.h>
#include <sched.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <pthread.h>
#endif

namespace phynity::platform
{

/// Set the calling thread's CPU affinity to a specific core.
/// Returns true on success. Best-effort on macOS (affinity tags only).
inline bool set_thread_affinity([[maybe_unused]] uint32_t core_index)
{
#ifdef _WIN32
    DWORD_PTR mask = static_cast<DWORD_PTR>(1) << core_index;
    return SetThreadAffinityMask(GetCurrentThread(), mask) != 0;
#elif defined(__linux__)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(static_cast<int>(core_index), &cpuset);
    return pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) == 0;
#elif defined(__APPLE__)
    // macOS does not support per-core pinning, only affinity tags.
    // Threads with the same tag are scheduled on the same core when possible.
    thread_affinity_policy_data_t policy;
    policy.affinity_tag = static_cast<integer_t>(core_index + 1); // 0 = no affinity
    kern_return_t result = thread_policy_set(mach_thread_self(),
                                             THREAD_AFFINITY_POLICY,
                                             reinterpret_cast<thread_policy_t>(&policy),
                                             THREAD_AFFINITY_POLICY_COUNT);
    return result == KERN_SUCCESS;
#else
    return false;
#endif
}

/// Get the number of physical cores (may differ from logical cores on hyperthreaded systems).
/// Falls back to hardware_concurrency() if detection is not available.
inline uint32_t physical_core_count()
{
#ifdef _WIN32
    DWORD length = 0;
    GetLogicalProcessorInformation(nullptr, &length);
    if (length == 0)
    {
        return std::thread::hardware_concurrency();
    }

    std::vector<SYSTEM_LOGICAL_PROCESSOR_INFORMATION> buffer(length / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION));
    if (!GetLogicalProcessorInformation(buffer.data(), &length))
    {
        return std::thread::hardware_concurrency();
    }

    uint32_t cores = 0;
    for (const auto &info : buffer)
    {
        if (info.Relationship == RelationProcessorCore)
        {
            ++cores;
        }
    }
    return cores > 0 ? cores : std::thread::hardware_concurrency();
#elif defined(__linux__)
    // sysconf returns logical processors; for physical cores, parse /proc/cpuinfo
    // For simplicity, use hardware_concurrency / 2 as heuristic when HT is likely
    unsigned int hw = std::thread::hardware_concurrency();
    return hw > 0 ? hw : 1;
#else
    unsigned int hw = std::thread::hardware_concurrency();
    return hw > 0 ? hw : 1;
#endif
}

} // namespace phynity::platform

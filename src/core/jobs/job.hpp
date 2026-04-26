#pragma once

#include "job_id.hpp"

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace phynity::jobs
{

/// Raw function pointer type for job execution. Data is passed via void*.
using JobFnPtr = void (*)(void *data);

/// Maximum number of dependents stored inline in a Job.
/// Jobs with more dependents use the overflow array in the JobPool.
static constexpr uint32_t kMaxInlineDependents = 6;

/// Maximum size of inline data that can be stored directly in a Job.
/// Captures that exceed this must be heap-allocated externally.
static constexpr uint32_t kMaxInlineDataSize = 32;

/// Fixed-size, cacheline-aligned job slot.
///
/// No std::function, no mutex, no condition_variable.
/// All synchronization is through atomic predecessor_count and a shared EventCount.
///
/// Layout (128 bytes = 2 cache lines):
///   Line 1: function, data, inline_data, predecessor_count, dependent_count, affinity_hint
///   Line 2: generation, debug_name, dependents[], overflow
struct alignas(64) Job
{
    // === Cache line 1: execution data ===
    JobFnPtr function = nullptr;                                 // 8
    void *data = nullptr;                                        // 8
    uint8_t inline_data[kMaxInlineDataSize]{};                   // 32
    std::atomic<int32_t> predecessor_count{0};                   // 4
    uint16_t dependent_count = 0;                                // 2
    uint16_t affinity_hint = (std::numeric_limits<uint16_t>::max)(); // 2 (max = no preference)
    uint32_t generation = 0;                                     // 4
    // padding: 4 bytes

    // === Cache line 2: dependency data ===
    const char *debug_name = nullptr;                            // 8
    JobId dependents[kMaxInlineDependents]{};                    // 48 (6 * 8)
    uint32_t overflow_offset = 0;                                // 4 (index into pool's overflow array)
    uint32_t overflow_count = 0;                                 // 4 (number of overflow dependents)
};

static_assert(sizeof(Job) == 128, "Job must be exactly 128 bytes (2 cache lines)");
static_assert(alignof(Job) == 64, "Job must be cacheline-aligned");

/// Descriptor for submitting a job. Used by JobGraph and direct submission APIs.
struct JobDesc
{
    JobFnPtr function = nullptr;
    void *data = nullptr;
    uint16_t affinity_hint = (std::numeric_limits<uint16_t>::max)();
    const char *debug_name = nullptr;
};

/// Helper to store small data inline in a Job's inline_data buffer.
/// Returns a pointer to the inline storage (which the caller uses as `data`).
template <typename T> void *store_inline_data(Job &job, const T &value)
{
    static_assert(std::is_trivially_copyable_v<T>, "Inline data must be trivially copyable");
    static_assert(sizeof(T) <= kMaxInlineDataSize, "Data too large for inline storage");

    std::memcpy(job.inline_data, &value, sizeof(T));
    return static_cast<void *>(job.inline_data);
}

} // namespace phynity::jobs

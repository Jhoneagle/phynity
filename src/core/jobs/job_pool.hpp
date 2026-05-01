#pragma once

#include "job.hpp"
#include "job_id.hpp"

#include <platform/threading.hpp>

#include <atomic>
#include <cassert>
#include <cstdint>
#include <mutex>
#include <vector>

namespace phynity::jobs
{

/// Pre-allocated flat array of Job slots with generation-based recycling.
///
/// Slots are allocated via a monotonic counter (next_alloc_) modulo capacity.
/// Each slot has a generation counter for ABA protection.
/// When a slot is needed but still occupied, the submitter spins until it is freed.
///
/// The overflow_deps_ array stores dependents beyond kMaxInlineDependents.
class JobPool
{
public:
    explicit JobPool(uint32_t capacity = 4096) : capacity_(capacity), slots_(capacity)
    {
    }

    /// Allocate a job slot. Spins if the target slot is still in use.
    /// Returns a JobId with the slot index and its new generation.
    [[nodiscard]] JobId allocate()
    {
        uint32_t raw = next_alloc_.fetch_add(1, std::memory_order_relaxed);
        uint32_t idx = raw % capacity_;
        uint32_t gen = raw / capacity_ + 1;

        auto &slot = slots_[idx];

        // Spin until the slot is free (predecessor_count <= 0 and generation is older)
        while (slot.generation >= gen)
        {
            platform::yield_thread();
        }

        // Initialize the slot
        slot.function = nullptr;
        slot.data = nullptr;
        slot.predecessor_count.store(0, std::memory_order_relaxed);
        slot.dependent_count = 0;
        slot.affinity_hint = (std::numeric_limits<uint16_t>::max)();
        slot.generation = gen;
        slot.debug_name = nullptr;
        slot.overflow_offset = 0;
        slot.overflow_count = 0;

        return JobId{idx, gen};
    }

    /// Release a slot for reuse. Called after a job has completed and its
    /// dependents have been notified.
    void release(JobId id) noexcept
    {
        // Generation already set at allocation; the slot becomes available
        // when next_alloc_ wraps around and the new generation exceeds it.
        // No explicit action needed — the spin in allocate() handles this.
        (void) id;
    }

    /// Get a mutable reference to the job at the given index.
    [[nodiscard]] Job &operator[](uint32_t index) noexcept
    {
        assert(index < capacity_);
        return slots_[index];
    }

    /// Get a const reference to the job at the given index.
    [[nodiscard]] const Job &operator[](uint32_t index) const noexcept
    {
        assert(index < capacity_);
        return slots_[index];
    }

    /// Convenience: access via JobId.
    [[nodiscard]] Job &at(JobId id) noexcept
    {
        assert(id.valid() && id.index < capacity_);
        return slots_[id.index];
    }

    [[nodiscard]] const Job &at(JobId id) const noexcept
    {
        assert(id.valid() && id.index < capacity_);
        return slots_[id.index];
    }

    [[nodiscard]] uint32_t capacity() const noexcept
    {
        return capacity_;
    }

    /// Reserve space in the overflow dependents array and return the starting offset.
    /// Thread-safe via mutex (overflow is rare, contention is acceptable).
    uint32_t allocate_overflow(uint32_t count)
    {
        std::lock_guard<std::mutex> lock(overflow_mutex_);
        uint32_t offset = static_cast<uint32_t>(overflow_deps_.size());
        overflow_deps_.resize(offset + count);
        return offset;
    }

    /// Access an overflow dependent by offset.
    [[nodiscard]] JobId &overflow_dependent(uint32_t offset) noexcept
    {
        assert(offset < overflow_deps_.size());
        return overflow_deps_[offset];
    }

    [[nodiscard]] const JobId &overflow_dependent(uint32_t offset) const noexcept
    {
        assert(offset < overflow_deps_.size());
        return overflow_deps_[offset];
    }

    /// Clear overflow storage. Call between frames or when the pool is quiescent.
    void clear_overflow()
    {
        std::lock_guard<std::mutex> lock(overflow_mutex_);
        overflow_deps_.clear();
    }

private:
    uint32_t capacity_;
    std::vector<Job> slots_;
    std::atomic<uint32_t> next_alloc_{0};

    std::vector<JobId> overflow_deps_;
    std::mutex overflow_mutex_;
};

} // namespace phynity::jobs

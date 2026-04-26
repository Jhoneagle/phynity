#pragma once

#include "eventcount.hpp"
#include "job_id.hpp"

#include <atomic>
#include <cassert>
#include <cstdint>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace phynity::jobs
{

/// Internal counter slot used by the counter pool.
struct CounterSlot
{
    std::atomic<int32_t> value{0};
    uint32_t generation = 0;
};

/// Pool of shared completion counters. Counters are recycled via generation tracking.
///
/// Typical flow:
///   1. acquire(N) -> CounterHandle (counter starts at N)
///   2. Each completing job calls decrement(handle)
///   3. Main thread calls wait(handle, ec) to block until counter reaches zero
///   4. wait() releases the counter back to the pool
class CounterPool
{
public:
    explicit CounterPool(uint32_t capacity = 256) : slots_(capacity)
    {
        free_list_.reserve(capacity);
        for (uint32_t i = capacity; i > 0; --i)
        {
            free_list_.push_back(i - 1);
        }
    }

    /// Acquire a counter initialized to `initial_value`.
    /// Returns an invalid handle if the pool is exhausted.
    [[nodiscard]] CounterHandle acquire(int32_t initial_value)
    {
        std::lock_guard<std::mutex> lock(free_mutex_);
        if (free_list_.empty())
        {
            return CounterHandle{};
        }

        uint32_t idx = free_list_.back();
        free_list_.pop_back();

        auto &slot = slots_[idx];
        slot.generation++;
        slot.value.store(initial_value, std::memory_order_release);

        return CounterHandle{idx, slot.generation};
    }

    /// Decrement the counter by 1. Returns true if the counter reached zero
    /// (i.e., the caller was the last to decrement).
    bool decrement(CounterHandle handle, EventCount &ec) noexcept
    {
        if (!handle.valid())
        {
            return false;
        }

        auto &slot = slots_[handle.index];
        if (slot.generation != handle.generation)
        {
            return false; // stale handle
        }

        int32_t prev = slot.value.fetch_sub(1, std::memory_order_acq_rel);
        if (prev == 1)
        {
            // Counter reached zero — wake any waiter
            ec.notify_all();
            return true;
        }

        return false;
    }

    /// Block until the counter reaches zero, then release the slot back to the pool.
    /// If `running` is provided and becomes false, returns early without waiting.
    void wait(CounterHandle handle, EventCount &ec, const std::atomic<bool> *running = nullptr)
    {
        if (!handle.valid())
        {
            return;
        }

        auto &slot = slots_[handle.index];
        if (slot.generation != handle.generation)
        {
            return; // stale handle
        }

        // Spin briefly
        for (int spin = 0; spin < 64; ++spin)
        {
            if (slot.value.load(std::memory_order_acquire) <= 0)
            {
                release(handle);
                return;
            }
            if (running && !running->load(std::memory_order_acquire))
            {
                return;
            }

#if defined(_MSC_VER)
            _mm_pause();
#elif defined(__x86_64__) || defined(__i386__)
            __builtin_ia32_pause();
#else
            // ARM or other: yield
            std::this_thread::yield();
#endif
        }

        // Fall back to eventcount blocking
        for (;;)
        {
            auto token = ec.prepare_wait();
            if (slot.value.load(std::memory_order_acquire) <= 0)
            {
                ec.cancel_wait(token);
                release(handle);
                return;
            }
            if (running && !running->load(std::memory_order_acquire))
            {
                ec.cancel_wait(token);
                return;
            }
            ec.wait(token);
        }
    }

    /// Read the current value of a counter (racy but useful for debugging).
    [[nodiscard]] int32_t peek(CounterHandle handle) const noexcept
    {
        if (!handle.valid() || handle.index >= slots_.size())
        {
            return 0;
        }
        return slots_[handle.index].value.load(std::memory_order_relaxed);
    }

private:
    void release(CounterHandle handle)
    {
        std::lock_guard<std::mutex> lock(free_mutex_);
        free_list_.push_back(handle.index);
    }

    std::vector<CounterSlot> slots_;
    std::vector<uint32_t> free_list_;
    std::mutex free_mutex_;
};

} // namespace phynity::jobs

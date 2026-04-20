#pragma once

#include <atomic>
#include <cassert>
#include <cstdint>
#include <memory>
#include <vector>

namespace phynity::jobs
{

/// Lock-free work-stealing deque (Chase-Lev algorithm).
///
/// The owning thread pushes/pops from the bottom (LIFO - cache-hot work).
/// Thief threads steal from the top (FIFO - load balancing).
/// Fixed capacity with power-of-2 sizing.
template <typename T> class WorkStealingDeque
{
public:
    explicit WorkStealingDeque(uint32_t capacity_log2 = 10) : mask_((1u << capacity_log2) - 1)
    {
        buffer_.resize(mask_ + 1);
    }

    WorkStealingDeque(const WorkStealingDeque &) = delete;
    WorkStealingDeque &operator=(const WorkStealingDeque &) = delete;

    /// Push an item (owner thread only). Returns false if full.
    bool push(const T &item)
    {
        int64_t b = bottom_.load(std::memory_order_relaxed);
        int64_t t = top_.load(std::memory_order_acquire);

        if (b - t >= static_cast<int64_t>(mask_ + 1))
        {
            return false; // full
        }

        buffer_[static_cast<uint32_t>(b) & mask_] = item;
        std::atomic_thread_fence(std::memory_order_release);
        bottom_.store(b + 1, std::memory_order_relaxed);
        return true;
    }

    /// Pop an item from the bottom (owner thread only, LIFO).
    bool pop(T &item)
    {
        int64_t b = bottom_.load(std::memory_order_relaxed) - 1;
        bottom_.store(b, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_seq_cst);
        int64_t t = top_.load(std::memory_order_relaxed);

        if (t <= b)
        {
            // Non-empty
            item = buffer_[static_cast<uint32_t>(b) & mask_];

            if (t == b)
            {
                // Last element - race with steal
                if (!top_.compare_exchange_strong(t, t + 1, std::memory_order_seq_cst,
                                                  std::memory_order_relaxed))
                {
                    // Lost race to a thief
                    bottom_.store(t + 1, std::memory_order_relaxed);
                    return false;
                }
                bottom_.store(t + 1, std::memory_order_relaxed);
            }
            return true;
        }

        // Empty
        bottom_.store(t, std::memory_order_relaxed);
        return false;
    }

    /// Steal an item from the top (any thread, FIFO).
    bool steal(T &item)
    {
        int64_t t = top_.load(std::memory_order_acquire);
        std::atomic_thread_fence(std::memory_order_seq_cst);
        int64_t b = bottom_.load(std::memory_order_acquire);

        if (t >= b)
        {
            return false; // empty
        }

        item = buffer_[static_cast<uint32_t>(t) & mask_];

        if (!top_.compare_exchange_strong(t, t + 1, std::memory_order_seq_cst, std::memory_order_relaxed))
        {
            return false; // lost race
        }

        return true;
    }

    /// Approximate size (racy but useful for heuristics).
    [[nodiscard]] int64_t size_approx() const
    {
        int64_t b = bottom_.load(std::memory_order_relaxed);
        int64_t t = top_.load(std::memory_order_relaxed);
        return b - t;
    }

private:
    // Read-only after construction — safe to share cache lines with each other
    uint32_t mask_;
    std::vector<T> buffer_;

    // Contended atomics on separate cache lines to prevent false sharing.
    // top_ is CAS'd by thieves; bottom_ is modified by the owner.
    alignas(64) std::atomic<int64_t> top_{0};
    alignas(64) std::atomic<int64_t> bottom_{0};
};

} // namespace phynity::jobs

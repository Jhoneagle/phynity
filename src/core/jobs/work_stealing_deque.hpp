#pragma once

#include <atomic>
#include <cassert>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace phynity::jobs
{

/// Lock-free work-stealing deque (Chase-Lev algorithm).
///
/// The owning thread pushes/pops from the bottom (LIFO - cache-hot work).
/// Thief threads steal from the top (FIFO - load balancing).
/// Fixed capacity with power-of-2 sizing.
///
/// T must be trivially copyable and fit in a std::atomic (<=8 bytes).
/// Buffer elements are std::atomic<T> — no mutex needed.
template <typename T> class WorkStealingDeque
{
    static_assert(std::is_trivially_copyable_v<T>, "WorkStealingDeque requires trivially copyable T");
    static_assert(sizeof(T) <= 8, "WorkStealingDeque requires sizeof(T) <= 8 for lock-free atomics");

public:
    explicit WorkStealingDeque(uint32_t capacity_log2 = 10)
        : mask_((1u << capacity_log2) - 1), buffer_(new std::atomic<T>[static_cast<size_t>(mask_) + 1]{})
    {
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

        const uint32_t index = static_cast<uint32_t>(b) & mask_;
        buffer_[index].store(item, std::memory_order_relaxed);
        bottom_.store(b + 1, std::memory_order_release);
        return true;
    }

    /// Pop an item from the bottom (owner thread only, LIFO).
    bool pop(T &item)
    {
        int64_t b = bottom_.load(std::memory_order_relaxed) - 1;
        bottom_.store(b, std::memory_order_seq_cst);
        int64_t t = top_.load(std::memory_order_seq_cst);

        if (t <= b)
        {
            const uint32_t index = static_cast<uint32_t>(b) & mask_;
            item = buffer_[index].load(std::memory_order_relaxed);

            if (t == b)
            {
                // Last element — CAS resolves race with steal.
                // Use b+1 (not t+1) because compare_exchange_strong mutates t on failure.
                if (!top_.compare_exchange_strong(t, t + 1, std::memory_order_seq_cst, std::memory_order_relaxed))
                {
                    bottom_.store(b + 1, std::memory_order_relaxed);
                    return false;
                }
                bottom_.store(b + 1, std::memory_order_relaxed);
            }
            return true;
        }

        bottom_.store(t, std::memory_order_relaxed);
        return false;
    }

    /// Steal an item from the top (any thread, FIFO).
    bool steal(T &item)
    {
        int64_t t = top_.load(std::memory_order_seq_cst);
        int64_t b = bottom_.load(std::memory_order_seq_cst);

        if (t >= b)
        {
            return false; // empty
        }

        const uint32_t index = static_cast<uint32_t>(t) & mask_;
        item = buffer_[index].load(std::memory_order_relaxed);

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
    std::unique_ptr<std::atomic<T>[]> buffer_;

    // Contended atomics on separate cache lines to prevent false sharing.
    // top_ is CAS'd by thieves; bottom_ is modified by the owner.
    // MSVC C4324: structure was padded due to alignment specifier — intentional.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
    alignas(64) std::atomic<int64_t> top_{0};
    alignas(64) std::atomic<int64_t> bottom_{0};
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
};

} // namespace phynity::jobs

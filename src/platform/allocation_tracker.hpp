#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace phynity::platform
{

struct AllocatorUsageSnapshot
{
    int64_t current_live_bytes = 0;
    int64_t peak_live_bytes = 0;
};

namespace detail
{

inline std::atomic<int64_t> &current_live_bytes_storage() noexcept
{
    static std::atomic<int64_t> value{0};
    return value;
}

inline std::atomic<int64_t> &peak_live_bytes_storage() noexcept
{
    static std::atomic<int64_t> value{0};
    return value;
}

inline void update_peak_live_bytes(const int64_t candidate) noexcept
{
    auto &peak = peak_live_bytes_storage();
    int64_t observed = peak.load(std::memory_order_relaxed);
    while (candidate > observed &&
           !peak.compare_exchange_weak(observed, candidate, std::memory_order_relaxed, std::memory_order_relaxed))
    {
    }
}

} // namespace detail

inline void record_allocator_allocation_bytes(const size_t bytes) noexcept
{
    if (bytes == 0)
    {
        return;
    }

    auto &current = detail::current_live_bytes_storage();
    const int64_t delta = static_cast<int64_t>(bytes);
    const int64_t now = current.fetch_add(delta, std::memory_order_relaxed) + delta;
    detail::update_peak_live_bytes(now);
}

inline void record_allocator_deallocation_bytes(const size_t bytes) noexcept
{
    if (bytes == 0)
    {
        return;
    }

    detail::current_live_bytes_storage().fetch_sub(static_cast<int64_t>(bytes), std::memory_order_relaxed);
}

inline AllocatorUsageSnapshot capture_allocator_usage() noexcept
{
    return {
        detail::current_live_bytes_storage().load(std::memory_order_relaxed),
        detail::peak_live_bytes_storage().load(std::memory_order_relaxed),
    };
}

inline void reset_allocator_peak_to_current() noexcept
{
    detail::peak_live_bytes_storage().store(detail::current_live_bytes_storage().load(std::memory_order_relaxed),
                                            std::memory_order_relaxed);
}

class AllocatorDeltaScope
{
public:
    AllocatorDeltaScope() noexcept : baseline_live_bytes_(capture_allocator_usage().current_live_bytes)
    {
        reset_allocator_peak_to_current();
    }

    [[nodiscard]] int64_t delta_bytes() const noexcept
    {
        const int64_t peak_live_bytes = capture_allocator_usage().peak_live_bytes;
        if (peak_live_bytes <= baseline_live_bytes_)
        {
            return 0;
        }

        return peak_live_bytes - baseline_live_bytes_;
    }

private:
    int64_t baseline_live_bytes_ = 0;
};

template <typename T> class TrackedAllocator
{
public:
    using value_type = T;

    TrackedAllocator() noexcept = default;

    template <typename U> TrackedAllocator(const TrackedAllocator<U> &) noexcept
    {
    }

    [[nodiscard]] T *allocate(const size_t count)
    {
        record_allocator_allocation_bytes(count * sizeof(T));
        return std::allocator<T>{}.allocate(count);
    }

    void deallocate(T *ptr, const size_t count) noexcept
    {
        record_allocator_deallocation_bytes(count * sizeof(T));
        std::allocator<T>{}.deallocate(ptr, count);
    }

    template <typename U> struct rebind
    {
        using other = TrackedAllocator<U>;
    };

    using is_always_equal = std::true_type;
};

template <typename T, typename U>
inline bool operator==(const TrackedAllocator<T> &, const TrackedAllocator<U> &) noexcept
{
    return true;
}

template <typename T, typename U>
inline bool operator!=(const TrackedAllocator<T> &, const TrackedAllocator<U> &) noexcept
{
    return false;
}

template <typename T> using TrackedVector = std::vector<T, TrackedAllocator<T>>;

template <typename K, typename V, typename Hash = std::hash<K>, typename KeyEqual = std::equal_to<K>>
using TrackedUnorderedMap = std::unordered_map<K, V, Hash, KeyEqual, TrackedAllocator<std::pair<const K, V>>>;

template <typename K, typename Hash = std::hash<K>, typename KeyEqual = std::equal_to<K>>
using TrackedUnorderedSet = std::unordered_set<K, Hash, KeyEqual, TrackedAllocator<K>>;

template <typename T, typename Alloc>
inline void track_vector_capacity_change(const std::vector<T, Alloc> &values, const size_t previous_capacity) noexcept
{
    const size_t current_capacity = values.capacity();
    if (current_capacity == previous_capacity)
    {
        return;
    }

    if (current_capacity > 0)
    {
        record_allocator_allocation_bytes(current_capacity * sizeof(T));
    }
    if (previous_capacity > 0)
    {
        record_allocator_deallocation_bytes(previous_capacity * sizeof(T));
    }
}

template <typename T, typename Alloc>
inline void track_vector_capacity_release(const std::vector<T, Alloc> &values) noexcept
{
    if (values.capacity() > 0)
    {
        record_allocator_deallocation_bytes(values.capacity() * sizeof(T));
    }
}

} // namespace phynity::platform
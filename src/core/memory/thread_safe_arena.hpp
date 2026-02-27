#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace phynity::memory {

// Simple thread-safe bump allocator for per-frame allocations.
// Uses a single mutex to protect the allocation cursor.
class ThreadSafeArena {
public:
    explicit ThreadSafeArena(size_t capacity_bytes)
        : buffer_(capacity_bytes, std::byte{0}), capacity_(capacity_bytes) {}

    ThreadSafeArena(const ThreadSafeArena&) = delete;
    ThreadSafeArena& operator=(const ThreadSafeArena&) = delete;

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        offset_ = 0;
    }

    void* allocate(size_t size, size_t alignment = alignof(std::max_align_t)) {
        std::lock_guard<std::mutex> lock(mutex_);
        const size_t aligned = align_up(offset_, alignment);
        if (aligned + size > capacity_) {
            return nullptr;
        }
        void* ptr = buffer_.data() + aligned;
        offset_ = aligned + size;
        return ptr;
    }

    [[nodiscard]] size_t capacity() const noexcept { return capacity_; }
    [[nodiscard]] size_t size() const noexcept { return offset_; }

private:
    static size_t align_up(size_t value, size_t alignment) {
        const size_t mask = alignment - 1;
        return (value + mask) & ~mask;
    }

    std::mutex mutex_;
    std::vector<std::byte> buffer_;
    size_t capacity_ = 0;
    size_t offset_ = 0;
};

}  // namespace phynity::memory

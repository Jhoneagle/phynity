#pragma once

#include "job_handle.hpp"

#include <cstdint>
#include <deque>
#include <mutex>

namespace phynity::jobs {

// Simple work-stealing deque for bootstrap phase.
// Uses a single mutex for all operations (no lock-free yet).
class JobQueue {
public:
    explicit JobQueue(uint32_t capacity = 1024) : capacity_(capacity) {}
    JobQueue(const JobQueue&) = delete;
    JobQueue& operator=(const JobQueue&) = delete;

    bool push(JobHandle job) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= capacity_) {
            return false;
        }
        queue_.push_back(job);
        return true;
    }

    bool pop(JobHandle& job) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        job = queue_.back();
        queue_.pop_back();
        return true;
    }

    bool steal(JobHandle& job) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        job = queue_.front();
        queue_.pop_front();
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

private:
    mutable std::mutex mutex_;
    std::deque<JobHandle> queue_;
    uint32_t capacity_;
};

}  // namespace phynity::jobs

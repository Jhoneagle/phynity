#pragma once

#include "job_handle.hpp"
#include "work_stealing_deque.hpp"

#include <cstdint>

namespace phynity::jobs
{

/// Work-stealing job queue backed by a lock-free Chase-Lev deque.
/// Owner thread pushes/pops (LIFO), thieves steal (FIFO).
class JobQueue
{
public:
    explicit JobQueue(uint32_t capacity_log2 = 10) : deque_(capacity_log2)
    {
    }
    JobQueue(const JobQueue &) = delete;
    JobQueue &operator=(const JobQueue &) = delete;

    bool push(JobHandle job)
    {
        return deque_.push(job);
    }

    bool pop(JobHandle &job)
    {
        return deque_.pop(job);
    }

    bool steal(JobHandle &job)
    {
        return deque_.steal(job);
    }

    [[nodiscard]] int64_t size_approx() const
    {
        return deque_.size_approx();
    }

private:
    WorkStealingDeque<JobHandle> deque_;
};

} // namespace phynity::jobs

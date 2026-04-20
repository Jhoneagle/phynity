#include "job_system.hpp"
#include "work_stealing_deque.hpp"

#include <platform/allocation_tracker.hpp>
#include <platform/thread_affinity.hpp>
#include <platform/threading.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <vector>

namespace phynity::jobs
{

// Internal PIMPL implementation intentionally uses compact naming and state layout.
// NOLINTBEGIN(readability-identifier-naming,misc-non-private-member-variables-in-classes,readability-function-cognitive-complexity,readability-convert-member-functions-to-static)

/// Pre-allocated job slot. Uses generation counting to detect stale handles.
struct JobSlot
{
    JobFn fn;
    std::atomic<bool> completed{false};
    std::atomic<uint32_t> generation{0}; // incremented on each reuse
    std::condition_variable done_cv;
    std::mutex done_mutex;

    void reset(JobFn f, uint32_t gen)
    {
        fn = std::move(f);
        completed.store(false, std::memory_order_relaxed);
        generation.store(gen, std::memory_order_release);
    }
};

class JobSystemImpl
{
public:
    explicit JobSystemImpl(const JobSystemConfig &config);
    ~JobSystemImpl();

    void start(const JobSystemConfig &config);
    void shutdown();

    bool is_running() const noexcept
    {
        return running_.load();
    }
    uint32_t worker_count() const noexcept
    {
        return worker_count_;
    }
    SchedulingMode scheduling_mode() const noexcept
    {
        return mode_;
    }

    JobHandle submit(JobFn job);
    JobHandle submit_to_worker(uint32_t worker_index, JobFn job);
    void wait(JobHandle handle);

private:
    JobHandle submit_impl(JobFn job, uint32_t target_worker);
    void worker_loop(uint32_t worker_index);

    std::atomic<bool> running_{false};

    uint32_t worker_count_ = 0;
    SchedulingMode mode_ = SchedulingMode::Concurrent;

    phynity::platform::TrackedVector<platform::Thread> workers_;

    // Job pool: fixed-size ring buffer indexed by job_id % pool_capacity_
    static constexpr uint32_t pool_capacity_ = 4096;
    std::vector<JobSlot> job_pool_;
    std::atomic<uint32_t> next_job_id_{1};

    // Per-worker work-stealing deques
    std::vector<std::unique_ptr<WorkStealingDeque<uint32_t>>> worker_queues_;

    // Round-robin submit distribution
    std::atomic<uint32_t> submit_counter_{0};

    // Wakeup mechanism
    std::mutex wake_mutex_;
    std::condition_variable wake_cv_;
    std::atomic<int64_t> pending_work_{0};
};

JobSystemImpl::JobSystemImpl(const JobSystemConfig &config)
{
    start(config);
}

void JobSystemImpl::start(const JobSystemConfig &config)
{
    if (running_.load())
    {
        return;
    }

    mode_ = config.mode;
    worker_count_ = config.worker_count;
    if (worker_count_ == 0)
    {
        worker_count_ = platform::hardware_concurrency();
    }

    // In deterministic/replay modes, use single worker thread.
    uint32_t actual_workers =
        (mode_ == SchedulingMode::Deterministic || mode_ == SchedulingMode::DeterministicReplay) ? 1 : worker_count_;

    // Initialize job pool
    job_pool_.resize(pool_capacity_);

    // Create per-worker deques
    worker_queues_.clear();
    for (uint32_t i = 0; i < actual_workers; ++i)
    {
        worker_queues_.push_back(std::make_unique<WorkStealingDeque<uint32_t>>(10)); // capacity 1024
    }

    pending_work_.store(0);
    running_.store(true);

    for (uint32_t i = 0; i < actual_workers; ++i)
    {
        workers_.emplace_back([this, i] { worker_loop(i); });
    }
}

void JobSystemImpl::shutdown()
{
    if (!running_.load())
    {
        return;
    }

    running_.store(false);

    // Wake all workers
    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        wake_cv_.notify_all();
    }

    // Join all workers
    for (auto &worker : workers_)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
    workers_.clear();
    worker_queues_.clear();
}

JobSystemImpl::~JobSystemImpl()
{
    shutdown();
}

JobHandle JobSystemImpl::submit(JobFn job)
{
    uint32_t target = submit_counter_.fetch_add(1, std::memory_order_relaxed) %
                      static_cast<uint32_t>(worker_queues_.size());
    return submit_impl(std::move(job), target);
}

JobHandle JobSystemImpl::submit_to_worker(uint32_t worker_index, JobFn job)
{
    uint32_t target = worker_index % static_cast<uint32_t>(worker_queues_.size());
    return submit_impl(std::move(job), target);
}

JobHandle JobSystemImpl::submit_impl(JobFn job, uint32_t target_worker)
{
    if (!running_.load())
    {
        return JobHandle{};
    }

    uint32_t job_id = next_job_id_.fetch_add(1, std::memory_order_relaxed);
    uint32_t slot_index = job_id % pool_capacity_;
    uint32_t generation = job_id / pool_capacity_ + 1;

    auto &slot = job_pool_[slot_index];

    // Spin-wait until the previous occupant is consumed (completed or generation mismatch).
    // With pool_capacity_=4096 and typical ~20 tasks/frame, this never spins in practice.
    while (slot.generation.load(std::memory_order_acquire) != 0 && !slot.completed.load(std::memory_order_acquire))
    {
        platform::yield_thread();
    }

    slot.reset(std::move(job), generation);

    worker_queues_[target_worker]->push(job_id);
    pending_work_.fetch_add(1, std::memory_order_release);

    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        wake_cv_.notify_one();
    }

    return JobHandle{job_id, generation};
}

void JobSystemImpl::wait(JobHandle handle)
{
    if (!handle.valid() || !running_.load())
    {
        return;
    }

    uint32_t slot_index = handle.id % pool_capacity_;
    auto &slot = job_pool_[slot_index];

    // Check generation to ensure we're waiting on the right job
    if (slot.generation.load(std::memory_order_acquire) != handle.generation)
    {
        return; // already recycled
    }

    std::unique_lock<std::mutex> lock(slot.done_mutex);
    slot.done_cv.wait(lock,
                      [&slot, &handle]
                      {
                          return slot.completed.load() ||
                                 slot.generation.load(std::memory_order_acquire) != handle.generation;
                      });

    // Clear the fn to release captured resources promptly
    slot.fn = nullptr;
}

void JobSystemImpl::worker_loop(uint32_t worker_index)
{
    // Best-effort CPU affinity for cache locality
    platform::set_thread_affinity(worker_index);

    auto &my_queue = *worker_queues_[worker_index];
    const uint32_t num_queues = static_cast<uint32_t>(worker_queues_.size());

    while (running_.load())
    {
        uint32_t job_id = 0;
        bool found = false;

        // 1. Try own queue first (cache-hot)
        found = my_queue.pop(job_id);

        // 2. Try stealing from neighbors
        if (!found)
        {
            for (uint32_t offset = 1; offset < num_queues && !found; ++offset)
            {
                uint32_t victim = (worker_index + offset) % num_queues;
                found = worker_queues_[victim]->steal(job_id);
            }
        }

        if (found)
        {
            uint32_t slot_index = job_id % pool_capacity_;
            auto &slot = job_pool_[slot_index];

            if (static_cast<bool>(slot.fn))
            {
                slot.fn();
                slot.completed.store(true, std::memory_order_release);
                slot.done_cv.notify_all();
            }

            pending_work_.fetch_sub(1, std::memory_order_release);
        }
        else
        {
            // 3. No work found - wait with timeout
            std::unique_lock<std::mutex> lock(wake_mutex_);
            wake_cv_.wait_for(lock, std::chrono::milliseconds(1),
                              [this] { return pending_work_.load(std::memory_order_acquire) > 0 || !running_.load(); });
        }
    }
}

// =============================================================================
// JobSystem public interface (instance-owned PIMPL)
// =============================================================================

JobSystem::JobSystem() = default;

JobSystem::JobSystem(const JobSystemConfig &config)
{
    start(config);
}

JobSystem::~JobSystem()
{
    shutdown();
}

JobSystem::JobSystem(JobSystem &&) noexcept = default;
JobSystem &JobSystem::operator=(JobSystem &&) noexcept = default;

void JobSystem::start(const JobSystemConfig &config)
{
    if (!impl_)
    {
        impl_ = std::make_unique<JobSystemImpl>(config);
    }
}

void JobSystem::shutdown()
{
    if (impl_)
    {
        impl_->shutdown();
        impl_.reset();
    }
}

bool JobSystem::is_running() const noexcept
{
    return impl_ && impl_->is_running();
}

uint32_t JobSystem::worker_count() const noexcept
{
    return impl_ ? impl_->worker_count() : 0;
}

SchedulingMode JobSystem::scheduling_mode() const noexcept
{
    return impl_ ? impl_->scheduling_mode() : SchedulingMode::Concurrent;
}

JobHandle JobSystem::submit(JobFn job)
{
    return impl_ ? impl_->submit(std::move(job)) : JobHandle{};
}

JobHandle JobSystem::submit_to_worker(uint32_t worker_index, JobFn job)
{
    return impl_ ? impl_->submit_to_worker(worker_index, std::move(job)) : JobHandle{};
}

void JobSystem::wait(JobHandle handle)
{
    if (impl_)
    {
        impl_->wait(handle);
    }
}

void JobSystem::wait_all(std::span<const JobHandle> handles)
{
    for (const auto &handle : handles)
    {
        wait(handle);
    }
}

// NOLINTEND(readability-identifier-naming,misc-non-private-member-variables-in-classes,readability-function-cognitive-complexity,readability-convert-member-functions-to-static)

} // namespace phynity::jobs

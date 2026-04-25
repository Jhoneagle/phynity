#include "job_system.hpp"

#include "work_stealing_deque.hpp"

#include <platform/allocation_tracker.hpp>
#include <platform/thread_affinity.hpp>
#include <platform/threading.hpp>

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

namespace phynity::jobs
{

// Internal PIMPL implementation intentionally uses compact naming and state layout.
// NOLINTBEGIN(readability-identifier-naming,misc-non-private-member-variables-in-classes,readability-function-cognitive-complexity,readability-convert-member-functions-to-static)

/// Lifecycle states for a job slot. Packed into the low 2 bits of state_gen.
enum class SlotState : uint32_t
{
    Free = 0,      // Slot available for reuse
    Queued = 1,    // Job assigned, waiting in worker deque
    Running = 2,   // Worker is executing the job function
    Completed = 3, // Execution finished, waiter can collect
};

static constexpr uint32_t kStateBits = 2;
static constexpr uint32_t kStateMask = 0x3;
static constexpr uint32_t kGenShift = kStateBits;
static constexpr uint32_t kGenMask = 0x3FFFFFFF; // 30-bit generation

inline SlotState unpack_state(uint32_t packed)
{
    return static_cast<SlotState>(packed & kStateMask);
}
inline uint32_t unpack_gen(uint32_t packed)
{
    return packed >> kGenShift;
}
inline uint32_t pack(SlotState s, uint32_t gen)
{
    return (gen << kGenShift) | static_cast<uint32_t>(s);
}

/// Pre-allocated job slot with state machine lifecycle.
/// Uses packed state+generation in a single atomic to prevent race conditions
/// between slot reuse, execution, and completion.
struct JobSlot
{
    JobFn fn;
    std::atomic<uint32_t> state_gen{0}; // packed: [31:2]=generation, [1:0]=SlotState
    std::condition_variable done_cv;
    std::mutex done_mutex; // only for condition_variable wait/notify protocol
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
    std::deque<JobSlot> job_pool_;
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

    assert(pending_work_.load(std::memory_order_relaxed) == 0 &&
           "pending_work_ counter drifted — submit/complete mismatch");
}

JobSystemImpl::~JobSystemImpl()
{
    shutdown();
}

JobHandle JobSystemImpl::submit(JobFn job)
{
    uint32_t target =
        submit_counter_.fetch_add(1, std::memory_order_relaxed) % static_cast<uint32_t>(worker_queues_.size());
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
    uint32_t gen = (job_id / pool_capacity_ + 1) & kGenMask;

    auto &slot = job_pool_[slot_index];

    // CAS loop: wait until slot is Free or Completed, then claim it as Queued.
    // With pool_capacity_=4096 and typical ~20 tasks/frame, this rarely loops.
    for (;;)
    {
        uint32_t cur = slot.state_gen.load(std::memory_order_acquire);
        SlotState st = unpack_state(cur);

        if (st == SlotState::Free || st == SlotState::Completed)
        {
            uint32_t desired = pack(SlotState::Queued, gen);
            if (slot.state_gen.compare_exchange_weak(cur, desired, std::memory_order_acq_rel,
                                                     std::memory_order_relaxed))
            {
                break; // We own the slot in Queued state
            }
            continue; // CAS failed, retry
        }

        // Slot is Queued or Running — previous job still in flight
        platform::yield_thread();
    }

    // We exclusively own the slot in Queued state with our generation.
    slot.fn = std::move(job);

    worker_queues_[target_worker]->push(job_id);
    pending_work_.fetch_add(1, std::memory_order_release);

    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        wake_cv_.notify_one();
    }

    return JobHandle{job_id, gen};
}

void JobSystemImpl::wait(JobHandle handle)
{
    if (!handle.valid() || !running_.load())
    {
        return;
    }

    uint32_t slot_index = handle.id % pool_capacity_;
    auto &slot = job_pool_[slot_index];
    uint32_t completed_val = pack(SlotState::Completed, handle.generation);

    std::unique_lock<std::mutex> lock(slot.done_mutex);
    slot.done_cv.wait(lock,
                      [&slot, &handle, completed_val]
                      {
                          uint32_t cur = slot.state_gen.load(std::memory_order_acquire);
                          // Wake if: completed with our generation, OR generation changed (recycled)
                          return cur == completed_val || unpack_gen(cur) != handle.generation;
                      });

    // If completed with our generation, transition to Free for reuse.
    // Worker already cleared slot.fn before storing Completed.
    uint32_t cur = slot.state_gen.load(std::memory_order_acquire);
    if (cur == completed_val)
    {
        slot.state_gen.compare_exchange_strong(cur, pack(SlotState::Free, 0), std::memory_order_release,
                                               std::memory_order_relaxed);
    }
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
            uint32_t expected_gen = (job_id / pool_capacity_ + 1) & kGenMask;
            auto &slot = job_pool_[slot_index];

            // CAS Queued->Running with generation validation.
            // If CAS fails, the slot was recycled (stale job_id) — skip silently.
            uint32_t expected = pack(SlotState::Queued, expected_gen);
            uint32_t desired = pack(SlotState::Running, expected_gen);

            if (slot.state_gen.compare_exchange_strong(expected, desired, std::memory_order_acq_rel,
                                                       std::memory_order_relaxed))
            {
                // CAS succeeded: we own execution rights.
                // acquire on CAS ensures we see slot.fn written by submit_impl.
                if (static_cast<bool>(slot.fn))
                {
                    slot.fn();
                }

                // Worker owns cleanup — clear fn before marking Completed.
                slot.fn = nullptr;

                // Transition Running->Completed (release publishes side-effects of fn())
                slot.state_gen.store(pack(SlotState::Completed, expected_gen), std::memory_order_release);

                // Notify waiters under done_mutex to prevent lost wakeups
                {
                    std::lock_guard<std::mutex> lock(slot.done_mutex);
                    slot.done_cv.notify_all();
                }
            }

            pending_work_.fetch_sub(1, std::memory_order_release);
        }
        else
        {
            // 3. No work found - wait with timeout
            std::unique_lock<std::mutex> lock(wake_mutex_);
            wake_cv_.wait_for(lock,
                              std::chrono::milliseconds(1),
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

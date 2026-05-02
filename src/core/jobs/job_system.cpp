#include "job_system.hpp"

#include "work_stealing_deque.hpp"

#include <platform/allocation_tracker.hpp>
#include <platform/thread_affinity.hpp>
#include <platform/threading.hpp>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstring>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace phynity::jobs
{

// Internal PIMPL implementation intentionally uses compact naming and state layout.
// NOLINTBEGIN(readability-identifier-naming,misc-non-private-member-variables-in-classes,readability-function-cognitive-complexity,readability-convert-member-functions-to-static)

/// RAII guard that tracks active external waiters so shutdown() can wait
/// for them to exit before destroying pool memory.
struct WaiterGuard
{
    std::atomic<uint32_t> &count;
    explicit WaiterGuard(std::atomic<uint32_t> &c) : count(c)
    {
        count.fetch_add(1, std::memory_order_acq_rel);
    }
    ~WaiterGuard()
    {
        count.fetch_sub(1, std::memory_order_acq_rel);
    }
    WaiterGuard(const WaiterGuard &) = delete;
    WaiterGuard &operator=(const WaiterGuard &) = delete;
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
        return running_.load(std::memory_order_acquire);
    }
    uint32_t worker_count() const noexcept
    {
        return worker_count_;
    }
    SchedulingMode scheduling_mode() const noexcept
    {
        return mode_;
    }

    JobId submit(JobFnPtr fn, void *data, const char *debug_name);
    JobId submit(JobFnPtr fn, void *data, CounterHandle counter, const char *debug_name);
    void wait(JobId id);
    void wait(CounterHandle counter);

    CounterHandle create_counter(int32_t count);
    CounterHandle submit_graph(const JobGraph &graph);

private:
    JobId submit_impl(JobFnPtr fn, void *data, CounterHandle counter, uint16_t affinity_hint, const char *debug_name);
    void enqueue(JobId id, uint16_t affinity_hint);
    void dispatch_dependents(Job &job);
    void worker_loop(uint32_t worker_index);
    void execute_serial(const JobGraph &graph);

    std::atomic<bool> running_{false};

    uint32_t worker_count_ = 0;
    SchedulingMode mode_ = SchedulingMode::Concurrent;

    phynity::platform::TrackedVector<platform::Thread> workers_;

    JobPool pool_{4096};
    CounterPool counters_{256};
    EventCount completion_event_;

    // Per-worker work-stealing deques (stores raw alloc indices from pool)
    std::vector<std::unique_ptr<WorkStealingDeque<uint32_t>>> worker_queues_;

    // Per-queue push mutex: the Chase-Lev deque's push() is single-producer only,
    // but enqueue() can be called from any thread (external submitters, workers
    // dispatching dependents). These mutexes serialize pushes to the same deque
    // while keeping pop() and steal() lock-free.
    std::vector<std::mutex> push_mutexes_;

    // Round-robin submit distribution
    std::atomic<uint32_t> submit_counter_{0};

    // Tracks external threads currently inside wait() so shutdown()
    // can defer pool destruction until they exit.
    std::atomic<uint32_t> active_waiters_{0};

    // Wakeup mechanism for workers
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

    // Create per-worker deques and their push mutexes
    worker_queues_.clear();
    push_mutexes_ = std::vector<std::mutex>(actual_workers);
    for (uint32_t i = 0; i < actual_workers; ++i)
    {
        worker_queues_.push_back(std::make_unique<WorkStealingDeque<uint32_t>>(10)); // capacity 1024
    }

    pending_work_.store(0);
    running_.store(true, std::memory_order_release);

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

    running_.store(false, std::memory_order_release);

    // Wake all workers
    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        wake_cv_.notify_all();
    }

    // Also notify anyone blocked on completion events
    completion_event_.notify_all();

    // Join all workers
    for (auto &worker : workers_)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
    workers_.clear();

    // Wait for external waiter threads to exit wait() before destroying
    // pool memory. Re-notify in case a waiter entered ec.wait() after our
    // initial notify_all().
    while (active_waiters_.load(std::memory_order_acquire) > 0)
    {
        completion_event_.notify_all();
        platform::yield_thread();
    }

    // Drain remaining jobs from all deques. Jobs submitted via submit_callable
    // heap-allocate the callable; the invoker frees it when executed. Jobs that
    // were never picked up by a worker would leak without this drain.
    for (auto &queue : worker_queues_)
    {
        uint32_t slot_index = 0;
        while (queue->pop(slot_index))
        {
            auto &job = pool_[slot_index];
            auto fn = job.function.load(std::memory_order_relaxed);
            auto d = job.data.load(std::memory_order_relaxed);
            if (fn && d)
            {
                try
                {
                    fn(d);
                }
                catch (...)
                {
                    // Swallow — we only care about triggering cleanup.
                }
            }
            job.function.store(nullptr, std::memory_order_relaxed);
            job.data.store(nullptr, std::memory_order_relaxed);
        }
    }

    worker_queues_.clear();

    pool_.clear_overflow();
}

JobSystemImpl::~JobSystemImpl()
{
    shutdown();
}

// =============================================================================
// Submission
// =============================================================================

JobId JobSystemImpl::submit(JobFnPtr fn, void *data, const char *debug_name)
{
    return submit_impl(fn, data, CounterHandle{}, (std::numeric_limits<uint16_t>::max)(), debug_name);
}

JobId JobSystemImpl::submit(JobFnPtr fn, void *data, CounterHandle counter, const char *debug_name)
{
    return submit_impl(fn, data, counter, (std::numeric_limits<uint16_t>::max)(), debug_name);
}

CounterHandle JobSystemImpl::create_counter(int32_t count)
{
    return counters_.acquire(count);
}

JobId JobSystemImpl::submit_impl(
    JobFnPtr fn, void *data, CounterHandle counter, uint16_t affinity_hint, const char *debug_name)
{
    if (!running_.load(std::memory_order_acquire))
    {
        return JobId{};
    }

    JobId id = pool_.allocate();
    auto &job = pool_.at(id);

    job.function.store(fn, std::memory_order_relaxed);
    job.data.store(data, std::memory_order_relaxed);
    job.predecessor_count.store(0, std::memory_order_relaxed);
    job.dependent_count = 0;
    job.affinity_hint = affinity_hint;
    job.debug_name = debug_name;
    job.overflow_offset = 0;
    job.overflow_count = 0;

    // Store counter handle in the job's inline_data if a counter is set.
    // We pack the counter handle into the first 8 bytes of inline_data.
    if (counter.valid())
    {
        static_assert(sizeof(CounterHandle) <= kMaxInlineDataSize);
        std::memcpy(static_cast<void *>(job.inline_data), &counter, sizeof(CounterHandle));
    }
    else
    {
        // Clear the counter slot
        CounterHandle empty{};
        std::memcpy(static_cast<void *>(job.inline_data), &empty, sizeof(CounterHandle));
    }

    // No predecessors — enqueue immediately
    enqueue(id, affinity_hint);
    return id;
}

void JobSystemImpl::enqueue(JobId id, uint16_t affinity_hint)
{
    uint32_t target;
    if (affinity_hint != (std::numeric_limits<uint16_t>::max)() && affinity_hint < worker_queues_.size())
    {
        target = affinity_hint;
    }
    else
    {
        target = submit_counter_.fetch_add(1, std::memory_order_relaxed) % static_cast<uint32_t>(worker_queues_.size());
    }

    // Try target queue first; if full, round-robin through others.
    // This prevents silent job loss when a single deque is saturated.
    // Each push is serialized per-queue because the Chase-Lev deque's push()
    // is single-producer only, but enqueue() runs from arbitrary threads.
    bool pushed = false;
    {
        std::lock_guard<std::mutex> lock(push_mutexes_[target]);
        pushed = worker_queues_[target]->push(id.index);
    }
    if (!pushed)
    {
        uint32_t queues = static_cast<uint32_t>(worker_queues_.size());
        for (uint32_t offset = 1; offset < queues; ++offset)
        {
            uint32_t alt = (target + offset) % queues;
            std::lock_guard<std::mutex> lock(push_mutexes_[alt]);
            if (worker_queues_[alt]->push(id.index))
            {
                pushed = true;
                break;
            }
        }
    }
    assert(pushed && "All worker deques full — increase deque capacity or reduce concurrent submissions");
    (void) pushed;

    pending_work_.fetch_add(1, std::memory_order_release);

    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        wake_cv_.notify_one();
    }
}

// =============================================================================
// Waiting
// =============================================================================

void JobSystemImpl::wait(JobId id)
{
    if (!id.valid())
    {
        return;
    }

    WaiterGuard guard(active_waiters_);
    auto &job = pool_.at(id);

    // The completion signal is function pointer atomically set to nullptr.
    // The worker does a release store; we do an acquire load.

    // Spin briefly
    for (int spin = 0; spin < 64; ++spin)
    {
        if (job.function.load(std::memory_order_acquire) == nullptr)
        {
            return;
        }
        if (!running_.load(std::memory_order_acquire))
        {
            return;
        }

#if defined(_MSC_VER)
        _mm_pause();
#else
        platform::yield_thread();
#endif
    }

    // Fall back to eventcount (mutex-based, provides full synchronization)
    for (;;)
    {
        auto token = completion_event_.prepare_wait();
        if (job.function.load(std::memory_order_acquire) == nullptr)
        {
            completion_event_.cancel_wait(token);
            return;
        }
        if (!running_.load(std::memory_order_acquire))
        {
            completion_event_.cancel_wait(token);
            return;
        }
        completion_event_.wait(token);
    }
}

void JobSystemImpl::wait(CounterHandle counter)
{
    WaiterGuard guard(active_waiters_);
    counters_.wait(counter, completion_event_, &running_);
}

// =============================================================================
// Graph submission
// =============================================================================

CounterHandle JobSystemImpl::submit_graph(const JobGraph &graph)
{
    const uint32_t n = graph.size();
    if (n == 0)
    {
        return CounterHandle{};
    }

    // In deterministic mode, execute serially on the calling thread
    if (mode_ == SchedulingMode::Deterministic || mode_ == SchedulingMode::DeterministicReplay || !running_.load())
    {
        execute_serial(graph);
        return CounterHandle{};
    }

    // Allocate a completion counter for the entire graph
    auto counter = counters_.acquire(static_cast<int32_t>(n));

    // Phase 1: Allocate pool slots for all graph nodes
    std::vector<JobId> pool_ids(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        pool_ids[i] = pool_.allocate();
    }

    // Phase 2: Configure each job slot with function, data, dependencies, and counter
    for (uint32_t i = 0; i < n; ++i)
    {
        const auto &desc = graph.desc(JobId{i, 0});
        auto &job = pool_.at(pool_ids[i]);

        job.function.store(desc.function, std::memory_order_relaxed);
        job.data.store(desc.data, std::memory_order_relaxed);
        job.affinity_hint = desc.affinity_hint;
        job.debug_name = desc.debug_name;

        // Set predecessor count from the graph
        job.predecessor_count.store(static_cast<int32_t>(graph.predecessor_count(JobId{i, 0})),
                                    std::memory_order_relaxed);

        // Wire up dependents: map graph-local indices to pool-allocated JobIds
        auto deps = graph.dependents(JobId{i, 0});
        uint32_t dep_count = static_cast<uint32_t>(deps.size());
        job.dependent_count = static_cast<uint16_t>((std::min)(dep_count, static_cast<uint32_t>(UINT16_MAX)));

        if (dep_count <= kMaxInlineDependents)
        {
            for (uint32_t d = 0; d < dep_count; ++d)
            {
                job.dependents[d] = pool_ids[deps[d].index];
            }
        }
        else
        {
            // Store inline portion
            for (uint32_t d = 0; d < kMaxInlineDependents; ++d)
            {
                job.dependents[d] = pool_ids[deps[d].index];
            }

            // Overflow portion
            uint32_t overflow_count = dep_count - kMaxInlineDependents;
            uint32_t offset = pool_.allocate_overflow(overflow_count);
            job.overflow_offset = offset;
            job.overflow_count = overflow_count;
            for (uint32_t d = 0; d < overflow_count; ++d)
            {
                pool_.overflow_dependent(offset + d) = pool_ids[deps[kMaxInlineDependents + d].index];
            }
        }

        // Store counter handle in inline_data
        static_assert(sizeof(CounterHandle) <= kMaxInlineDataSize);
        std::memcpy(static_cast<void *>(job.inline_data), &counter, sizeof(CounterHandle));
    }

    // Phase 3: Enqueue all root nodes (predecessor_count == 0)
    for (uint32_t i = 0; i < n; ++i)
    {
        if (graph.predecessor_count(JobId{i, 0}) == 0)
        {
            enqueue(pool_ids[i], graph.desc(JobId{i, 0}).affinity_hint);
        }
    }

    return counter;
}

void JobSystemImpl::execute_serial(const JobGraph &graph)
{
    // Kahn's algorithm: deterministic topological sort with stable tie-breaking
    const uint32_t n = graph.size();
    std::vector<uint32_t> in_degree(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        in_degree[i] = graph.predecessor_count(JobId{i, 0});
    }

    std::vector<JobId> current;
    for (uint32_t i = 0; i < n; ++i)
    {
        if (in_degree[i] == 0)
        {
            current.push_back(JobId{i, 0});
        }
    }
    std::sort(current.begin(), current.end());

    while (!current.empty())
    {
        std::vector<JobId> next;

        for (const auto &id : current)
        {
            const auto &desc = graph.desc(id);
            if (desc.function)
            {
                desc.function(desc.data);
            }

            for (const auto &child : graph.dependents(id))
            {
                assert(in_degree[child.index] > 0);
                --in_degree[child.index];
                if (in_degree[child.index] == 0)
                {
                    next.push_back(child);
                }
            }
        }

        std::sort(next.begin(), next.end());
        current = std::move(next);
    }
}

// =============================================================================
// Dependency dispatch (called after a job completes)
// =============================================================================

void JobSystemImpl::dispatch_dependents(Job &job)
{
    uint32_t total = job.dependent_count;
    uint32_t inline_count = (std::min)(total, static_cast<uint32_t>(kMaxInlineDependents));

    for (uint32_t d = 0; d < inline_count; ++d)
    {
        JobId dep_id = job.dependents[d];
        if (!dep_id.valid())
        {
            continue;
        }

        auto &dep_job = pool_.at(dep_id);
        int32_t prev = dep_job.predecessor_count.fetch_sub(1, std::memory_order_acq_rel);
        if (prev == 1)
        {
            // This was the last predecessor — enqueue the dependent
            enqueue(dep_id, dep_job.affinity_hint);
        }
    }

    // Overflow dependents
    for (uint32_t d = 0; d < job.overflow_count; ++d)
    {
        JobId dep_id = pool_.overflow_dependent(job.overflow_offset + d);
        if (!dep_id.valid())
        {
            continue;
        }

        auto &dep_job = pool_.at(dep_id);
        int32_t prev = dep_job.predecessor_count.fetch_sub(1, std::memory_order_acq_rel);
        if (prev == 1)
        {
            enqueue(dep_id, dep_job.affinity_hint);
        }
    }
}

// =============================================================================
// Worker loop
// =============================================================================

void JobSystemImpl::worker_loop(uint32_t worker_index)
{
    platform::set_thread_affinity(worker_index);

    auto &my_queue = *worker_queues_[worker_index];
    const uint32_t num_queues = static_cast<uint32_t>(worker_queues_.size());

    while (running_.load(std::memory_order_acquire))
    {
        uint32_t slot_index = 0;
        bool found = false;
        {
            std::lock_guard<std::mutex> lock(push_mutexes_[worker_index]);
            found = my_queue.pop(slot_index);
        }

        if (!found)
        {
            for (uint32_t offset = 1; offset < num_queues && !found; ++offset)
            {
                uint32_t victim = (worker_index + offset) % num_queues;
                found = worker_queues_[victim]->steal(slot_index);
            }
        }

        if (found)
        {
            auto &job = pool_[slot_index];

            // Execute the job. Catch exceptions to prevent a throwing job from
            // killing the worker thread, which would leave dependents and
            // counters permanently stuck.
            auto fn = job.function.load(std::memory_order_relaxed);
            if (fn)
            {
                try
                {
                    fn(job.data.load(std::memory_order_relaxed));
                }
                catch (...)
                {
                    // Job failed — fall through to still dispatch dependents,
                    // decrement counters, and mark complete so the system
                    // doesn't deadlock.
                }
            }

            // Dispatch dependents (atomic predecessor decrement)
            dispatch_dependents(job);

            // Decrement completion counter if set
            CounterHandle counter;
            std::memcpy(&counter, static_cast<const void *>(job.inline_data), sizeof(CounterHandle));
            if (counter.valid())
            {
                counters_.decrement(counter, completion_event_);
            }

            // Mark job as complete. Store data first (relaxed), then function
            // (release) — function==nullptr is the completion signal; acquire
            // loads by waiters will see all prior writes.
            job.data.store(nullptr, std::memory_order_relaxed);
            job.function.store(nullptr, std::memory_order_release);

            // Notify anyone waiting on this specific job or a counter
            completion_event_.notify_all();

            pending_work_.fetch_sub(1, std::memory_order_release);
        }
        else
        {
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

JobId JobSystem::submit(JobFnPtr fn, void *data, const char *debug_name)
{
    return impl_ ? impl_->submit(fn, data, debug_name) : JobId{};
}

JobId JobSystem::submit(JobFnPtr fn, void *data, CounterHandle counter, const char *debug_name)
{
    return impl_ ? impl_->submit(fn, data, counter, debug_name) : JobId{};
}

void JobSystem::wait(JobId id)
{
    if (impl_)
    {
        impl_->wait(id);
    }
}

void JobSystem::wait(CounterHandle counter)
{
    if (impl_)
    {
        impl_->wait(counter);
    }
}

CounterHandle JobSystem::create_counter(int32_t count)
{
    return impl_ ? impl_->create_counter(count) : CounterHandle{};
}

CounterHandle JobSystem::submit_graph(const JobGraph &graph)
{
    return impl_ ? impl_->submit_graph(graph) : CounterHandle{};
}

void JobSystem::wait_all(std::span<const JobId> ids)
{
    for (const auto &id : ids)
    {
        wait(id);
    }
}

// NOLINTEND(readability-identifier-naming,misc-non-private-member-variables-in-classes,readability-function-cognitive-complexity,readability-convert-member-functions-to-static)

} // namespace phynity::jobs

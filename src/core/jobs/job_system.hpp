#pragma once

#include "counter.hpp"
#include "eventcount.hpp"
#include "job.hpp"
#include "job_graph.hpp"
#include "job_id.hpp"
#include "job_pool.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

namespace phynity::jobs
{

enum class SchedulingMode : uint8_t
{
    Concurrent,
    Deterministic,
    DeterministicReplay // Replay a previously recorded schedule for bit-identical results
};

struct JobSystemConfig
{
    uint32_t worker_count = 0; // 0 = use hardware concurrency
    SchedulingMode mode = SchedulingMode::Concurrent;
};

class JobSystemImpl; // forward declaration for PIMPL

class JobSystem
{
public:
    JobSystem();
    explicit JobSystem(const JobSystemConfig &config);
    ~JobSystem();

    JobSystem(const JobSystem &) = delete;
    JobSystem &operator=(const JobSystem &) = delete;
    JobSystem(JobSystem &&) noexcept;
    JobSystem &operator=(JobSystem &&) noexcept;

    void start(const JobSystemConfig &config);
    void shutdown();

    [[nodiscard]] bool is_running() const noexcept;
    [[nodiscard]] uint32_t worker_count() const noexcept;
    [[nodiscard]] SchedulingMode scheduling_mode() const noexcept;

    // ========================================================================
    // Direct job submission (no dependencies)
    // ========================================================================

    /// Submit a job using a raw function pointer + data.
    JobId submit(JobFnPtr fn, void *data = nullptr, const char *debug_name = nullptr);

    /// Submit a job that decrements a completion counter when done.
    JobId submit(JobFnPtr fn, void *data, CounterHandle counter, const char *debug_name = nullptr);

    /// Wait for a single job to complete.
    void wait(JobId id);

    // ========================================================================
    // Counter-based completion groups
    // ========================================================================

    /// Create a completion counter initialized to `count`.
    [[nodiscard]] CounterHandle create_counter(int32_t count);

    /// Block until the counter reaches zero.
    void wait(CounterHandle counter);

    // ========================================================================
    // Graph submission (dependency-driven dispatch)
    // ========================================================================

    /// Submit an entire dependency graph. Returns a counter that reaches zero
    /// when all jobs in the graph have completed.
    CounterHandle submit_graph(const JobGraph &graph);

    // ========================================================================
    // Convenience APIs
    // ========================================================================

    /// Submit a callable as a job. The callable must be trivially copyable and
    /// fit within kMaxInlineDataSize bytes (32 bytes). For larger captures,
    /// use the raw function pointer API.
    template <typename Fn> JobId submit(Fn &&fn, const char *debug_name = nullptr)
    {
        return submit_callable(std::forward<Fn>(fn), CounterHandle{}, debug_name);
    }

    /// Submit a callable that decrements a counter when done.
    template <typename Fn> JobId submit(Fn &&fn, CounterHandle counter, const char *debug_name = nullptr)
    {
        return submit_callable(std::forward<Fn>(fn), counter, debug_name);
    }

    /// Wait for all jobs in a span of JobIds.
    void wait_all(std::span<const JobId> ids);

    /// Parallel for over [begin, end) with grain-size chunking.
    template <typename IndexFn> void parallel_for(uint32_t begin, uint32_t end, uint32_t grain, IndexFn &&fn)
    {
        if (begin >= end)
        {
            return;
        }

        uint32_t count = end - begin;
        uint32_t effective_grain = (grain > 0) ? grain : count;

        // Serial fallback: system not running, small range, or deterministic/replay mode
        if (!is_running() || count <= effective_grain || scheduling_mode() == SchedulingMode::Deterministic ||
            scheduling_mode() == SchedulingMode::DeterministicReplay)
        {
            for (uint32_t i = begin; i < end; ++i)
            {
                fn(i);
            }
            return;
        }

        uint32_t chunk_count = (count + effective_grain - 1) / effective_grain;
        auto counter = create_counter(static_cast<int32_t>(chunk_count));

        for (uint32_t chunk_begin = begin; chunk_begin < end; chunk_begin += effective_grain)
        {
            uint32_t chunk_end = (std::min)(chunk_begin + effective_grain, end);
            submit(
                [chunk_begin, chunk_end, &fn]()
                {
                    for (uint32_t i = chunk_begin; i < chunk_end; ++i)
                    {
                        fn(i);
                    }
                },
                counter);
        }

        wait(counter);
    }

private:
    template <typename Fn> JobId submit_callable(Fn &&fn, CounterHandle counter, const char *debug_name);

    std::unique_ptr<JobSystemImpl> impl_;
};

// Template implementation (must be in header)
template <typename Fn> JobId JobSystem::submit_callable(Fn &&fn, CounterHandle counter, const char *debug_name)
{
    // Heap-allocate the callable. Lambdas with captures are not trivially copyable,
    // so inline storage would require a more complex submission path that patches
    // the Job's inline_data after allocation. For simplicity and correctness, always
    // heap-allocate here. The raw function pointer API (used by physics systems)
    // avoids this allocation entirely.
    using FnType = std::decay_t<Fn>;
    auto *heap_fn = new FnType(std::forward<Fn>(fn));

    auto invoker = [](void *data)
    {
        auto *f = static_cast<FnType *>(data);
        (*f)();
        delete f;
    };

    JobId id = submit(+invoker, static_cast<void *>(heap_fn), counter, debug_name);
    if (!id.valid())
    {
        // System not running — invoker won't execute, so clean up the heap allocation
        delete heap_fn;
    }
    return id;
}

} // namespace phynity::jobs

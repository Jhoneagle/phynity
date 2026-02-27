#pragma once

#include "job_handle.hpp"

#include <cstdint>
#include <functional>
#include <span>

namespace phynity::jobs {

enum class SchedulingMode : uint8_t {
    Concurrent,
    Deterministic
};

struct JobSystemConfig {
    uint32_t worker_count = 0;           // 0 = use hardware concurrency
    SchedulingMode mode = SchedulingMode::Concurrent;
    uint32_t queue_capacity = 1024;
};

using JobFn = std::function<void()>;

class JobSystem {
public:
    JobSystem() = default;
    explicit JobSystem(const JobSystemConfig& config);

    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    void start(const JobSystemConfig& config);
    void shutdown();

    [[nodiscard]] bool is_running() const noexcept;
    [[nodiscard]] uint32_t worker_count() const noexcept;
    [[nodiscard]] SchedulingMode scheduling_mode() const noexcept;

    JobHandle submit(JobFn job);
    void wait(JobHandle handle);
    void wait_all(std::span<const JobHandle> handles);

    template <typename IndexFn>
    void parallel_for(uint32_t begin, uint32_t end, uint32_t grain, IndexFn&& fn) {
        (void)grain;
        // Sketch: serial fallback until the worker system is implemented.
        for (uint32_t i = begin; i < end; ++i) {
            fn(i);
        }
    }

private:
    JobSystemConfig config_{};
    bool running_ = false;
};

}  // namespace phynity::jobs

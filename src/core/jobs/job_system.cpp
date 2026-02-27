#include "job_system.hpp"

#include <platform/threading.hpp>

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <vector>

namespace phynity::jobs {

struct JobEntry {
    JobFn fn;
    std::atomic<bool> completed{false};
    std::condition_variable done_cv;
    std::mutex done_mutex;

    JobEntry() = default;
    explicit JobEntry(JobFn f) : fn(std::move(f)), completed(false) {}
};

class JobSystemImpl {
public:
    explicit JobSystemImpl(const JobSystemConfig& config);
    ~JobSystemImpl();

    void start(const JobSystemConfig& config);
    void shutdown();

    bool is_running() const noexcept { return running_.load(); }
    uint32_t worker_count() const noexcept { return worker_count_; }
    SchedulingMode scheduling_mode() const noexcept { return mode_; }

    JobHandle submit(JobFn job);
    void wait(JobHandle handle);

private:
    void worker_loop();

    std::atomic<bool> running_{false};
    std::atomic<uint32_t> shutdown_signal_{0};

    uint32_t worker_count_ = 0;
    SchedulingMode mode_ = SchedulingMode::Concurrent;

    std::vector<platform::Thread> workers_;

    std::mutex jobs_mutex_;
    std::map<uint32_t, std::unique_ptr<JobEntry>> jobs_;
    uint32_t next_job_id_{1};

    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::vector<uint32_t> job_queue_;
};

JobSystemImpl::JobSystemImpl(const JobSystemConfig& config) {
    start(config);
}

void JobSystemImpl::start(const JobSystemConfig& config) {
    if (running_.load()) {
        return;
    }

    mode_ = config.mode;
    worker_count_ = config.worker_count;
    if (worker_count_ == 0) {
        worker_count_ = platform::hardware_concurrency();
    }

    shutdown_signal_.store(0);
    running_.store(true);

    // In deterministic mode, use single worker thread.
    uint32_t actual_workers = (mode_ == SchedulingMode::Deterministic) ? 1 : worker_count_;

    for (uint32_t i = 0; i < actual_workers; ++i) {
        workers_.emplace_back([this] { worker_loop(); });
    }
}

void JobSystemImpl::shutdown() {
    if (!running_.load()) {
        return;
    }

    running_.store(false);
    shutdown_signal_.store(1);

    // Wake all workers
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_cv_.notify_all();
    }

    // Join all workers
    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    workers_.clear();
}

JobSystemImpl::~JobSystemImpl() {
    shutdown();
}

JobHandle JobSystemImpl::submit(JobFn job) {
    if (!running_.load()) {
        return JobHandle{};
    }

    uint32_t job_id;
    {
        std::lock_guard<std::mutex> lock(jobs_mutex_);
        job_id = next_job_id_++;
        jobs_[job_id] = std::make_unique<JobEntry>(std::move(job));
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        job_queue_.push_back(job_id);
        queue_cv_.notify_one();
    }

    return JobHandle{job_id, 1};
}

void JobSystemImpl::wait(JobHandle handle) {
    if (!handle.valid() || !running_.load()) {
        return;
    }

    JobEntry* entry = nullptr;
    {
        std::lock_guard<std::mutex> lock(jobs_mutex_);
        auto it = jobs_.find(handle.id);
        if (it == jobs_.end()) {
            return;
        }
        entry = it->second.get();
    }

    // Wait for job completion
    {
        std::unique_lock<std::mutex> lock(entry->done_mutex);
        entry->done_cv.wait(lock, [entry] { return entry->completed.load(); });
    }

    // Clean up after waiting
    {
        std::lock_guard<std::mutex> lock(jobs_mutex_);
        jobs_.erase(handle.id);
    }
}

void JobSystemImpl::worker_loop() {
    while (running_.load()) {
        uint32_t job_id = 0;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !job_queue_.empty() || !running_.load();
            });

            if (!running_.load()) {
                break;
            }

            if (!job_queue_.empty()) {
                job_id = job_queue_.front();
                job_queue_.erase(job_queue_.begin());
            }
        }

        if (job_id == 0) {
            continue;
        }

        JobEntry* entry = nullptr;
        {
            std::lock_guard<std::mutex> lock(jobs_mutex_);
            auto it = jobs_.find(job_id);
            if (it != jobs_.end()) {
                entry = it->second.get();
            }
        }

        if (entry && entry->fn) {
            entry->fn();
            entry->completed.store(true);
            entry->done_cv.notify_all();
        }
    }
}

// =============================================================================
// JobSystem public interface (thin wrapper)
// =============================================================================

static thread_local std::unique_ptr<JobSystemImpl> g_job_system;

JobSystem::JobSystem(const JobSystemConfig& config) {
    start(config);
}

void JobSystem::start(const JobSystemConfig& config) {
    if (!g_job_system) {
        g_job_system = std::make_unique<JobSystemImpl>(config);
    }
}

void JobSystem::shutdown() {
    if (g_job_system) {
        g_job_system->shutdown();
        g_job_system.reset();
    }
}

bool JobSystem::is_running() const noexcept {
    return g_job_system && g_job_system->is_running();
}

uint32_t JobSystem::worker_count() const noexcept {
    return g_job_system ? g_job_system->worker_count() : 0;
}

SchedulingMode JobSystem::scheduling_mode() const noexcept {
    return g_job_system ? g_job_system->scheduling_mode() : SchedulingMode::Concurrent;
}

JobHandle JobSystem::submit(JobFn job) {
    return g_job_system ? g_job_system->submit(std::move(job)) : JobHandle{};
}

void JobSystem::wait(JobHandle handle) {
    if (g_job_system) {
        g_job_system->wait(handle);
    }
}

void JobSystem::wait_all(std::span<const JobHandle> handles) {
    for (const auto& handle : handles) {
        wait(handle);
    }
}

}  // namespace phynity::jobs

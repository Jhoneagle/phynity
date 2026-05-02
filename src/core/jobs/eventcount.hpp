#pragma once

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace phynity::jobs
{

/// A shared signaling primitive for efficient spin-then-block waiting.
///
/// Replaces per-slot mutex+CV with a single shared instance.
/// Uses an epoch counter to avoid lost wakeups: waiters capture the epoch
/// before checking their condition, then only block if the epoch hasn't advanced.
///
/// Typical usage:
///   auto token = ec.prepare_wait();
///   if (condition_met()) { ec.cancel_wait(token); return; }
///   ec.wait(token);
class EventCount
{
public:
    using Token = uint32_t;

    /// Capture the current epoch. Call before checking your condition.
    [[nodiscard]] Token prepare_wait() noexcept
    {
        waiters_.fetch_add(1, std::memory_order_acq_rel);
        return epoch_.load(std::memory_order_acquire);
    }

    /// Cancel a wait that was prepared but not needed (condition already met).
    void cancel_wait([[maybe_unused]] Token token) noexcept
    {
        waiters_.fetch_sub(1, std::memory_order_release);
    }

    /// Block until the epoch advances past `token`.
    /// Must be called after prepare_wait() returned `token`.
    void wait(Token token)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this, token] { return epoch_.load(std::memory_order_acquire) != token; });
        waiters_.fetch_sub(1, std::memory_order_release);
    }

    /// Advance the epoch and wake one waiting thread.
    /// The mutex is locked before advancing the epoch so that the epoch
    /// change and the CV notification are atomic with respect to waiters
    /// entering cv_.wait. Without this, a waiter can capture the new epoch
    /// in prepare_wait() and then block forever because the notification
    /// already fired before it entered cv_.wait.
    void notify_one() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        epoch_.fetch_add(1, std::memory_order_release);
        cv_.notify_one();
    }

    /// Advance the epoch and wake all waiting threads.
    void notify_all() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        epoch_.fetch_add(1, std::memory_order_release);
        cv_.notify_all();
    }

private:
    std::atomic<uint32_t> epoch_{0};
    std::atomic<uint32_t> waiters_{0};
    std::mutex mutex_;
    std::condition_variable cv_;
};

} // namespace phynity::jobs

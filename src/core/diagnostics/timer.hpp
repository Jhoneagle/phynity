#pragma once

#include <chrono>
#include <cstdint>

namespace phynity::diagnostics {

/**
 * @brief High-resolution timer for performance measurements.
 * 
 * Provides microsecond-precision timing using std::chrono::high_resolution_clock.
 * Supports lap/split timing and multiple start/stop cycles.
 * 
 * Usage:
 * ```cpp
 * Timer timer;
 * timer.start();
 * // ... code to measure
 * timer.stop();
 * auto us = timer.elapsed_microseconds();
 * ```
 * 
 * Or RAII-style with ScopedTimer:
 * ```cpp
 * uint64_t duration_us = 0;
 * {
 *     ScopedTimer timer(duration_us);
 *     // ... code to measure
 * } // duration_us is set on scope exit
 * ```
 */
class Timer {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::nanoseconds;

    Timer() noexcept = default;

    /**
     * @brief Start or restart the timer.
     * 
     * Records the current time as the start point.
     * Can be called multiple times to restart timing.
     */
    void start() noexcept {
        start_time_ = Clock::now();
        is_running_ = true;
    }

    /**
     * @brief Stop the timer and record the end time.
     * 
     * After stopping, elapsed time can be queried.
     * Does nothing if timer is not running.
     */
    void stop() noexcept {
        if (is_running_) {
            end_time_ = Clock::now();
            is_running_ = false;
        }
    }

    /**
     * @brief Record a lap time without stopping the timer.
     * 
     * @return Elapsed microseconds since start
     * 
     * Useful for split/lap timing where you want to measure
     * multiple intervals without restarting the timer.
     */
    [[nodiscard]] uint64_t lap() noexcept {
        lap_time_ = Clock::now();
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            lap_time_ - start_time_
        ).count());
    }

    /**
     * @brief Get elapsed time in microseconds.
     * 
     * @return Elapsed microseconds between start() and stop()
     * 
     * If timer is still running, returns time since start().
     * If timer was never started, returns 0.
     */
    [[nodiscard]] uint64_t elapsed_microseconds() const noexcept {
        if (!is_running_ && start_time_ == TimePoint{}) {
            return 0;  // Never started
        }

        const TimePoint end = is_running_ ? Clock::now() : end_time_;
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            end - start_time_
        ).count());
    }

    /**
     * @brief Get elapsed time in milliseconds.
     * 
     * @return Elapsed milliseconds (as double for fractional precision)
     */
    [[nodiscard]] double elapsed_milliseconds() const noexcept {
        return static_cast<double>(elapsed_microseconds()) / 1000.0;
    }

    /**
     * @brief Get elapsed time in seconds.
     * 
     * @return Elapsed seconds (as double for fractional precision)
     */
    [[nodiscard]] double elapsed_seconds() const noexcept {
        return static_cast<double>(elapsed_microseconds()) / 1000000.0;
    }

    /**
     * @brief Check if the timer is currently running.
     */
    [[nodiscard]] bool is_running() const noexcept {
        return is_running_;
    }

    /**
     * @brief Reset the timer to initial state.
     * 
     * Clears all recorded times. Timer must be restarted after reset.
     */
    void reset() noexcept {
        start_time_ = TimePoint{};
        end_time_ = TimePoint{};
        lap_time_ = TimePoint{};
        is_running_ = false;
    }

private:
    TimePoint start_time_{};
    TimePoint end_time_{};
    TimePoint lap_time_{};
    bool is_running_ = false;
};

/**
 * @brief RAII-style scoped timer that automatically measures duration.
 * 
 * On construction, starts timing. On destruction, records elapsed time
 * to the provided output variable.
 * 
 * Usage:
 * ```cpp
 * uint64_t duration_us = 0;
 * {
 *     ScopedTimer timer(duration_us);
 *     expensive_operation();
 * }
 * // duration_us now contains elapsed time
 * ```
 */
class ScopedTimer {
public:
    /**
     * @brief Construct scoped timer and start timing.
     * 
     * @param out_duration_us Reference to variable that will receive elapsed microseconds
     */
    explicit ScopedTimer(uint64_t& out_duration_us) noexcept
        : out_duration_(out_duration_us) {
        timer_.start();
    }

    /**
     * @brief Destruct and record elapsed time to output variable.
     */
    ~ScopedTimer() noexcept {
        timer_.stop();
        out_duration_ = timer_.elapsed_microseconds();
    }

    // Non-copyable, non-movable (RAII guard)
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
    ScopedTimer(ScopedTimer&&) = delete;
    ScopedTimer& operator=(ScopedTimer&&) = delete;

private:
    Timer timer_;
    uint64_t& out_duration_;
};

} // namespace phynity::diagnostics

#pragma once

#include "profiler.hpp"
#include <vector>
#include <cstdint>
#include <string_view>
#include <algorithm>
#include <limits>

namespace phynity::diagnostics {

/**
 * @brief Complete profile data for a single frame.
 * 
 * Contains all timed zones recorded during one frame, plus frame metadata.
 */
struct FrameProfile {
    uint64_t frame_number = 0;          ///< Frame index
    uint64_t total_frame_time_us = 0;   ///< Total frame duration in microseconds
    std::vector<ProfileZone> zones;     ///< All profile zones recorded in this frame
    
    FrameProfile() = default;
    
    explicit FrameProfile(uint64_t frame_num)
        : frame_number(frame_num)
    {}
    
    void clear() noexcept {
        zones.clear();
        total_frame_time_us = 0;
    }
};

/**
 * @brief Statistics for a specific profiling zone across multiple frames.
 */
struct ZoneStats {
    std::string_view name;              ///< Zone name
    uint64_t min_duration_us = std::numeric_limits<uint64_t>::max();  ///< Minimum duration
    uint64_t max_duration_us = 0;       ///< Maximum duration
    uint64_t total_duration_us = 0;     ///< Sum of all durations
    uint32_t call_count = 0;            ///< Number of times zone appeared
    
    /**
     * @brief Get average duration across all calls.
     */
    [[nodiscard]] double average_duration_us() const noexcept {
        return call_count > 0 ? static_cast<double>(total_duration_us) / call_count : 0.0;
    }
    
    /**
     * @brief Get average duration in milliseconds.
     */
    [[nodiscard]] double average_duration_ms() const noexcept {
        return average_duration_us() / 1000.0;
    }
};

/**
 * @brief Frame profiler with history tracking and statistics.
 * 
 * Maintains a ring buffer of recent frames and provides aggregated statistics.
 * Call begin_frame() at the start of each frame and end_frame() at the end.
 * 
 * Usage:
 * ```cpp
 * FrameProfiler profiler(60);  // Track last 60 frames
 * 
 * while (running) {
 *     profiler.begin_frame();
 *     
 *     {
 *         PROFILE_SCOPE("update");
 *         update();
 *     }
 *     {
 *         PROFILE_SCOPE("render");
 *         render();
 *     }
 *     
 *     profiler.end_frame();
 *     
 *     // Optionally print stats
 *     auto avg = profiler.get_average_frame_time(30);
 *     printf("Avg frame time (30 frames): %.2f ms\n", avg / 1000.0);
 * }
 * ```
 */
class FrameProfiler {
public:
    /**
     * @brief Construct frame profiler with specified history size.
     * 
     * @param history_size Number of frames to keep in ring buffer (default: 60)
     */
    explicit FrameProfiler(size_t history_size = 60)
        : history_size_(history_size)
        , current_frame_index_(0)
        , frame_counter_(0)
        , frame_start_time_us_(0)
    {
        frame_history_.resize(history_size_);
    }
    
    /**
     * @brief Begin a new frame.
     * 
     * Records the start time and prepares for zone collection.
     * Call this at the very start of your frame/update loop.
     */
    void begin_frame() noexcept {
        Profiler::clear_frame();
        frame_start_time_us_ = static_cast<uint64_t>(Timer::Clock::now().time_since_epoch().count() / 1000);
    }
    
    /**
     * @brief End the current frame and record its profile.
     * 
     * Captures all zones from the profiler, computes frame time,
     * and stores in ring buffer. Call this at the very end of your frame.
     */
    void end_frame() noexcept {
        const uint64_t frame_end_time_us = static_cast<uint64_t>(Timer::Clock::now().time_since_epoch().count() / 1000);
        
        // Get current frame entry in ring buffer
        FrameProfile& frame = frame_history_[current_frame_index_];
        frame.frame_number = frame_counter_;
        frame.total_frame_time_us = frame_end_time_us - frame_start_time_us_;
        
        // Copy zones from profiler
        frame.zones = Profiler::get_zones();
        
        // Advance to next frame
        current_frame_index_ = (current_frame_index_ + 1) % history_size_;
        ++frame_counter_;
    }
    
    /**
     * @brief Get the most recently completed frame.
     * 
     * @return Reference to last frame's profile data
     * 
     * Note: Returns the frame before the current one if begin_frame()
     * has been called but end_frame() hasn't completed yet.
     */
    [[nodiscard]] const FrameProfile& get_last_frame() const noexcept {
        const size_t last_index = (current_frame_index_ + history_size_ - 1) % history_size_;
        return frame_history_[last_index];
    }
    
    /**
     * @brief Get a specific frame from history.
     * 
     * @param frames_ago How many frames back (0 = most recent)
     * @return Reference to frame profile, or empty frame if out of range
     */
    [[nodiscard]] const FrameProfile& get_frame(size_t frames_ago) const noexcept {
        if (frames_ago >= history_size_ || frames_ago >= frame_counter_) {
            static const FrameProfile empty_frame;
            return empty_frame;
        }
        
        const size_t index = (current_frame_index_ + history_size_ - frames_ago - 1) % history_size_;
        return frame_history_[index];
    }
    
    /**
     * @brief Get average frame time over recent frames.
     * 
     * @param num_frames Number of recent frames to average (default: all in history)
     * @return Average frame time in microseconds
     */
    [[nodiscard]] uint64_t get_average_frame_time(size_t num_frames = 0) const noexcept {
        if (num_frames == 0 || num_frames > history_size_) {
            num_frames = history_size_;
        }
        
        const size_t frames_to_check = std::min(num_frames, static_cast<size_t>(frame_counter_));
        if (frames_to_check == 0) {
            return 0;
        }
        
        uint64_t total = 0;
        for (size_t i = 0; i < frames_to_check; ++i) {
            total += get_frame(i).total_frame_time_us;
        }
        
        return total / frames_to_check;
    }
    
    /**
     * @brief Get statistics for a specific zone across recent frames.
     * 
     * @param zone_name Name of the zone to analyze
     * @param num_frames Number of recent frames to analyze (0 = all in history)
     * @return Statistics for the zone
     */
    [[nodiscard]] ZoneStats get_zone_stats(std::string_view zone_name, size_t num_frames = 0) const noexcept {
        if (num_frames == 0 || num_frames > history_size_) {
            num_frames = history_size_;
        }
        
        const size_t frames_to_check = std::min(num_frames, static_cast<size_t>(frame_counter_));
        
        ZoneStats stats;
        stats.name = zone_name;
        
        for (size_t i = 0; i < frames_to_check; ++i) {
            const FrameProfile& frame = get_frame(i);
            
            for (const auto& zone : frame.zones) {
                if (zone.name == zone_name) {
                    stats.min_duration_us = std::min(stats.min_duration_us, zone.duration_us);
                    stats.max_duration_us = std::max(stats.max_duration_us, zone.duration_us);
                    stats.total_duration_us += zone.duration_us;
                    ++stats.call_count;
                }
            }
        }
        
        // Reset min if no calls found
        if (stats.call_count == 0) {
            stats.min_duration_us = 0;
        }
        
        return stats;
    }
    
    /**
     * @brief Get minimum frame time in recent history.
     * 
     * @param num_frames Number of frames to check (0 = all)
     * @return Minimum frame time in microseconds
     */
    [[nodiscard]] uint64_t get_min_frame_time(size_t num_frames = 0) const noexcept {
        if (num_frames == 0 || num_frames > history_size_) {
            num_frames = history_size_;
        }
        
        const size_t frames_to_check = std::min(num_frames, static_cast<size_t>(frame_counter_));
        if (frames_to_check == 0) {
            return 0;
        }
        
        uint64_t min_time = std::numeric_limits<uint64_t>::max();
        for (size_t i = 0; i < frames_to_check; ++i) {
            min_time = std::min(min_time, get_frame(i).total_frame_time_us);
        }
        
        return min_time;
    }
    
    /**
     * @brief Get maximum frame time in recent history.
     * 
     * @param num_frames Number of frames to check (0 = all)
     * @return Maximum frame time in microseconds
     */
    [[nodiscard]] uint64_t get_max_frame_time(size_t num_frames = 0) const noexcept {
        if (num_frames == 0 || num_frames > history_size_) {
            num_frames = history_size_;
        }
        
        const size_t frames_to_check = std::min(num_frames, static_cast<size_t>(frame_counter_));
        if (frames_to_check == 0) {
            return 0;
        }
        
        uint64_t max_time = 0;
        for (size_t i = 0; i < frames_to_check; ++i) {
            max_time = std::max(max_time, get_frame(i).total_frame_time_us);
        }
        
        return max_time;
    }
    
    /**
     * @brief Get current frame counter.
     * 
     * @return Total number of frames processed
     */
    [[nodiscard]] uint64_t get_frame_count() const noexcept {
        return frame_counter_;
    }
    
    /**
     * @brief Get frame history buffer size.
     * 
     * @return Number of frames kept in history
     */
    [[nodiscard]] size_t get_history_size() const noexcept {
        return history_size_;
    }
    
    /**
     * @brief Clear all frame history.
     * 
     * Resets frame counter and clears all recorded data.
     */
    void clear() noexcept {
        for (auto& frame : frame_history_) {
            frame.clear();
        }
        current_frame_index_ = 0;
        frame_counter_ = 0;
        frame_start_time_us_ = 0;
    }

private:
    std::vector<FrameProfile> frame_history_;  ///< Ring buffer of frame profiles
    size_t history_size_;                      ///< Size of ring buffer
    size_t current_frame_index_;               ///< Current write position in ring buffer
    uint64_t frame_counter_;                   ///< Total frames processed
    uint64_t frame_start_time_us_;             ///< Start time of current frame
};

} // namespace phynity::diagnostics

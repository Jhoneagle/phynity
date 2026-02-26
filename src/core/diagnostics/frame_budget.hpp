#pragma once

#include "frame_profiler.hpp"
#include <functional>
#include <unordered_map>
#include <string>
#include <cstdint>

namespace phynity::diagnostics {

/**
 * @brief Budget violation information.
 */
struct BudgetViolation {
    std::string_view zone_name;     ///< Name of zone that exceeded budget (empty = total frame)
    uint64_t budget_us;              ///< Allocated budget in microseconds
    uint64_t actual_us;              ///< Actual time taken in microseconds
    uint64_t frame_number;           ///< Frame number where violation occurred
    
    /**
     * @brief Get overage amount.
     */
    [[nodiscard]] uint64_t overage_us() const noexcept {
        return actual_us > budget_us ? actual_us - budget_us : 0;
    }
    
    /**
     * @brief Get overage percentage.
     */
    [[nodiscard]] double overage_percent() const noexcept {
        return budget_us > 0 ? (static_cast<double>(overage_us()) / static_cast<double>(budget_us)) * 100.0 : 0.0;
    }
};

/**
 * @brief Callback type for budget violations.
 * 
 * Called when a zone or frame exceeds its allocated time budget.
 */
using BudgetViolationCallback = std::function<void(const BudgetViolation&)>;

/**
 * @brief Frame budget monitor for performance targets.
 * 
 * Tracks time budgets for the total frame and individual zones.
 * Triggers callbacks when budgets are exceeded.
 * 
 * Common frame budgets:
 * - 60 FPS: 16.67ms (16667 microseconds)
 * - 30 FPS: 33.33ms (33333 microseconds)
 * - 144 FPS: 6.94ms (6944 microseconds)
 * 
 * Usage:
 * ```cpp
 * FrameBudget budget;
 * budget.set_target_fps(60);  // 16.67ms per frame
 * budget.set_zone_budget("physics", 5000);  // 5ms for physics
 * budget.set_zone_budget("rendering", 8000);  // 8ms for rendering
 * 
 * budget.set_violation_callback([](const BudgetViolation& v) {
 *     printf("Budget exceeded: %s (%.2f%% over)\n",
 *            v.zone_name.empty() ? "FRAME" : v.zone_name.data(),
 *            v.overage_percent());
 * });
 * 
 * // In frame loop
 * budget.check_frame(profiler.get_last_frame());
 * ```
 */
class FrameBudget {
public:
    FrameBudget() noexcept
        : target_frame_time_us_(16667)  // Default: 60 FPS
        , enabled_(true)
    {}
    
    /**
     * @brief Set target frame time directly in microseconds.
     * 
     * @param time_us Target frame time in microseconds
     */
    void set_target_frame_time(uint64_t time_us) noexcept {
        target_frame_time_us_ = time_us;
    }
    
    /**
     * @brief Set target frame rate and compute frame time budget.
     * 
     * @param fps Target frames per second (e.g., 60, 30, 144)
     * 
     * Calculates frame time as: 1000000 / fps microseconds
     */
    void set_target_fps(double fps) noexcept {
        if (fps > 0.0) {
            target_frame_time_us_ = static_cast<uint64_t>(1000000.0 / fps);
        }
    }
    
    /**
     * @brief Get current target frame time.
     * 
     * @return Target frame time in microseconds
     */
    [[nodiscard]] uint64_t get_target_frame_time() const noexcept {
        return target_frame_time_us_;
    }
    
    /**
     * @brief Get target frame rate.
     * 
     * @return Target FPS
     */
    [[nodiscard]] double get_target_fps() const noexcept {
        return target_frame_time_us_ > 0 ? 1000000.0 / static_cast<double>(target_frame_time_us_) : 0.0;
    }
    
    /**
     * @brief Set time budget for a specific zone.
     * 
     * @param zone_name Name of the zone
     * @param budget_us Allocated time in microseconds
     */
    void set_zone_budget(const std::string& zone_name, uint64_t budget_us) {
        zone_budgets_[zone_name] = budget_us;
    }
    
    /**
     * @brief Remove budget for a zone.
     * 
     * @param zone_name Name of zone to remove
     */
    void remove_zone_budget(const std::string& zone_name) {
        zone_budgets_.erase(zone_name);
    }
    
    /**
     * @brief Clear all zone budgets.
     */
    void clear_zone_budgets() noexcept {
        zone_budgets_.clear();
    }
    
    /**
     * @brief Get budget for a specific zone.
     * 
     * @param zone_name Name of zone
     * @return Budget in microseconds, or 0 if no budget set
     */
    [[nodiscard]] uint64_t get_zone_budget(const std::string& zone_name) const {
        auto it = zone_budgets_.find(zone_name);
        return it != zone_budgets_.end() ? it->second : 0;
    }
    
    /**
     * @brief Set callback for budget violations.
     * 
     * @param callback Function to call when budget is exceeded
     */
    void set_violation_callback(BudgetViolationCallback callback) {
        violation_callback_ = std::move(callback);
    }
    
    /**
     * @brief Enable or disable budget checking.
     * 
     * @param enabled True to enable, false to disable
     */
    void set_enabled(bool enabled) noexcept {
        enabled_ = enabled;
    }
    
    /**
     * @brief Check if budget monitoring is enabled.
     */
    [[nodiscard]] bool is_enabled() const noexcept {
        return enabled_;
    }
    
    /**
     * @brief Check a frame against budgets and trigger violations.
     * 
     * @param frame Frame to check
     * 
     * Checks total frame time and all zones with budgets.
     * Calls violation callback for each violation detected.
     */
    void check_frame(const FrameProfile& frame) {
        if (!enabled_) {
            return;
        }
        
        // Check total frame budget
        if (frame.total_frame_time_us > target_frame_time_us_) {
            trigger_violation({
                "",  // Empty name = total frame
                target_frame_time_us_,
                frame.total_frame_time_us,
                frame.frame_number
            });
        }
        
        // Check zone budgets
        for (const auto& [zone_name, budget] : zone_budgets_) {
            // Find zone in frame
            for (const auto& zone : frame.zones) {
                if (zone.name == zone_name && zone.duration_us > budget) {
                    trigger_violation({
                        zone.name,
                        budget,
                        zone.duration_us,
                        frame.frame_number
                    });
                    break;  // Only report first instance of this zone
                }
            }
        }
    }
    
    /**
     * @brief Check multiple frames for violations.
     * 
     * @param profiler Frame profiler to check recent frames from
     * @param num_frames Number of recent frames to check (0 = all in history)
     */
    void check_recent_frames(const FrameProfiler& profiler, size_t num_frames = 1) {
        if (!enabled_) {
            return;
        }
        
        if (num_frames == 0) {
            num_frames = profiler.get_history_size();
        }
        
        const size_t frames_to_check = std::min(num_frames, 
                                                 static_cast<size_t>(profiler.get_frame_count()));
        
        for (size_t i = 0; i < frames_to_check; ++i) {
            check_frame(profiler.get_frame(i));
        }
    }

private:
    uint64_t target_frame_time_us_;                         ///< Target frame time budget
    std::unordered_map<std::string, uint64_t> zone_budgets_; ///< Per-zone budgets
    BudgetViolationCallback violation_callback_;            ///< Callback for violations
    bool enabled_;                                           ///< Enable/disable flag
    
    /**
     * @brief Trigger a budget violation callback.
     */
    void trigger_violation(const BudgetViolation& violation) {
        if (violation_callback_) {
            violation_callback_(violation);
        }
    }
};

} // namespace phynity::diagnostics

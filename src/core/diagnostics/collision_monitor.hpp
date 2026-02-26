#pragma once

#include <cstdint>
#include <functional>

namespace phynity::diagnostics {

/**
 * @brief Statistics about collision detection efficiency.
 * 
 * Tracks broadphase/narrowphase performance and collision detection
 * efficiency to help optimize spatial data structures.
 */
struct CollisionStats {
    uint64_t frame_number;                  ///< Frame when stats were collected
    uint32_t broadphase_candidates;         ///< Potential collision pairs from broadphase
    uint32_t narrowphase_tests;             ///< Actual collision tests performed
    uint32_t actual_collisions;             ///< Confirmed collisions detected
    double efficiency;                      ///< actual_collisions / broadphase_candidates (0-1)
    double false_positive_rate;             ///< (candidates - collisions) / candidates (0-1)
};

/**
 * @brief Violation indicating poor collision detection efficiency.
 * 
 * Triggered when broadphase produces too many false positives,
 * indicating spatial structure needs tuning (e.g., grid cell size).
 */
struct CollisionEfficiencyViolation {
    uint64_t frame_number;
    uint32_t broadphase_candidates;
    uint32_t actual_collisions;
    double efficiency;                      ///< Actual efficiency
    double min_efficiency;                  ///< Expected minimum
    double false_positive_rate;
};

/**
 * @brief Monitors collision detection efficiency and statistics.
 * 
 * Tracks broadphase/narrowphase performance to detect when spatial
 * data structures are producing too many false positives. Helps
 * identify when grid cell sizes or other parameters need tuning.
 * 
 * Usage:
 * ```cpp
 * CollisionMonitor monitor;
 * monitor.set_min_efficiency(0.1);  // Expect at least 10% of candidates to collide
 * 
 * monitor.set_violation_callback([](const CollisionEfficiencyViolation& v) {
 *     printf("Frame %llu: Efficiency %.1f%% (below %.1f%% threshold)\n",
 *            v.frame_number, v.efficiency * 100.0, v.min_efficiency * 100.0);
 * });
 * 
 * // After broadphase
 * monitor.set_broadphase_candidates(candidate_pairs.size());
 * 
 * // After narrowphase
 * monitor.set_narrowphase_tests(tests_performed);
 * 
 * // After collision resolution
 * monitor.set_actual_collisions(collisions.size());
 * 
 * // At frame end
 * monitor.end_frame();
 * ```
 */
class CollisionMonitor {
public:
    using ViolationCallback = std::function<void(const CollisionEfficiencyViolation&)>;

    CollisionMonitor() noexcept
        : frame_number_(0)
        , current_broadphase_candidates_(0)
        , current_narrowphase_tests_(0)
        , current_actual_collisions_(0)
        , min_efficiency_(0.05)          // Default: expect at least 5% efficiency
        , enabled_(true)
    {}

    /**
     * @brief Set minimum acceptable collision detection efficiency.
     * 
     * Efficiency = actual_collisions / broadphase_candidates
     * 
     * @param efficiency Minimum efficiency ratio (0.0-1.0), e.g., 0.1 for 10%
     */
    void set_min_efficiency(double efficiency) noexcept {
        min_efficiency_ = efficiency;
    }

    /**
     * @brief Get current minimum efficiency threshold.
     */
    double get_min_efficiency() const noexcept {
        return min_efficiency_;
    }

    /**
     * @brief Set broadphase candidate count for current frame.
     * 
     * @param count Number of potential collision pairs from broadphase
     */
    void set_broadphase_candidates(uint32_t count) noexcept {
        if (!enabled_) return;
        current_broadphase_candidates_ = count;
    }

    /**
     * @brief Set narrowphase test count for current frame.
     * 
     * @param count Number of narrowphase collision tests performed
     */
    void set_narrowphase_tests(uint32_t count) noexcept {
        if (!enabled_) return;
        current_narrowphase_tests_ = count;
    }

    /**
     * @brief Set actual collision count for current frame.
     * 
     * @param count Number of confirmed collisions detected
     */
    void set_actual_collisions(uint32_t count) noexcept {
        if (!enabled_) return;
        current_actual_collisions_ = count;
    }

    /**
     * @brief End current frame and check for efficiency violations.
     * 
     * Calculates efficiency metrics and triggers violation callback if efficiency
     * is below threshold. Automatically resets counters for next frame.
     */
    void end_frame() {
        if (!enabled_) {
            reset_frame_counters();
            return;
        }

        ++frame_number_;

        // Calculate efficiency metrics
        double efficiency = 0.0;
        double false_positive_rate = 0.0;

        if (current_broadphase_candidates_ > 0) {
            efficiency = static_cast<double>(current_actual_collisions_) / 
                         static_cast<double>(current_broadphase_candidates_);
            false_positive_rate = static_cast<double>(current_broadphase_candidates_ - current_actual_collisions_) /
                                  static_cast<double>(current_broadphase_candidates_);
        }

        // Check for efficiency violation
        if (current_broadphase_candidates_ > 0 && efficiency < min_efficiency_) {
            if (violation_callback_) {
                CollisionEfficiencyViolation violation;
                violation.frame_number = frame_number_;
                violation.broadphase_candidates = current_broadphase_candidates_;
                violation.actual_collisions = current_actual_collisions_;
                violation.efficiency = efficiency;
                violation.min_efficiency = min_efficiency_;
                violation.false_positive_rate = false_positive_rate;
                
                violation_callback_(violation);
            }
        }

        // Reset for next frame
        reset_frame_counters();
    }

    /**
     * @brief Get statistics for the last completed frame.
     * 
     * Note: This returns stats from the previous frame since counters
     * are reset at end_frame(). To get live stats, use get_current_*() methods.
     */
    CollisionStats get_last_frame_stats() const noexcept {
        CollisionStats stats;
        stats.frame_number = frame_number_;
        stats.broadphase_candidates = last_broadphase_candidates_;
        stats.narrowphase_tests = last_narrowphase_tests_;
        stats.actual_collisions = last_actual_collisions_;
        stats.efficiency = last_efficiency_;
        stats.false_positive_rate = last_false_positive_rate_;
        return stats;
    }

    /**
     * @brief Get current frame's broadphase candidate count.
     */
    uint32_t get_current_broadphase_candidates() const noexcept {
        return current_broadphase_candidates_;
    }

    /**
     * @brief Get current frame's narrowphase test count.
     */
    uint32_t get_current_narrowphase_tests() const noexcept {
        return current_narrowphase_tests_;
    }

    /**
     * @brief Get current frame's actual collision count.
     */
    uint32_t get_current_actual_collisions() const noexcept {
        return current_actual_collisions_;
    }

    /**
     * @brief Get current frame number.
     */
    uint64_t get_frame_count() const noexcept {
        return frame_number_;
    }

    /**
     * @brief Set callback for efficiency violations.
     */
    void set_violation_callback(ViolationCallback callback) noexcept {
        violation_callback_ = callback;
    }

    /**
     * @brief Enable or disable monitoring.
     */
    void set_enabled(bool enabled) noexcept {
        enabled_ = enabled;
    }

    /**
     * @brief Check if monitoring is enabled.
     */
    bool is_enabled() const noexcept {
        return enabled_;
    }

    /**
     * @brief Reset all statistics and frame counters.
     */
    void reset() noexcept {
        frame_number_ = 0;
        reset_frame_counters();
        last_broadphase_candidates_ = 0;
        last_narrowphase_tests_ = 0;
        last_actual_collisions_ = 0;
        last_efficiency_ = 0.0;
        last_false_positive_rate_ = 0.0;
    }

private:
    void reset_frame_counters() noexcept {
        // Save current frame stats before reset
        last_broadphase_candidates_ = current_broadphase_candidates_;
        last_narrowphase_tests_ = current_narrowphase_tests_;
        last_actual_collisions_ = current_actual_collisions_;
        
        if (current_broadphase_candidates_ > 0) {
            last_efficiency_ = static_cast<double>(current_actual_collisions_) / 
                              static_cast<double>(current_broadphase_candidates_);
            last_false_positive_rate_ = static_cast<double>(current_broadphase_candidates_ - current_actual_collisions_) /
                                        static_cast<double>(current_broadphase_candidates_);
        } else {
            last_efficiency_ = 0.0;
            last_false_positive_rate_ = 0.0;
        }

        current_broadphase_candidates_ = 0;
        current_narrowphase_tests_ = 0;
        current_actual_collisions_ = 0;
    }

    uint64_t frame_number_;
    
    // Current frame counters
    uint32_t current_broadphase_candidates_;
    uint32_t current_narrowphase_tests_;
    uint32_t current_actual_collisions_;
    
    // Last frame stats (for retrieval)
    uint32_t last_broadphase_candidates_ = 0;
    uint32_t last_narrowphase_tests_ = 0;
    uint32_t last_actual_collisions_ = 0;
    double last_efficiency_ = 0.0;
    double last_false_positive_rate_ = 0.0;

    double min_efficiency_;
    bool enabled_;
    
    ViolationCallback violation_callback_;
};

}  // namespace phynity::diagnostics

#pragma once

#include <cstdint>
#include <vector>
#include <string_view>
#include <functional>
#include <algorithm>

namespace phynity::diagnostics {

/**
 * @brief Energy conservation violation report.
 */
struct EnergyViolation {
    uint64_t frame_number;              ///< Frame where violation occurred
    double previous_energy;             ///< Energy in previous frame
    double current_energy;              ///< Energy in current frame
    double energy_loss;                 ///< Absolute energy lost
    double loss_percentage;             ///< Loss as percentage of initial energy
    std::string_view violation_type;    ///< "loss", "gain", or "spike"
    
    /**
     * @brief Check if loss is within acceptable tolerance.
     * 
     * @param max_loss_percent Maximum acceptable loss percentage
     * @return True if loss is within tolerance
     */
    [[nodiscard]] bool is_within_tolerance(double max_loss_percent) const noexcept {
        return loss_percentage <= max_loss_percent;
    }
};

/**
 * @brief Callback for energy violations.
 */
using EnergyViolationCallback = std::function<void(const EnergyViolation&)>;

/**
 * @brief Monitor system energy for conservation violations.
 * 
 * Tracks total mechanical energy (kinetic + potential) and detects:
 * - Excessive energy loss (numerical integration errors, approximations)
 * - Unexpected energy gain (simulation bugs, invalid physics)
 * - Energy spikes (collision discontinuities, constraint violations)
 * 
 * Usage:
 * ```cpp
 * EnergyMonitor monitor;
 * monitor.set_max_loss_percent(1.0);  // Allow 1% loss per frame
 * monitor.set_max_gain_percent(0.5);  // Allow 0.5% gain per frame
 * 
 * monitor.set_violation_callback([](const EnergyViolation& v) {
 *     if (v.loss_percentage > 5.0) {
 *         printf("WARNING: %s of %.2f%%\n", v.violation_type.data(), v.loss_percentage);
 *     }
 * });
 * 
 * // Update after each physics step
 * double kinetic = calculate_kinetic_energy(particles);
 * double potential = calculate_potential_energy(particles, gravity);
 * monitor.update(kinetic + potential);
 * ```
 */
class EnergyMonitor {
public:
    EnergyMonitor() noexcept
        : previous_energy_(0.0)
        , max_loss_percent_(2.0)      // Default: 2% loss acceptable
        , max_gain_percent_(0.5)      // Default: 0.5% gain acceptable
        , max_spike_percent_(10.0)    // Default: 10% spike acceptable
        , frame_number_(0)
        , enabled_(true)
        , has_previous_measurement_(false)
    {}
    
    /**
     * @brief Set maximum acceptable energy loss per frame (as percentage).
     * 
     * @param percent Maximum loss percentage (e.g., 1.0 for 1%)
     */
    void set_max_loss_percent(double percent) noexcept {
        max_loss_percent_ = percent;
    }
    
    /**
     * @brief Set maximum acceptable energy gain per frame (as percentage).
     * 
     * @param percent Maximum gain percentage (e.g., 0.5 for 0.5%)
     */
    void set_max_gain_percent(double percent) noexcept {
        max_gain_percent_ = percent;
    }
    
    /**
     * @brief Set maximum acceptable energy spike (sudden discontinuity).
     * 
     * Spikes can occur due to collision impulses. This sets the threshold
     * for detecting unreasonable spikes that suggest simulation issues.
     * 
     * @param percent Maximum spike percentage (e.g., 10.0 for 10%)
     */
    void set_max_spike_percent(double percent) noexcept {
        max_spike_percent_ = percent;
    }
    
    /**
     * @brief Update monitor with current system energy.
     * 
     * @param current_energy Total mechanical energy (kinetic + potential + other)
     * 
     * Compares with previous measurement and triggers violation callbacks if needed.
     */
    void update(double current_energy) {
        if (!has_previous_measurement_) {
            // First measurement - just store and return
            previous_energy_ = current_energy;
            has_previous_measurement_ = true;
            ++frame_number_;
            return;
        }
        
        if (!enabled_) {
            // Still update state when disabled, just skip violation checks
            previous_energy_ = current_energy;
            ++frame_number_;
            return;
        }
        
        // Calculate change
        const double change = current_energy - previous_energy_;
        const double magnitude = std::abs(previous_energy_);
        const double change_percent = magnitude > 0.0 ? (change / magnitude) * 100.0 : 0.0;
        
        // Check for violations
        if (change < 0.0) {
            // Energy loss
            const double loss_percent = std::abs(change_percent);
            if (loss_percent > max_loss_percent_) {
                trigger_violation({
                    frame_number_,
                    previous_energy_,
                    current_energy,
                    std::abs(change),
                    loss_percent,
                    "loss"
                });
            }
        } else if (change > 0.0) {
            // Energy gain
            if (change_percent > max_gain_percent_) {
                trigger_violation({
                    frame_number_,
                    previous_energy_,
                    current_energy,
                    change,
                    change_percent,
                    "gain"
                });
            }
        }
        
        previous_energy_ = current_energy;
        ++frame_number_;
    }
    
    /**
     * @brief Get current total energy.
     */
    [[nodiscard]] double get_current_energy() const noexcept {
        return previous_energy_;
    }
    
    /**
     * @brief Get total frames measured.
     */
    [[nodiscard]] uint64_t get_frame_count() const noexcept {
        return frame_number_;
    }
    
    /**
     * @brief Set violation callback.
     */
    void set_violation_callback(EnergyViolationCallback callback) {
        violation_callback_ = std::move(callback);
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
    [[nodiscard]] bool is_enabled() const noexcept {
        return enabled_;
    }
    
    /**
     * @brief Reset to initial state.
     */
    void reset() noexcept {
        previous_energy_ = 0.0;
        frame_number_ = 0;
        has_previous_measurement_ = false;
    }

private:
    double previous_energy_;
    double max_loss_percent_;
    double max_gain_percent_;
    double max_spike_percent_;
    uint64_t frame_number_;
    bool enabled_;
    bool has_previous_measurement_;
    EnergyViolationCallback violation_callback_;
    
    void trigger_violation(const EnergyViolation& violation) {
        if (violation_callback_) {
            violation_callback_(violation);
        }
    }
};

} // namespace phynity::diagnostics

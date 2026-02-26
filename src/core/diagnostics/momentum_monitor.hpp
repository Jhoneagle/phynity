#pragma once

#include <cstdint>
#include <vector>
#include <string_view>
#include <functional>
#include <array>
#include <cmath>

namespace phynity::diagnostics {

/**
 * @brief 3D vector for momentum representation.
 */
struct Vec3 {
    double x, y, z;
    
    Vec3() noexcept : x(0.0), y(0.0), z(0.0) {}
    Vec3(double x, double y, double z) noexcept : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& other) const noexcept {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    
    Vec3 operator-(const Vec3& other) const noexcept {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    
    [[nodiscard]] double magnitude() const noexcept {
        return std::sqrt(x*x + y*y + z*z);
    }
};

/**
 * @brief Momentum conservation violation report.
 */
struct MomentumViolation {
    uint64_t frame_number;              ///< Frame where violation occurred
    Vec3 previous_momentum;             ///< Momentum in previous frame
    Vec3 current_momentum;              ///< Momentum in current frame
    Vec3 momentum_change;               ///< Change in momentum
    double change_magnitude;            ///< Magnitude of change vector
    std::string_view cause;             ///< "external_force" or "numerical"
    
    /**
     * @brief Check if change is within tolerance.
     */
    [[nodiscard]] bool is_within_tolerance(double max_change_magnitude) const noexcept {
        return change_magnitude <= max_change_magnitude;
    }
};

/**
 * @brief Callback for momentum violations.
 */
using MomentumViolationCallback = std::function<void(const MomentumViolation&)>;

/**
 * @brief Monitor system momentum for conservation violations.
 * 
 * Tracks total linear momentum and detects:
 * - Unexpected momentum changes (missing forces, unaccounted impulses)
 * - Numerical drift (integration errors)
 * - External forces (should be explicitly accounted for)
 * 
 * Note: In a closed system with only internal forces, momentum should be conserved.
 * External forces (gravity, wind, etc.) cause expected changes.
 * 
 * Usage:
 * ```cpp
 * MomentumMonitor monitor;
 * monitor.set_max_change_magnitude(10.0);  // Max allowed change
 * 
 * monitor.set_violation_callback([](const MomentumViolation& v) {
 *     printf("Momentum changed by %.2f\n", v.change_magnitude);
 * });
 * 
 * // Update after each physics step
 * Vec3 total_momentum = calculate_total_momentum(particles);
 * monitor.update(total_momentum);
 * ```
 */
class MomentumMonitor {
public:
    MomentumMonitor() noexcept
        : previous_momentum_({0.0, 0.0, 0.0})
        , max_change_magnitude_(100.0)  // Default: max 100 units change
        , frame_number_(0)
        , enabled_(true)
        , has_previous_measurement_(false)
    {}
    
    /**
     * @brief Set maximum allowed momentum change per frame.
     * 
     * @param magnitude Maximum change magnitude (considering external forces like gravity)
     */
    void set_max_change_magnitude(double magnitude) noexcept {
        max_change_magnitude_ = std::abs(magnitude);
    }
    
    /**
     * @brief Update monitor with current system momentum.
     * 
     * @param current_momentum Total linear momentum of system
     */
    void update(const Vec3& current_momentum) {
        if (!has_previous_measurement_) {
            // First measurement - just store and return
            previous_momentum_ = current_momentum;
            has_previous_measurement_ = true;
            ++frame_number_;
            return;
        }
        
        if (!enabled_) {
            // Still update state when disabled, just skip violation checks
            previous_momentum_ = current_momentum;
            ++frame_number_;
            return;
        }
        
        // Calculate change
        const Vec3 change = current_momentum - previous_momentum_;
        const double change_mag = change.magnitude();
        
        // Check for violations
        if (change_mag > max_change_magnitude_) {
            trigger_violation({
                frame_number_,
                previous_momentum_,
                current_momentum,
                change,
                change_mag,
                "numerical"
            });
        }
        
        previous_momentum_ = current_momentum;
        ++frame_number_;
    }
    
    /**
     * @brief Get current momentum.
     */
    [[nodiscard]] Vec3 get_current_momentum() const noexcept {
        return previous_momentum_;
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
    void set_violation_callback(MomentumViolationCallback callback) {
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
        previous_momentum_ = Vec3{0.0, 0.0, 0.0};
        frame_number_ = 0;
        has_previous_measurement_ = false;
    }

private:
    Vec3 previous_momentum_;
    double max_change_magnitude_;
    uint64_t frame_number_;
    bool enabled_;
    bool has_previous_measurement_;
    MomentumViolationCallback violation_callback_;
    
    void trigger_violation(const MomentumViolation& violation) {
        if (violation_callback_) {
            violation_callback_(violation);
        }
    }
};

} // namespace phynity::diagnostics

#pragma once

#include <cmath>
#include <type_traits>
#include "constants.hpp"

namespace phynity::math::utilities {

    // ================================================================
    // Floating-point validity checks
    // ================================================================

    /**
     * Check if a value is NaN (Not a Number).
     */
    template <typename T>
    inline bool is_nan(T value) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::isnan(value);
    }

    /**
     * Check if a value is infinite.
     */
    template <typename T>
    inline bool is_inf(T value) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::isinf(value);
    }

    /**
     * Check if a value is finite (not NaN and not infinite).
     */
    template <typename T>
    inline bool is_finite(T value) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::isfinite(value);
    }

    /**
     * Check if a value is normal (not zero, subnormal, infinite, or NaN).
     */
    template <typename T>
    inline bool is_normal(T value) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::isnormal(value) || value == T(0);
    }

    // ================================================================
    // Floating-point comparison with tolerance
    // ================================================================

    /**
     * Check if two values are approximately equal within absolute tolerance.
     */
    template <typename T>
    inline bool equals_absolute(T a, T b, T tolerance = epsilon<T>()) noexcept {
        static_assert(std::is_floating_point_v<T>);
        if (!is_finite(a) || !is_finite(b)) {
            return a == b;
        }
        const T diff = a > b ? a - b : b - a;
        return diff <= tolerance;
    }

    /**
     * Check if two values are approximately equal within relative tolerance.
     * Useful for comparing values of different magnitudes.
     * 
     * Formula: |a - b| <= tolerance * max(|a|, |b|)
     * 
     * This checks if the difference between a and b is within a percentage
     * of their magnitudes. For example, with tolerance=1e-4:
     *   - Values differing by 0.01% are considered equal
     *   - 1000.1 ≈ 1000.0 (0.01% difference)
     *   - 1.0001 ≈ 1.0 (0.01% difference)
     * 
     * @param a First value
     * @param b Second value
     * @param tolerance Relative tolerance (default: 10 * epsilon)
     * @return true if |a - b| <= tolerance * max(|a|, |b|)
     */
    template <typename T>
    inline bool equals_relative(T a, T b, T tolerance = epsilon<T>() * T(10)) noexcept {
        static_assert(std::is_floating_point_v<T>);
        if (!is_finite(a) || !is_finite(b)) {
            return a == b;
        }
        const T abs_a = a < T(0) ? -a : a;
        const T abs_b = b < T(0) ? -b : b;
        const T max_abs = abs_a > abs_b ? abs_a : abs_b;
        const T diff = a > b ? a - b : b - a;
        return diff <= tolerance * max_abs;
    }

    /**
     * Check if two values are approximately equal using combined absolute and relative tolerance.
     * Robust for both small and large values.
     */
    template <typename T>
    inline bool equals(T a, T b, T abs_tolerance = epsilon<T>(), 
                       T rel_tolerance = epsilon<T>() * T(10)) noexcept {
        static_assert(std::is_floating_point_v<T>);
        if (!is_finite(a) || !is_finite(b)) {
            return a == b;
        }
        const T diff = a > b ? a - b : b - a;
        
        // Absolute tolerance check
        if (diff <= abs_tolerance) {
            return true;
        }
        
        // Relative tolerance check
        const T abs_a = a < T(0) ? -a : a;
        const T abs_b = b < T(0) ? -b : b;
        const T max_abs = abs_a > abs_b ? abs_a : abs_b;
        return diff <= rel_tolerance * max_abs;
    }

    /**
     * Check if a value is close to zero within tolerance.
     */
    template <typename T>
    inline bool is_zero(T value, T tolerance = epsilon<T>()) noexcept {
        static_assert(std::is_floating_point_v<T>);
        const T abs_val = value < T(0) ? -value : value;
        return abs_val <= tolerance;
    }

    // ================================================================
    // Clamping to valid range
    // ================================================================

    /**
     * Clamp a value to a safe range (not NaN or infinite).
     */
    template <typename T>
    inline T clamp_to_safe_range(T value, T max_magnitude = infinity<T>()) noexcept {
        static_assert(std::is_floating_point_v<T>);
        if (!is_finite(value)) {
            return T(0);
        }
        const T abs_val = value < T(0) ? -value : value;
        if (abs_val > max_magnitude) {
            return value < T(0) ? -max_magnitude : max_magnitude;
        }
        return value;
    }

} // namespace phynity::math::utilities

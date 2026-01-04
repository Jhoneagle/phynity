#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace phynity::math::utilities {

    // ================================================================
    // Clamping
    // ================================================================

    /**
     * Clamp a value between min and max.
     */
    template <typename T>
    inline constexpr T clamp(T value, T min_val, T max_val) noexcept {
        return std::max(min_val, std::min(value, max_val));
    }

    // ================================================================
    // Linear Interpolation
    // ================================================================

    /**
     * Linear interpolation between two values.
     * t = 0 returns a, t = 1 returns b.
     */
    template <typename T>
    inline constexpr T lerp(T a, T b, T t) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return a + t * (b - a);
    }

    /**
     * Inverse linear interpolation: find t such that lerp(a, b, t) = c.
     * If a == b, returns 0.
     */
    template <typename T>
    inline T inverse_lerp(T a, T b, T c) noexcept {
        static_assert(std::is_floating_point_v<T>);
        if (a == b) return T(0);
        return (c - a) / (b - a);
    }

    // ================================================================
    // Smoothstep and Smootherstep
    // ================================================================

    /**
     * Smoothstep interpolation: smooth Hermite interpolation between 0 and 1.
     * Returns 0 if x <= min, 1 if x >= max, and smooth cubic interpolation in between.
     */
    template <typename T>
    inline constexpr T smoothstep(T min_val, T max_val, T x) noexcept {
        static_assert(std::is_floating_point_v<T>);
        const T t = clamp((x - min_val) / (max_val - min_val), T(0), T(1));
        return t * t * (T(3) - T(2) * t);
    }

    /**
     * Smootherstep interpolation: even smoother than smoothstep.
     * Uses 5th-order polynomial instead of 3rd-order.
     */
    template <typename T>
    inline constexpr T smootherstep(T min_val, T max_val, T x) noexcept {
        static_assert(std::is_floating_point_v<T>);
        const T t = clamp((x - min_val) / (max_val - min_val), T(0), T(1));
        return t * t * t * (t * (t * T(6) - T(15)) + T(10));
    }

    // ================================================================
    // Min/Max with multiple arguments
    // ================================================================

    /**
     * Return the minimum of two values.
     */
    template <typename T>
    inline constexpr T min(T a, T b) noexcept {
        return a < b ? a : b;
    }

    /**
     * Return the maximum of two values.
     */
    template <typename T>
    inline constexpr T max(T a, T b) noexcept {
        return a > b ? a : b;
    }

    /**
     * Return the absolute value.
     */
    template <typename T>
    inline constexpr T abs(T value) noexcept {
        return value < T(0) ? -value : value;
    }

    /**
     * Return the sign of a value: -1, 0, or 1.
     */
    template <typename T>
    inline constexpr T sign(T value) noexcept {
        if (value > T(0)) return T(1);
        if (value < T(0)) return T(-1);
        return T(0);
    }

    // ================================================================
    // Rounding and fractional parts
    // ================================================================

    /**
     * Get the fractional part of a value.
     */
    template <typename T>
    inline T fract(T value) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return value - std::floor(value);
    }

    /**
     * Repeat a value in a range [0, max).
     */
    template <typename T>
    inline T repeat(T value, T max_val) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return value - std::floor(value / max_val) * max_val;
    }

    /**
     * Ping-pong: oscillate between 0 and max_val.
     */
    template <typename T>
    inline T ping_pong(T value, T max_val) noexcept {
        static_assert(std::is_floating_point_v<T>);
        const T repeated = repeat(value, max_val * T(2));
        return max_val - abs(repeated - max_val);
    }

} // namespace phynity::math::utilities

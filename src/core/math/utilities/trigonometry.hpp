#pragma once

#include <cmath>
#include <type_traits>
#include "constants.hpp"

namespace phynity::math::utilities {

    // ================================================================
    // Trigonometric wrappers with angle normalization
    // ================================================================

    /**
     * Normalize an angle to the range [-π, π].
     */
    template <typename T>
    inline T normalize_angle(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        constexpr T pi = math<T>::pi;
        constexpr T two_pi = math<T>::two_pi;

        // Reduce to [-2π, 2π]
        while (radians > pi) {
            radians -= two_pi;
        }
        while (radians < -pi) {
            radians += two_pi;
        }
        return radians;
    }

    /**
     * Compute sin of an angle (in radians) with normalization.
     */
    template <typename T>
    inline T sin_normalized(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::sin(normalize_angle(radians));
    }

    /**
     * Compute cos of an angle (in radians) with normalization.
     */
    template <typename T>
    inline T cos_normalized(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::cos(normalize_angle(radians));
    }

    /**
     * Compute tan of an angle (in radians) with normalization.
     */
    template <typename T>
    inline T tan_normalized(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::tan(normalize_angle(radians));
    }

    /**
     * Compute arcsin, returning result in radians.
     */
    template <typename T>
    inline T asin(T x) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::asin(x);
    }

    /**
     * Compute arccos, returning result in radians.
     */
    template <typename T>
    inline T acos(T x) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::acos(x);
    }

    /**
     * Compute arctan, returning result in radians.
     */
    template <typename T>
    inline T atan(T x) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::atan(x);
    }

    /**
     * Compute arctan2(y, x), returning result in radians.
     */
    template <typename T>
    inline T atan2(T y, T x) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::atan2(y, x);
    }

    // ================================================================
    // Small-angle approximations for numerical stability
    // ================================================================

    /**
     * Check if angle is small enough for approximation.
     * Threshold is typically around 0.1 radians (~5.7 degrees).
     */
    template <typename T>
    inline bool is_small_angle(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        constexpr T threshold = T(0.1);
        return radians >= -threshold && radians <= threshold;
    }

    /**
     * Small-angle approximation: sin(θ) ≈ θ
     * Accurate for |θ| < 0.1 radians.
     */
    template <typename T>
    inline T sin_small_angle(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        if (is_small_angle(radians)) {
            return radians;
        }
        return std::sin(radians);
    }

    /**
     * Small-angle approximation: cos(θ) ≈ 1 - θ²/2
     * Accurate for |θ| < 0.1 radians.
     */
    template <typename T>
    inline T cos_small_angle(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        if (is_small_angle(radians)) {
            const T r2 = radians * radians;
            return T(1) - r2 / T(2);
        }
        return std::cos(radians);
    }

    /**
     * Small-angle approximation: tan(θ) ≈ θ
     * Accurate for |θ| < 0.1 radians.
     */
    template <typename T>
    inline T tan_small_angle(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        if (is_small_angle(radians)) {
            return radians;
        }
        return std::tan(radians);
    }

} // namespace phynity::math::utilities

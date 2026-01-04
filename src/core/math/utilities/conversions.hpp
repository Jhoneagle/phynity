#pragma once

#include <type_traits>
#include "constants.hpp"

namespace phynity::math::utilities {

    // ================================================================
    // Degree/Radian Conversions
    // ================================================================

    /**
     * Convert degrees to radians.
     */
    template <typename T>
    inline constexpr T degrees_to_radians(T degrees) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return degrees * math<T>::deg_to_rad;
    }

    /**
     * Convert radians to degrees.
     */
    template <typename T>
    inline constexpr T radians_to_degrees(T radians) noexcept {
        static_assert(std::is_floating_point_v<T>);
        return radians * math<T>::rad_to_deg;
    }

    /**
     * Alias for degrees_to_radians.
     */
    template <typename T>
    inline constexpr T deg2rad(T degrees) noexcept {
        return degrees_to_radians(degrees);
    }

    /**
     * Alias for radians_to_degrees.
     */
    template <typename T>
    inline constexpr T rad2deg(T radians) noexcept {
        return radians_to_degrees(radians);
    }

} // namespace phynity::math::utilities

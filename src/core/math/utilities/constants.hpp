#pragma once

#include <limits>
#include <numbers>
#include <type_traits>

namespace phynity::math::utilities {

    // ================================================================
    // Numeric precision helpers
    // ================================================================

    template <typename T>
    constexpr T epsilon() noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::numeric_limits<T>::epsilon();
    }

    template <typename T>
    constexpr T infinity() noexcept {
        static_assert(std::is_floating_point_v<T>);
        return std::numeric_limits<T>::infinity();
    }

    // ================================================================
    // Mathematical constants
    // ================================================================

    template <typename T>
    struct math {
        static_assert(std::is_floating_point_v<T>);

        static constexpr T pi        = std::numbers::pi_v<T>;
        static constexpr T two_pi    = T(2) * pi;
        static constexpr T half_pi   = pi / T(2);

        static constexpr T e         = std::numbers::e_v<T>;
        static constexpr T sqrt2     = std::numbers::sqrt2_v<T>;
        static constexpr T inv_sqrt2 = T(1) / sqrt2;

        static constexpr T deg_to_rad = pi / T(180);
        static constexpr T rad_to_deg = T(180) / pi;

        static constexpr T zero = T(0);
        static constexpr T one  = T(1);
    };

    using mathf = math<float>;
    using mathd = math<double>;

    // ================================================================
    // Physics constants (SI units)
    // ================================================================

    template <typename T>
    struct physics_constants {
        static_assert(std::is_floating_point_v<T>);

        // Fundamental
        static constexpr T speed_of_light      = T(299792458);           // m/s
        static constexpr T gravitational_const = T(6.67430e-11);         // m^3 kg^-1 s^-2

        // Earth
        static constexpr T gravity_earth       = T(9.80665);             // m/s^2
        static constexpr T earth_radius        = T(6.371e6);             // meters

        // Thermodynamics
        static constexpr T boltzmann           = T(1.380649e-23);         // J/K
        static constexpr T avogadro            = T(6.02214076e23);        // 1/mol

        // Numerical tolerances
        static constexpr T linear_slop         = T(1e-5);                // meters
        static constexpr T angular_slop        = T(1e-5);                // radians
        static constexpr T sleep_epsilon       = T(1e-3);

        // Simulation
        static constexpr T max_linear_velocity = T(1e3);                 // m/s
        static constexpr T max_angular_velocity= T(1e3);                 // rad/s
    };

    using physicsf = physics_constants<float>;
    using physicsd = physics_constants<double>;

    // ================================================================
    // Quaternion / rotation tolerances
    // ================================================================

    template <typename T>
    struct rotation {
        static_assert(std::is_floating_point_v<T>);

        static constexpr T normalize_epsilon     = T(1e-6);
        static constexpr T orthonormal_epsilon   = T(1e-4);
        static constexpr T slerp_threshold       = T(0.9995);
        static constexpr T gimbal_lock_threshold = T(0.499);
    };

    using rotationf = rotation<float>;
    using rotationd = rotation<double>;

    // ================================================================
    // Collision / geometry tolerances
    // ================================================================

    template <typename T>
    struct geometry {
        static_assert(std::is_floating_point_v<T>);

        static constexpr T contact_epsilon  = T(1e-4);
        static constexpr T penetration_slop = T(1e-3);
        static constexpr T ray_epsilon      = T(1e-6);
    };

    using geometryf = geometry<float>;
    using geometryd = geometry<double>;

} // namespace physics

#pragma once

#include <cmath>
#include <type_traits>
#include "constants.hpp"

namespace phynity::math::utilities {

    // ================================================================
    // Vector utilities (basic types for geometry)
    // ================================================================

    template <typename T>
    struct Vec3 {
        static_assert(std::is_floating_point_v<T>);
        T x, y, z;
        
        constexpr Vec3() noexcept : x(0), y(0), z(0) {}
        constexpr Vec3(T x_, T y_, T z_) noexcept : x(x_), y(y_), z(z_) {}
        
        constexpr T length_squared() const noexcept {
            return x * x + y * y + z * z;
        }
        
        T length() const noexcept {
            return std::sqrt(length_squared());
        }
    };

    // ================================================================
    // Plane representation: Ax + By + Cz + D = 0
    // ================================================================

    template <typename T>
    struct Plane {
        static_assert(std::is_floating_point_v<T>);
        T a, b, c, d;  // Normal: (a, b, c), Distance: d
        
        constexpr Plane() noexcept : a(0), b(0), c(1), d(0) {}
        constexpr Plane(T a_, T b_, T c_, T d_) noexcept : a(a_), b(b_), c(c_), d(d_) {}
        
        /**
         * Get the normal vector of the plane.
         */
        constexpr Vec3<T> normal() const noexcept {
            return Vec3<T>(a, b, c);
        }
        
        /**
         * Calculate signed distance from point to plane.
         * Positive if point is on the side the normal points to.
         */
        constexpr T distance_to_point(const Vec3<T>& point) const noexcept {
            return a * point.x + b * point.y + c * point.z + d;
        }
    };

    // ================================================================
    // Ray-Plane Intersection
    // ================================================================

    /**
     * Ray-plane intersection test.
     * Ray: P(t) = origin + t * direction, t >= 0
     * Returns true if intersection exists and t is in [0, max_t]
     */
    template <typename T>
    inline bool ray_plane_intersection(
        const Vec3<T>& ray_origin,
        const Vec3<T>& ray_direction,
        const Plane<T>& plane,
        T& out_t,
        T max_t = infinity<T>()) noexcept {
        static_assert(std::is_floating_point_v<T>);

        // Compute denominator: normal · direction
        T denom = plane.a * ray_direction.x + 
                  plane.b * ray_direction.y + 
                  plane.c * ray_direction.z;

        // Ray is parallel to plane
        if (is_zero(denom, geometry<T>::ray_epsilon)) {
            return false;
        }

        // Compute t: -(normal · origin + d) / (normal · direction)
        T numerator = -(plane.a * ray_origin.x + 
                        plane.b * ray_origin.y + 
                        plane.c * ray_origin.z + 
                        plane.d);
        
        out_t = numerator / denom;

        // Check if intersection is in valid range
        return out_t >= -geometry<T>::ray_epsilon && out_t <= max_t;
    }

    // ================================================================
    // Sphere representation and tests
    // ================================================================

    template <typename T>
    struct Sphere {
        static_assert(std::is_floating_point_v<T>);
        Vec3<T> center;
        T radius;
        
        constexpr Sphere() noexcept : center(0, 0, 0), radius(T(1)) {}
        constexpr Sphere(T x, T y, T z, T r) noexcept : center(x, y, z), radius(r) {}
        explicit constexpr Sphere(const Vec3<T>& center_, T r) noexcept : center(center_), radius(r) {}
    };

    /**
     * Test if a point is inside a sphere.
     */
    template <typename T>
    inline bool point_in_sphere(const Vec3<T>& point, const Sphere<T>& sphere) noexcept {
        static_assert(std::is_floating_point_v<T>);
        
        T dx = point.x - sphere.center.x;
        T dy = point.y - sphere.center.y;
        T dz = point.z - sphere.center.z;
        T dist_sq = dx * dx + dy * dy + dz * dz;
        T radius_sq = sphere.radius * sphere.radius;
        
        return dist_sq <= radius_sq;
    }

    /**
     * Test if a point is on the surface of a sphere (within epsilon).
     */
    template <typename T>
    inline bool point_on_sphere(const Vec3<T>& point, const Sphere<T>& sphere,
                                T tolerance = geometry<T>::contact_epsilon) noexcept {
        static_assert(std::is_floating_point_v<T>);
        
        T dx = point.x - sphere.center.x;
        T dy = point.y - sphere.center.y;
        T dz = point.z - sphere.center.z;
        T dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        
        return equals(dist, sphere.radius, tolerance);
    }

    /**
     * Test if two spheres intersect or touch.
     */
    template <typename T>
    inline bool sphere_sphere_intersection(const Sphere<T>& s1, const Sphere<T>& s2) noexcept {
        static_assert(std::is_floating_point_v<T>);
        
        T dx = s1.center.x - s2.center.x;
        T dy = s1.center.y - s2.center.y;
        T dz = s1.center.z - s2.center.z;
        T dist_sq = dx * dx + dy * dy + dz * dz;
        
        T sum_radius = s1.radius + s2.radius;
        T sum_radius_sq = sum_radius * sum_radius;
        
        return dist_sq <= sum_radius_sq;
    }

    /**
     * Compute closest point on sphere surface to a given point.
     */
    template <typename T>
    inline Vec3<T> closest_point_on_sphere(const Vec3<T>& point, 
                                          const Sphere<T>& sphere) noexcept {
        static_assert(std::is_floating_point_v<T>);
        
        T dx = point.x - sphere.center.x;
        T dy = point.y - sphere.center.y;
        T dz = point.z - sphere.center.z;
        T dist_sq = dx * dx + dy * dy + dz * dz;
        
        // If point is at center, return arbitrary point on surface
        if (equals(dist_sq, T(0), geometry<T>::ray_epsilon)) {
            return Vec3<T>(sphere.center.x + sphere.radius, sphere.center.y, sphere.center.z);
        }
        
        T dist = std::sqrt(dist_sq);
        T scale = sphere.radius / dist;
        
        return Vec3<T>(
            sphere.center.x + dx * scale,
            sphere.center.y + dy * scale,
            sphere.center.z + dz * scale
        );
    }

    // ================================================================
    // AABB (Axis-Aligned Bounding Box) representation and tests
    // ================================================================

    template <typename T>
    struct AABB {
        static_assert(std::is_floating_point_v<T>);
        Vec3<T> min, max;
        
        constexpr AABB() noexcept : min(0, 0, 0), max(0, 0, 0) {}
        constexpr AABB(const Vec3<T>& min_, const Vec3<T>& max_) noexcept : min(min_), max(max_) {}
    };

    /**
     * Test if a point is inside an AABB.
     */
    template <typename T>
    inline bool point_in_aabb(const Vec3<T>& point, const AABB<T>& aabb) noexcept {
        return point.x >= aabb.min.x && point.x <= aabb.max.x &&
               point.y >= aabb.min.y && point.y <= aabb.max.y &&
               point.z >= aabb.min.z && point.z <= aabb.max.z;
    }

    /**
     * Test if two AABBs intersect.
     */
    template <typename T>
    inline bool aabb_aabb_intersection(const AABB<T>& a1, const AABB<T>& a2) noexcept {
        return a1.min.x <= a2.max.x && a1.max.x >= a2.min.x &&
               a1.min.y <= a2.max.y && a1.max.y >= a2.min.y &&
               a1.min.z <= a2.max.z && a1.max.z >= a2.min.z;
    }

    /**
     * Compute the closest point on an AABB to a given point.
     */
    template <typename T>
    inline Vec3<T> closest_point_on_aabb(const Vec3<T>& point, const AABB<T>& aabb) noexcept {
        return Vec3<T>(
            clamp(point.x, aabb.min.x, aabb.max.x),
            clamp(point.y, aabb.min.y, aabb.max.y),
            clamp(point.z, aabb.min.z, aabb.max.z)
        );
    }

} // namespace phynity::math::utilities

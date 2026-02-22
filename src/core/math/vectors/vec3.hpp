#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>
#include <type_traits>

namespace phynity::math::vectors {

/// Three-component vector template for any floating-point type.
template <typename T = float>
struct Vec3 {
    static_assert(std::is_floating_point_v<T>, "Vec3 requires a floating-point type");
    
    T x = T(0);
    T y = T(0);
    T z = T(0);

    // Constructors
    constexpr Vec3() = default;
    explicit constexpr Vec3(T v) : x(v), y(v), z(v) {}
    constexpr Vec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    // Operators
    constexpr Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    constexpr Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    constexpr Vec3 operator*(T scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    constexpr Vec3 operator/(T scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    constexpr Vec3 operator*(const Vec3& other) const {
        return Vec3(x * other.x, y * other.y, z * other.z);
    }

    constexpr Vec3 operator/(const Vec3& other) const {
        return Vec3(x / other.x, y / other.y, z / other.z);
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator*=(T scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator*=(const Vec3& other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator/=(T scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    Vec3& operator/=(const Vec3& other) {
        x /= other.x;
        y /= other.y;
        z /= other.z;
        return *this;
    }

    constexpr Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    constexpr bool operator==(const Vec3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    constexpr bool operator!=(const Vec3& other) const {
        return !(*this == other);
    }

    constexpr T& operator[](int i) {
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    constexpr const T& operator[](int i) const {
        return (i == 0) ? x : (i == 1) ? y : z;
    }
    /// Bounds-checked element access
    T& at(int i) {
        if (i < 0 || i >= 3) throw std::out_of_range("Vec3 index out of range");
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    const T& at(int i) const {
        if (i < 0 || i >= 3) throw std::out_of_range("Vec3 index out of range");
        return (i == 0) ? x : (i == 1) ? y : z;
    }
    T dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    T squaredLength() const {
        return dot(*this);
    }

    T length() const {
        return std::sqrt(squaredLength());
    }

    Vec3 normalized() const {
        T len = length();
        return len > T(0) ? *this / len : Vec3(T(0));
    }

    Vec3& normalize() {
        T len = length();
        if (len > T(0)) {
            *this /= len;
        }
        return *this;
    }

    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    T distance(const Vec3& other) const {
        return (*this - other).length();
    }

    T squaredDistance(const Vec3& other) const {
        return (*this - other).squaredLength();
    }

    T angle(const Vec3& other) const {
        T denom = length() * other.length();
        if (denom < T(1e-6)) return T(0);  // Handle zero-length vectors
        T cosTheta = dot(other) / denom;
        // Clamp to [-1, 1] to handle floating point errors
        cosTheta = cosTheta < T(-1) ? T(-1) : (cosTheta > T(1) ? T(1) : cosTheta);
        return std::acos(cosTheta);
    }

    Vec3 clamped(T maxLength) const {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    Vec3& clamp(T maxLength) {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    Vec3 lerp(const Vec3& other, T t) const {
        return *this + (other - *this) * t;
    }

    Vec3 project(const Vec3& onto) const {
        T ontoLenSq = onto.squaredLength();
        if (ontoLenSq < T(1e-6)) return Vec3(T(0));
        return onto * (dot(onto) / ontoLenSq);
    }

    Vec3 reflect(const Vec3& normal) const {
        return *this - normal * (T(2) * dot(normal));
    }

    Vec3 perpendicular() const {
        // Choose axis with smallest absolute component to avoid parallel vectors
        Vec3 axis = (std::abs(x) < std::abs(y) && std::abs(x) < std::abs(z)) 
                    ? Vec3(T(1), T(0), T(0))
                    : (std::abs(y) < std::abs(z) ? Vec3(T(0), T(1), T(0)) : Vec3(T(0), T(0), T(1)));
        return cross(axis).normalized();
    }

    Vec3 min(const Vec3& other) const {
        return Vec3(
            x < other.x ? x : other.x,
            y < other.y ? y : other.y,
            z < other.z ? z : other.z
        );
    }

    Vec3 max(const Vec3& other) const {
        return Vec3(
            x > other.x ? x : other.x,
            y > other.y ? y : other.y,
            z > other.z ? z : other.z
        );
    }

    bool isZero() const {
        return x == T(0) && y == T(0) && z == T(0);
    }

    bool isNormalized() const {
        T lenSq = squaredLength();
        return std::abs(lenSq - T(1)) < T(1e-5);
    }

    bool approxEqual(const Vec3& other, T epsilon = T(1e-5)) const {
        return std::abs(x - other.x) < epsilon && 
               std::abs(y - other.y) < epsilon && 
               std::abs(z - other.z) < epsilon;
    }

    Vec3 abs() const {
        return Vec3(std::abs(x), std::abs(y), std::abs(z));
    }

    // Static utility vectors
    static Vec3 zero() { return Vec3(T(0), T(0), T(0)); }
    static Vec3 one() { return Vec3(T(1), T(1), T(1)); }
    static Vec3 up() { return Vec3(T(0), T(1), T(0)); }
    static Vec3 down() { return Vec3(T(0), T(-1), T(0)); }
    static Vec3 right() { return Vec3(T(1), T(0), T(0)); }
    static Vec3 left() { return Vec3(T(-1), T(0), T(0)); }
    static Vec3 forward() { return Vec3(T(0), T(0), T(1)); }
    static Vec3 back() { return Vec3(T(0), T(0), T(-1)); }
};

template <typename T = float>
inline Vec3<T> operator*(T scalar, const Vec3<T>& v) {
    return v * scalar;
}

template <typename T = float>
inline std::ostream& operator<<(std::ostream& os, const Vec3<T>& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

// Type aliases for convenience
using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;

}  // namespace phynity::math::vectors

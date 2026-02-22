#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>
#include <type_traits>

namespace phynity::math::vectors {

/// Two-component vector template for any floating-point type.
template <typename T = float>
struct Vec2 {
    static_assert(std::is_floating_point_v<T>, "Vec2 requires a floating-point type");
    
    T x = T(0);
    T y = T(0);

    // Constructors
    constexpr Vec2() = default;
    explicit constexpr Vec2(T v) : x(v), y(v) {}
    constexpr Vec2(T x_, T y_) : x(x_), y(y_) {}

    // Operators
    constexpr Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    constexpr Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    constexpr Vec2 operator*(T scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    constexpr Vec2 operator/(T scalar) const {
        return Vec2(x / scalar, y / scalar);
    }

    constexpr Vec2 operator*(const Vec2& other) const {
        return Vec2(x * other.x, y * other.y);
    }

    constexpr Vec2 operator/(const Vec2& other) const {
        return Vec2(x / other.x, y / other.y);
    }

    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator*=(T scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vec2& operator*=(const Vec2& other) {
        x *= other.x;
        y *= other.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2& operator/=(T scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    Vec2& operator/=(const Vec2& other) {
        x /= other.x;
        y /= other.y;
        return *this;
    }

    constexpr Vec2 operator-() const {
        return Vec2(-x, -y);
    }

    constexpr bool operator==(const Vec2& other) const {
        return x == other.x && y == other.y;
    }

    constexpr bool operator!=(const Vec2& other) const {
        return !(*this == other);
    }

    T& operator[](int i) {
        return (i == 0) ? x : y;
    }

    const T& operator[](int i) const {
        return (i == 0) ? x : y;
    }

    T dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }

    T squaredLength() const {
        return dot(*this);
    }

    T length() const {
        return std::sqrt(squaredLength());
    }

    Vec2 normalized() const {
        T len = length();
        return len > T(0) ? *this / len : Vec2(T(0));
    }

    Vec2& normalize() {
        T len = length();
        if (len > T(0)) {
            *this /= len;
        }
        return *this;
    }

    T distance(const Vec2& other) const {
        return (*this - other).length();
    }

    T squaredDistance(const Vec2& other) const {
        return (*this - other).squaredLength();
    }

    T angle(const Vec2& other) const {
        T denom = length() * other.length();
        if (denom < T(1e-6)) return T(0);
        T cosTheta = dot(other) / denom;
        cosTheta = cosTheta < T(-1) ? T(-1) : (cosTheta > T(1) ? T(1) : cosTheta);
        return std::acos(cosTheta);
    }

    Vec2 clamped(T maxLength) const {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    Vec2& clamp(T maxLength) {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    Vec2 lerp(const Vec2& other, T t) const {
        return *this + (other - *this) * t;
    }

    Vec2 project(const Vec2& onto) const {
        T ontoLenSq = onto.squaredLength();
        if (ontoLenSq < T(1e-6)) return Vec2(T(0));
        return onto * (dot(onto) / ontoLenSq);
    }

    // 2D reflection (reflect across a line defined by normal)
    Vec2 reflect(const Vec2& normal) const {
        return *this - normal * (T(2) * dot(normal));
    }

    // 2D perpendicular (rotate 90 degrees)
    Vec2 perpendicular() const {
        return Vec2(-y, x);
    }

    Vec2 min(const Vec2& other) const {
        return Vec2(
            x < other.x ? x : other.x,
            y < other.y ? y : other.y
        );
    }

    Vec2 max(const Vec2& other) const {
        return Vec2(
            x > other.x ? x : other.x,
            y > other.y ? y : other.y
        );
    }

    bool isZero() const {
        return x == T(0) && y == T(0);
    }

    bool isNormalized() const {
        T lenSq = squaredLength();
        return std::abs(lenSq - T(1)) < T(1e-5);
    }

    bool approxEqual(const Vec2& other, T epsilon = T(1e-5)) const {
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
    }

    Vec2 abs() const {
        return Vec2(std::abs(x), std::abs(y));
    }

    // Static utility vectors
    static Vec2 zero() { return Vec2(T(0), T(0)); }
    static Vec2 one() { return Vec2(T(1), T(1)); }
    static Vec2 up() { return Vec2(T(0), T(1)); }
    static Vec2 down() { return Vec2(T(0), T(-1)); }
    static Vec2 right() { return Vec2(T(1), T(0)); }
    static Vec2 left() { return Vec2(T(-1), T(0)); }
};

template <typename T = float>
inline Vec2<T> operator*(T scalar, const Vec2<T>& v) {
    return v * scalar;
}

template <typename T = float>
inline std::ostream& operator<<(std::ostream& os, const Vec2<T>& v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

// Type aliases for convenience
using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;

}  // namespace phynity::math::vectors

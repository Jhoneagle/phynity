#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>
#include <type_traits>

namespace phynity::math::vectors {

/// Four-component vector template for any floating-point type.
template <typename T = float>
struct Vec4 {
    static_assert(std::is_floating_point_v<T>, "Vec4 requires a floating-point type");
    
    T x = T(0);
    T y = T(0);
    T z = T(0);
    T w = T(0);

    // Constructors
    Vec4() = default;
    explicit Vec4(T v) : x(v), y(v), z(v), w(v) {}
    Vec4(T x_, T y_, T z_, T w_) : x(x_), y(y_), z(z_), w(w_) {}

    // Operators
    Vec4 operator+(const Vec4& other) const {
        return Vec4(x + other.x, y + other.y, z + other.z, w + other.w);
    }

    Vec4 operator-(const Vec4& other) const {
        return Vec4(x - other.x, y - other.y, z - other.z, w - other.w);
    }

    Vec4 operator*(T scalar) const {
        return Vec4(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    Vec4 operator/(T scalar) const {
        return Vec4(x / scalar, y / scalar, z / scalar, w / scalar);
    }

    Vec4 operator*(const Vec4& other) const {
        return Vec4(x * other.x, y * other.y, z * other.z, w * other.w);
    }

    Vec4 operator/(const Vec4& other) const {
        return Vec4(x / other.x, y / other.y, z / other.z, w / other.w);
    }

    Vec4& operator+=(const Vec4& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }

    Vec4& operator*=(T scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        w *= scalar;
        return *this;
    }

    Vec4& operator*=(const Vec4& other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        w *= other.w;
        return *this;
    }

    Vec4& operator-=(const Vec4& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        w -= other.w;
        return *this;
    }

    Vec4& operator/=(T scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        w /= scalar;
        return *this;
    }

    Vec4& operator/=(const Vec4& other) {
        x /= other.x;
        y /= other.y;
        z /= other.z;
        w /= other.w;
        return *this;
    }

    Vec4 operator-() const {
        return Vec4(-x, -y, -z, -w);
    }

    bool operator==(const Vec4& other) const {
        return x == other.x && y == other.y && z == other.z && w == other.w;
    }

    bool operator!=(const Vec4& other) const {
        return !(*this == other);
    }

    T& operator[](int i) {
        return (i == 0) ? x : (i == 1) ? y : (i == 2) ? z : w;
    }

    const T& operator[](int i) const {
        return (i == 0) ? x : (i == 1) ? y : (i == 2) ? z : w;
    }

    T dot(const Vec4& other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    T squaredLength() const {
        return dot(*this);
    }

    T length() const {
        return std::sqrt(squaredLength());
    }

    Vec4 normalized() const {
        T len = length();
        return len > T(0) ? *this / len : Vec4(T(0));
    }

    Vec4& normalize() {
        T len = length();
        if (len > T(0)) {
            *this /= len;
        }
        return *this;
    }

    T distance(const Vec4& other) const {
        return (*this - other).length();
    }

    T squaredDistance(const Vec4& other) const {
        return (*this - other).squaredLength();
    }

    T angle(const Vec4& other) const {
        T denom = length() * other.length();
        if (denom < T(1e-6)) return T(0);
        T cosTheta = dot(other) / denom;
        cosTheta = cosTheta < T(-1) ? T(-1) : (cosTheta > T(1) ? T(1) : cosTheta);
        return std::acos(cosTheta);
    }

    Vec4 clamped(T maxLength) const {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    Vec4& clamp(T maxLength) {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    Vec4 lerp(const Vec4& other, T t) const {
        return *this + (other - *this) * t;
    }

    Vec4 project(const Vec4& onto) const {
        float ontoLenSq = onto.squaredLength();
        if (ontoLenSq < 1e-6f) return Vec4(0.0f);
        return onto * (dot(onto) / ontoLenSq);
    }

    Vec4 reflect(const Vec4& normal) const {
        return *this - normal * (2.0f * dot(normal));
    }

    Vec4 min(const Vec4& other) const {
        return Vec4(
            x < other.x ? x : other.x,
            y < other.y ? y : other.y,
            z < other.z ? z : other.z,
            w < other.w ? w : other.w
        );
    }

    Vec4 max(const Vec4& other) const {
        return Vec4(
            x > other.x ? x : other.x,
            y > other.y ? y : other.y,
            z > other.z ? z : other.z,
            w > other.w ? w : other.w
        );
    }

    bool isZero() const {
        return x == T(0) && y == T(0) && z == T(0) && w == T(0);
    }

    bool isNormalized() const {
        T lenSq = squaredLength();
        return std::abs(lenSq - T(1)) < T(1e-5);
    }

    bool approxEqual(const Vec4& other, T epsilon = T(1e-5)) const {
        return std::abs(x - other.x) < epsilon && 
               std::abs(y - other.y) < epsilon && 
               std::abs(z - other.z) < epsilon && 
               std::abs(w - other.w) < epsilon;
    }

    Vec4 abs() const {
        return Vec4(std::abs(x), std::abs(y), std::abs(z), std::abs(w));
    }

    // Static utility vectors
    static Vec4 zero() { return Vec4(T(0), T(0), T(0), T(0)); }
    static Vec4 one() { return Vec4(T(1), T(1), T(1), T(1)); }
    static Vec4 unitX() { return Vec4(T(1), T(0), T(0), T(0)); }
    static Vec4 unitY() { return Vec4(T(0), T(1), T(0), T(0)); }
    static Vec4 unitZ() { return Vec4(T(0), T(0), T(1), T(0)); }
    static Vec4 unitW() { return Vec4(T(0), T(0), T(0), T(1)); }
};

template <typename T = float>
inline Vec4<T> operator*(T scalar, const Vec4<T>& v) {
    return v * scalar;
}

template <typename T = float>
inline std::ostream& operator<<(std::ostream& os, const Vec4<T>& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
    return os;
}

// Type aliases for convenience
using Vec4f = Vec4<float>;
using Vec4d = Vec4<double>;

}  // namespace phynity::math::vectors

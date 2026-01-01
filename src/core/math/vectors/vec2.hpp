#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>

namespace phynity::math::vectors {

/// Two-component floating-point vector.
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    // Constructors
    Vec2() = default;
    explicit Vec2(float v) : x(v), y(v) {}
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    // Operators
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    Vec2 operator/(float scalar) const {
        return Vec2(x / scalar, y / scalar);
    }

    Vec2 operator*(const Vec2& other) const {
        return Vec2(x * other.x, y * other.y);
    }

    Vec2 operator/(const Vec2& other) const {
        return Vec2(x / other.x, y / other.y);
    }

    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator*=(float scalar) {
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

    Vec2& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    Vec2& operator/=(const Vec2& other) {
        x /= other.x;
        y /= other.y;
        return *this;
    }

    Vec2 operator-() const {
        return Vec2(-x, -y);
    }

    bool operator==(const Vec2& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vec2& other) const {
        return !(*this == other);
    }

    float& operator[](int i) {
        return (i == 0) ? x : y;
    }

    const float& operator[](int i) const {
        return (i == 0) ? x : y;
    }

    float dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }

    float squaredLength() const {
        return dot(*this);
    }

    float length() const {
        return std::sqrt(squaredLength());
    }

    Vec2 normalized() const {
        float len = length();
        return len > 0.0f ? *this / len : Vec2(0.0f);
    }

    Vec2& normalize() {
        float len = length();
        if (len > 0.0f) {
            *this /= len;
        }
        return *this;
    }

    float distance(const Vec2& other) const {
        return (*this - other).length();
    }

    float squaredDistance(const Vec2& other) const {
        return (*this - other).squaredLength();
    }

    float angle(const Vec2& other) const {
        float denom = length() * other.length();
        if (denom < 1e-6f) return 0.0f;
        float cosTheta = dot(other) / denom;
        cosTheta = cosTheta < -1.0f ? -1.0f : (cosTheta > 1.0f ? 1.0f : cosTheta);
        return std::acos(cosTheta);
    }

    Vec2 clamped(float maxLength) const {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    Vec2& clamp(float maxLength) {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    Vec2 lerp(const Vec2& other, float t) const {
        return *this + (other - *this) * t;
    }

    Vec2 project(const Vec2& onto) const {
        float ontoLenSq = onto.squaredLength();
        if (ontoLenSq < 1e-6f) return Vec2(0.0f);
        return onto * (dot(onto) / ontoLenSq);
    }

    // 2D reflection (reflect across a line defined by normal)
    Vec2 reflect(const Vec2& normal) const {
        return *this - normal * (2.0f * dot(normal));
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
        return x == 0.0f && y == 0.0f;
    }

    bool isNormalized() const {
        float lenSq = squaredLength();
        return std::abs(lenSq - 1.0f) < 1e-5f;
    }

    bool approxEqual(const Vec2& other, float epsilon = 1e-5f) const {
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
    }

    Vec2 abs() const {
        return Vec2(std::abs(x), std::abs(y));
    }

    // Static utility vectors
    static Vec2 zero() { return Vec2(0.0f, 0.0f); }
    static Vec2 one() { return Vec2(1.0f, 1.0f); }
    static Vec2 up() { return Vec2(0.0f, 1.0f); }
    static Vec2 down() { return Vec2(0.0f, -1.0f); }
    static Vec2 right() { return Vec2(1.0f, 0.0f); }
    static Vec2 left() { return Vec2(-1.0f, 0.0f); }
};

inline Vec2 operator*(float scalar, const Vec2& v) {
    return v * scalar;
}

inline std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

}  // namespace phynity::math::vectors

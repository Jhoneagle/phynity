#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>

namespace phynity::math::vectors {

/// Four-component floating-point vector.
struct Vec4 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 0.0f;

    // Constructors
    Vec4() = default;
    explicit Vec4(float v) : x(v), y(v), z(v), w(v) {}
    Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    // Operators
    Vec4 operator+(const Vec4& other) const {
        return Vec4(x + other.x, y + other.y, z + other.z, w + other.w);
    }

    Vec4 operator-(const Vec4& other) const {
        return Vec4(x - other.x, y - other.y, z - other.z, w - other.w);
    }

    Vec4 operator*(float scalar) const {
        return Vec4(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    Vec4 operator/(float scalar) const {
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

    Vec4& operator*=(float scalar) {
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

    Vec4& operator/=(float scalar) {
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

    float& operator[](int i) {
        return (i == 0) ? x : (i == 1) ? y : (i == 2) ? z : w;
    }

    const float& operator[](int i) const {
        return (i == 0) ? x : (i == 1) ? y : (i == 2) ? z : w;
    }

    float dot(const Vec4& other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    float squaredLength() const {
        return dot(*this);
    }

    float length() const {
        return std::sqrt(squaredLength());
    }

    Vec4 normalized() const {
        float len = length();
        return len > 0.0f ? *this / len : Vec4(0.0f);
    }

    Vec4& normalize() {
        float len = length();
        if (len > 0.0f) {
            *this /= len;
        }
        return *this;
    }

    float distance(const Vec4& other) const {
        return (*this - other).length();
    }

    float squaredDistance(const Vec4& other) const {
        return (*this - other).squaredLength();
    }

    float angle(const Vec4& other) const {
        float denom = length() * other.length();
        if (denom < 1e-6f) return 0.0f;
        float cosTheta = dot(other) / denom;
        cosTheta = cosTheta < -1.0f ? -1.0f : (cosTheta > 1.0f ? 1.0f : cosTheta);
        return std::acos(cosTheta);
    }

    Vec4 clamped(float maxLength) const {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    Vec4& clamp(float maxLength) {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    Vec4 lerp(const Vec4& other, float t) const {
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
        return x == 0.0f && y == 0.0f && z == 0.0f && w == 0.0f;
    }

    bool isNormalized() const {
        float lenSq = squaredLength();
        return std::abs(lenSq - 1.0f) < 1e-5f;
    }

    bool approxEqual(const Vec4& other, float epsilon = 1e-5f) const {
        return std::abs(x - other.x) < epsilon && 
               std::abs(y - other.y) < epsilon && 
               std::abs(z - other.z) < epsilon && 
               std::abs(w - other.w) < epsilon;
    }

    Vec4 abs() const {
        return Vec4(std::abs(x), std::abs(y), std::abs(z), std::abs(w));
    }

    // Static utility vectors
    static Vec4 zero() { return Vec4(0.0f, 0.0f, 0.0f, 0.0f); }
    static Vec4 one() { return Vec4(1.0f, 1.0f, 1.0f, 1.0f); }
    static Vec4 unitX() { return Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    static Vec4 unitY() { return Vec4(0.0f, 1.0f, 0.0f, 0.0f); }
    static Vec4 unitZ() { return Vec4(0.0f, 0.0f, 1.0f, 0.0f); }
    static Vec4 unitW() { return Vec4(0.0f, 0.0f, 0.0f, 1.0f); }
};

inline Vec4 operator*(float scalar, const Vec4& v) {
    return v * scalar;
}

inline std::ostream& operator<<(std::ostream& os, const Vec4& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
    return os;
}

}  // namespace phynity::math::vectors

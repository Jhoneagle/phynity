#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>

namespace phynity::math::vectors {

/// Three-component floating-point vector.
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Constructors
    Vec3() = default;
    explicit Vec3(float v) : x(v), y(v), z(v) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Operators
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 operator/(float scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    Vec3 operator*(const Vec3& other) const {
        return Vec3(x * other.x, y * other.y, z * other.z);
    }

    Vec3 operator/(const Vec3& other) const {
        return Vec3(x / other.x, y / other.y, z / other.z);
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    bool operator==(const Vec3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const Vec3& other) const {
        return !(*this == other);
    }

    float& operator[](int i) {
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    const float& operator[](int i) const {
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    float squaredLength() const {
        return dot(*this);
    }

    float length() const {
        return std::sqrt(squaredLength());
    }

    Vec3 normalized() const {
        float len = length();
        return len > 0.0f ? *this / len : Vec3(0.0f);
    }

    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    float distance(const Vec3& other) const {
        return (*this - other).length();
    }

    float squaredDistance(const Vec3& other) const {
        return (*this - other).squaredLength();
    }

    float angle(const Vec3& other) const {
        float denom = length() * other.length();
        if (denom < 1e-6f) return 0.0f;  // Handle zero-length vectors
        float cosTheta = dot(other) / denom;
        // Clamp to [-1, 1] to handle floating point errors
        cosTheta = cosTheta < -1.0f ? -1.0f : (cosTheta > 1.0f ? 1.0f : cosTheta);
        return std::acos(cosTheta);
    }

    Vec3 clamped(float maxLength) const {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    Vec3& clamp(float maxLength) {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    Vec3 lerp(const Vec3& other, float t) const {
        return *this + (other - *this) * t;
    }

    Vec3 project(const Vec3& onto) const {
        float ontoLenSq = onto.squaredLength();
        if (ontoLenSq < 1e-6f) return Vec3(0.0f);
        return onto * (dot(onto) / ontoLenSq);
    }

    Vec3 reflect(const Vec3& normal) const {
        return *this - normal * (2.0f * dot(normal));
    }

    Vec3 perpendicular() const {
        // Choose axis with smallest absolute component to avoid parallel vectors
        Vec3 axis = (std::abs(x) < std::abs(y) && std::abs(x) < std::abs(z)) 
                    ? Vec3(1.0f, 0.0f, 0.0f)
                    : (std::abs(y) < std::abs(z) ? Vec3(0.0f, 1.0f, 0.0f) : Vec3(0.0f, 0.0f, 1.0f));
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
        return x == 0.0f && y == 0.0f && z == 0.0f;
    }

    bool isNormalized() const {
        float lenSq = squaredLength();
        return std::abs(lenSq - 1.0f) < 1e-5f;
    }

    // Static utility vectors
    static Vec3 zero() { return Vec3(0.0f, 0.0f, 0.0f); }
    static Vec3 one() { return Vec3(1.0f, 1.0f, 1.0f); }
    static Vec3 up() { return Vec3(0.0f, 1.0f, 0.0f); }
    static Vec3 down() { return Vec3(0.0f, -1.0f, 0.0f); }
    static Vec3 right() { return Vec3(1.0f, 0.0f, 0.0f); }
    static Vec3 left() { return Vec3(-1.0f, 0.0f, 0.0f); }
    static Vec3 forward() { return Vec3(0.0f, 0.0f, 1.0f); }
    static Vec3 back() { return Vec3(0.0f, 0.0f, -1.0f); }
};

inline Vec3 operator*(float scalar, const Vec3& v) {
    return v * scalar;
}

inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

}  // namespace phynity::math::vectors

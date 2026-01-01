#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>
#include <array>

namespace phynity::math::vectors {

/// Fixed-length floating-point vector with compile-time size.
template<std::size_t N>
struct VecN {
    std::array<float, N> data{};

    // Constructors
    VecN() = default;
    
    explicit VecN(float v) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] = v;
        }
    }

    // Operators
    VecN operator+(const VecN& other) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }

    VecN operator-(const VecN& other) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }

    VecN operator*(float scalar) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    VecN operator/(float scalar) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] / scalar;
        }
        return result;
    }

    VecN operator*(const VecN& other) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] * other.data[i];
        }
        return result;
    }

    VecN operator/(const VecN& other) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] / other.data[i];
        }
        return result;
    }

    VecN& operator+=(const VecN& other) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] += other.data[i];
        }
        return *this;
    }

    VecN& operator-=(const VecN& other) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] -= other.data[i];
        }
        return *this;
    }

    VecN& operator*=(float scalar) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] *= scalar;
        }
        return *this;
    }

    VecN& operator/=(float scalar) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] /= scalar;
        }
        return *this;
    }

    VecN operator-() const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = -data[i];
        }
        return result;
    }

    bool operator==(const VecN& other) const {
        for (std::size_t i = 0; i < N; ++i) {
            if (data[i] != other.data[i]) return false;
        }
        return true;
    }

    bool operator!=(const VecN& other) const {
        return !(*this == other);
    }

    float& operator[](std::size_t i) {
        return data[i];
    }

    const float& operator[](std::size_t i) const {
        return data[i];
    }

    float dot(const VecN& other) const {
        float result = 0.0f;
        for (std::size_t i = 0; i < N; ++i) {
            result += data[i] * other.data[i];
        }
        return result;
    }

    float squaredLength() const {
        return dot(*this);
    }

    float length() const {
        return std::sqrt(squaredLength());
    }

    VecN normalized() const {
        float len = length();
        return len > 0.0f ? *this / len : VecN(0.0f);
    }

    float distance(const VecN& other) const {
        return (*this - other).length();
    }

    float squaredDistance(const VecN& other) const {
        return (*this - other).squaredLength();
    }

    float angle(const VecN& other) const {
        float denom = length() * other.length();
        if (denom < 1e-6f) return 0.0f;
        float cosTheta = dot(other) / denom;
        cosTheta = cosTheta < -1.0f ? -1.0f : (cosTheta > 1.0f ? 1.0f : cosTheta);
        return std::acos(cosTheta);
    }

    VecN clamped(float maxLength) const {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    VecN& clamp(float maxLength) {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    VecN lerp(const VecN& other, float t) const {
        return *this + (other - *this) * t;
    }

    VecN project(const VecN& onto) const {
        float ontoLenSq = onto.squaredLength();
        if (ontoLenSq < 1e-6f) return VecN(0.0f);
        return onto * (dot(onto) / ontoLenSq);
    }

    VecN reflect(const VecN& normal) const {
        return *this - normal * (2.0f * dot(normal));
    }

    VecN min(const VecN& other) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] < other.data[i] ? data[i] : other.data[i];
        }
        return result;
    }

    VecN max(const VecN& other) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] > other.data[i] ? data[i] : other.data[i];
        }
        return result;
    }

    bool isZero() const {
        for (std::size_t i = 0; i < N; ++i) {
            if (data[i] != 0.0f) return false;
        }
        return true;
    }

    bool isNormalized() const {
        float lenSq = squaredLength();
        return std::abs(lenSq - 1.0f) < 1e-5f;
    }

    // Get size
    static constexpr std::size_t size() {
        return N;
    }
};

// Scalar multiplication from left
template<std::size_t N>
inline VecN<N> operator*(float scalar, const VecN<N>& v) {
    return v * scalar;
}

// Stream output
template<std::size_t N>
inline std::ostream& operator<<(std::ostream& os, const VecN<N>& v) {
    os << "(";
    for (std::size_t i = 0; i < N; ++i) {
        if (i > 0) os << ", ";
        os << v.data[i];
    }
    os << ")";
    return os;
}

// Useful type aliases
using Vec6 = VecN<6>;
using Vec8 = VecN<8>;
using Vec16 = VecN<16>;

}  // namespace phynity::math::vectors

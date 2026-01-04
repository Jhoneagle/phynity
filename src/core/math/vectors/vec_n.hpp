#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>
#include <array>
#include <type_traits>

namespace phynity::math::vectors {

/// Fixed-length floating-point vector with compile-time size and precision.
template<std::size_t N, typename T = float>
struct VecN {
    static_assert(std::is_floating_point_v<T>, "VecN template parameter must be a floating-point type");
    std::array<T, N> data{};

    // Constructors
    VecN() = default;
    
    explicit VecN(T v) {
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

    VecN operator*(T scalar) const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    VecN operator/(T scalar) const {
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

    VecN& operator*=(T scalar) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] *= scalar;
        }
        return *this;
    }

    VecN& operator*=(const VecN& other) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] *= other.data[i];
        }
        return *this;
    }

    VecN& operator/=(T scalar) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] /= scalar;
        }
        return *this;
    }

    VecN& operator/=(const VecN& other) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] /= other.data[i];
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

    T& operator[](std::size_t i) {
        return data[i];
    }

    const T& operator[](std::size_t i) const {
        return data[i];
    }

    T dot(const VecN& other) const {
        T result = T(0);
        for (std::size_t i = 0; i < N; ++i) {
            result += data[i] * other.data[i];
        }
        return result;
    }

    T squaredLength() const {
        return dot(*this);
    }

    T length() const {
        return std::sqrt(squaredLength());
    }

    VecN normalized() const {
        T len = length();
        return len > T(0) ? *this / len : VecN(T(0));
    }

    VecN& normalize() {
        T len = length();
        if (len > T(0)) {
            *this /= len;
        }
        return *this;
    }

    T distance(const VecN& other) const {
        return (*this - other).length();
    }

    T squaredDistance(const VecN& other) const {
        return (*this - other).squaredLength();
    }

    T angle(const VecN& other) const {
        T denom = length() * other.length();
        if (denom < T(1e-6)) return T(0);
        T cosTheta = dot(other) / denom;
        cosTheta = cosTheta < T(-1) ? T(-1) : (cosTheta > T(1) ? T(1) : cosTheta);
        return std::acos(cosTheta);
    }

    VecN clamped(T maxLength) const {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    VecN& clamp(T maxLength) {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    VecN lerp(const VecN& other, T t) const {
        return *this + (other - *this) * t;
    }

    VecN project(const VecN& onto) const {
        T ontoLenSq = onto.squaredLength();
        if (ontoLenSq < T(1e-6)) return VecN(T(0));
        return onto * (dot(onto) / ontoLenSq);
    }

    VecN reflect(const VecN& normal) const {
        return *this - normal * (T(2) * dot(normal));
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
            if (data[i] != T(0)) return false;
        }
        return true;
    }

    bool isNormalized() const {
        T lenSq = squaredLength();
        return std::abs(lenSq - T(1)) < T(1e-5);
    }

    bool approxEqual(const VecN& other, T epsilon = T(1e-5)) const {
        for (std::size_t i = 0; i < N; ++i) {
            if (std::abs(data[i] - other.data[i]) >= epsilon) return false;
        }
        return true;
    }

    VecN abs() const {
        VecN result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = std::abs(data[i]);
        }
        return result;
    }

    // Get size
    static constexpr std::size_t size() {
        return N;
    }
};

// Scalar multiplication from left
template<std::size_t N, typename T = float>
inline VecN<N, T> operator*(T scalar, const VecN<N, T>& v) {
    return v * scalar;
}

// Stream output
template<std::size_t N, typename T = float>
inline std::ostream& operator<<(std::ostream& os, const VecN<N, T>& v) {
    os << "(";
    for (std::size_t i = 0; i < N; ++i) {
        if (i > 0) os << ", ";
        os << v.data[i];
    }
    os << ")";
    return os;
}

// Useful type aliases
using Vec6f = VecN<6, float>;
using Vec6d = VecN<6, double>;

using Vec8f = VecN<8, float>;
using Vec8d = VecN<8, double>;

using Vec16f = VecN<16, float>;
using Vec16d = VecN<16, double>;

}  // namespace phynity::math::vectors

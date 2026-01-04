#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>
#include <vector>
#include <stdexcept>
#include <type_traits>

namespace phynity::math::vectors {

/// Runtime-sized floating-point vector with dynamic dimensions and precision.
template<typename T = float>
class VecDynamic {
    static_assert(std::is_floating_point_v<T>, "VecDynamic template parameter must be a floating-point type");
private:
    std::vector<T> data;

public:
    // Constructors
    VecDynamic() = default;

    explicit VecDynamic(std::size_t size) : data(size, T(0)) {}

    VecDynamic(std::size_t size, T v) : data(size, v) {}

    template<typename Iter>
    VecDynamic(Iter begin, Iter end) : data(begin, end) {}

    // Accessors
    std::size_t size() const {
        return data.size();
    }

    bool empty() const {
        return data.empty();
    }

    T& operator[](std::size_t i) {
        return data[i];
    }

    const T& operator[](std::size_t i) const {
        return data[i];
    }

    T& at(std::size_t i) {
        if (i >= size()) throw std::out_of_range("VecDynamic index out of range");
        return data[i];
    }

    const T& at(std::size_t i) const {
        if (i >= size()) throw std::out_of_range("VecDynamic index out of range");
        return data[i];
    }

    // Operators
    VecDynamic operator+(const VecDynamic& other) const {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }

    VecDynamic operator-(const VecDynamic& other) const {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }

    VecDynamic operator*(T scalar) const {
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    VecDynamic operator/(T scalar) const {
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] / scalar;
        }
        return result;
    }

    VecDynamic operator*(const VecDynamic& other) const {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] * other.data[i];
        }
        return result;
    }

    VecDynamic operator/(const VecDynamic& other) const {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] / other.data[i];
        }
        return result;
    }

    VecDynamic& operator+=(const VecDynamic& other) {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        for (std::size_t i = 0; i < size(); ++i) {
            data[i] += other.data[i];
        }
        return *this;
    }

    VecDynamic& operator-=(const VecDynamic& other) {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        for (std::size_t i = 0; i < size(); ++i) {
            data[i] -= other.data[i];
        }
        return *this;
    }

    VecDynamic& operator*=(T scalar) {
        for (std::size_t i = 0; i < size(); ++i) {
            data[i] *= scalar;
        }
        return *this;
    }

    VecDynamic& operator*=(const VecDynamic& other) {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        for (std::size_t i = 0; i < size(); ++i) {
            data[i] *= other.data[i];
        }
        return *this;
    }

    VecDynamic& operator/=(T scalar) {
        for (std::size_t i = 0; i < size(); ++i) {
            data[i] /= scalar;
        }
        return *this;
    }

    VecDynamic& operator/=(const VecDynamic& other) {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        for (std::size_t i = 0; i < size(); ++i) {
            data[i] /= other.data[i];
        }
        return *this;
    }

    VecDynamic operator-() const {
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = -data[i];
        }
        return result;
    }

    bool operator==(const VecDynamic& other) const {
        if (size() != other.size()) return false;
        for (std::size_t i = 0; i < size(); ++i) {
            if (data[i] != other.data[i]) return false;
        }
        return true;
    }

    bool operator!=(const VecDynamic& other) const {
        return !(*this == other);
    }

    // Vector operations
    T dot(const VecDynamic& other) const {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        T result = T(0);
        for (std::size_t i = 0; i < size(); ++i) {
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

    VecDynamic normalized() const {
        T len = length();
        return len > T(0) ? *this / len : VecDynamic(size(), T(0));
    }

    VecDynamic& normalize() {
        T len = length();
        if (len > T(0)) {
            *this /= len;
        }
        return *this;
    }

    T distance(const VecDynamic& other) const {
        return (*this - other).length();
    }

    T squaredDistance(const VecDynamic& other) const {
        return (*this - other).squaredLength();
    }

    T angle(const VecDynamic& other) const {
        T denom = length() * other.length();
        if (denom < T(1e-6)) return T(0);
        T cosTheta = dot(other) / denom;
        cosTheta = cosTheta < T(-1) ? T(-1) : (cosTheta > T(1) ? T(1) : cosTheta);
        return std::acos(cosTheta);
    }

    VecDynamic clamped(T maxLength) const {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    VecDynamic& clamp(T maxLength) {
        T lenSq = squaredLength();
        T maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    VecDynamic lerp(const VecDynamic& other, T t) const {
        return *this + (other - *this) * t;
    }

    VecDynamic project(const VecDynamic& onto) const {
        T ontoLenSq = onto.squaredLength();
        if (ontoLenSq < T(1e-6)) return VecDynamic(size(), T(0));
        return onto * (dot(onto) / ontoLenSq);
    }

    VecDynamic reflect(const VecDynamic& normal) const {
        return *this - normal * (T(2) * dot(normal));
    }

    VecDynamic min(const VecDynamic& other) const {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] < other.data[i] ? data[i] : other.data[i];
        }
        return result;
    }

    VecDynamic max(const VecDynamic& other) const {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] > other.data[i] ? data[i] : other.data[i];
        }
        return result;
    }

    bool isZero() const {
        for (std::size_t i = 0; i < size(); ++i) {
            if (data[i] != T(0)) return false;
        }
        return true;
    }

    bool isNormalized() const {
        T lenSq = squaredLength();
        return std::abs(lenSq - T(1)) < T(1e-5);
    }

    bool approxEqual(const VecDynamic& other, T epsilon = T(1e-5)) const {
        if (size() != other.size()) return false;
        for (std::size_t i = 0; i < size(); ++i) {
            if (std::abs(data[i] - other.data[i]) >= epsilon) return false;
        }
        return true;
    }

    VecDynamic abs() const {
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = std::abs(data[i]);
        }
        return result;
    }

    // Resize
    void resize(std::size_t newSize) {
        data.resize(newSize, T(0));
    }

    void resize(std::size_t newSize, T value) {
        data.resize(newSize, value);
    }
};

// Scalar multiplication from left
template<typename T = float>
inline VecDynamic<T> operator*(T scalar, const VecDynamic<T>& v) {
    return v * scalar;
}

// Stream output
template<typename T = float>
inline std::ostream& operator<<(std::ostream& os, const VecDynamic<T>& v) {
    os << "(";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i > 0) os << ", ";
        os << v[i];
    }
    os << ")";
    return os;
}

// Type aliases
using VecDynamicf = VecDynamic<float>;
using VecDynamicd = VecDynamic<double>;

}  // namespace phynity::math::vectors

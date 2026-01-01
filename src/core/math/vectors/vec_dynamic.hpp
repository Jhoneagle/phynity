#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>
#include <vector>
#include <stdexcept>

namespace phynity::math::vectors {

/// Runtime-sized floating-point vector with dynamic dimensions.
class VecDynamic {
private:
    std::vector<float> data;

public:
    // Constructors
    VecDynamic() = default;

    explicit VecDynamic(std::size_t size) : data(size, 0.0f) {}

    VecDynamic(std::size_t size, float v) : data(size, v) {}

    template<typename Iter>
    VecDynamic(Iter begin, Iter end) : data(begin, end) {}

    // Accessors
    std::size_t size() const {
        return data.size();
    }

    bool empty() const {
        return data.empty();
    }

    float& operator[](std::size_t i) {
        return data[i];
    }

    const float& operator[](std::size_t i) const {
        return data[i];
    }

    float& at(std::size_t i) {
        if (i >= size()) throw std::out_of_range("VecDynamic index out of range");
        return data[i];
    }

    const float& at(std::size_t i) const {
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

    VecDynamic operator*(float scalar) const {
        VecDynamic result(size());
        for (std::size_t i = 0; i < size(); ++i) {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    VecDynamic operator/(float scalar) const {
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

    VecDynamic& operator*=(float scalar) {
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

    VecDynamic& operator/=(float scalar) {
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
    float dot(const VecDynamic& other) const {
        if (size() != other.size()) throw std::invalid_argument("Vector sizes do not match");
        float result = 0.0f;
        for (std::size_t i = 0; i < size(); ++i) {
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

    VecDynamic normalized() const {
        float len = length();
        return len > 0.0f ? *this / len : VecDynamic(size(), 0.0f);
    }

    VecDynamic& normalize() {
        float len = length();
        if (len > 0.0f) {
            *this /= len;
        }
        return *this;
    }

    float distance(const VecDynamic& other) const {
        return (*this - other).length();
    }

    float squaredDistance(const VecDynamic& other) const {
        return (*this - other).squaredLength();
    }

    float angle(const VecDynamic& other) const {
        float denom = length() * other.length();
        if (denom < 1e-6f) return 0.0f;
        float cosTheta = dot(other) / denom;
        cosTheta = cosTheta < -1.0f ? -1.0f : (cosTheta > 1.0f ? 1.0f : cosTheta);
        return std::acos(cosTheta);
    }

    VecDynamic clamped(float maxLength) const {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq <= maxLenSq) return *this;
        return *this * (maxLength / std::sqrt(lenSq));
    }

    VecDynamic& clamp(float maxLength) {
        float lenSq = squaredLength();
        float maxLenSq = maxLength * maxLength;
        if (lenSq > maxLenSq) {
            *this *= (maxLength / std::sqrt(lenSq));
        }
        return *this;
    }

    VecDynamic lerp(const VecDynamic& other, float t) const {
        return *this + (other - *this) * t;
    }

    VecDynamic project(const VecDynamic& onto) const {
        float ontoLenSq = onto.squaredLength();
        if (ontoLenSq < 1e-6f) return VecDynamic(size(), 0.0f);
        return onto * (dot(onto) / ontoLenSq);
    }

    VecDynamic reflect(const VecDynamic& normal) const {
        return *this - normal * (2.0f * dot(normal));
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
            if (data[i] != 0.0f) return false;
        }
        return true;
    }

    bool isNormalized() const {
        float lenSq = squaredLength();
        return std::abs(lenSq - 1.0f) < 1e-5f;
    }

    bool approxEqual(const VecDynamic& other, float epsilon = 1e-5f) const {
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
        data.resize(newSize, 0.0f);
    }

    void resize(std::size_t newSize, float value) {
        data.resize(newSize, value);
    }
};

// Scalar multiplication from left
inline VecDynamic operator*(float scalar, const VecDynamic& v) {
    return v * scalar;
}

// Stream output
inline std::ostream& operator<<(std::ostream& os, const VecDynamic& v) {
    os << "(";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i > 0) os << ", ";
        os << v[i];
    }
    os << ")";
    return os;
}

}  // namespace phynity::math::vectors

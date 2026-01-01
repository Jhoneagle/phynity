#pragma once

#include <core/math/vectors/vec_dynamic.hpp>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace phynity::math::matrices {

using phynity::math::vectors::VecDynamic;

/// Runtime-sized matrix with row-major storage
struct MatDynamic {
    std::size_t rows{0};
    std::size_t cols{0};
    std::vector<float> data{};

    MatDynamic() = default;

    MatDynamic(std::size_t r, std::size_t c) : rows(r), cols(c), data(r * c, 0.0f) {}

    MatDynamic(std::size_t r, std::size_t c, float scalar)
        : rows(r), cols(c), data(r * c, scalar) {}

    /// Element access (row, col)
    float& operator()(std::size_t row, std::size_t col) {
        return data[row * cols + col];
    }

    const float& operator()(std::size_t row, std::size_t col) const {
        return data[row * cols + col];
    }

    /// Bounds-checked element access
    float& at(std::size_t row, std::size_t col) {
        if (row >= rows || col >= cols) {
            throw std::out_of_range("MatDynamic index out of range");
        }
        return (*this)(row, col);
    }

    const float& at(std::size_t row, std::size_t col) const {
        if (row >= rows || col >= cols) {
            throw std::out_of_range("MatDynamic index out of range");
        }
        return (*this)(row, col);
    }

    bool operator==(const MatDynamic& other) const {
        return rows == other.rows && cols == other.cols && data == other.data;
    }

    bool operator!=(const MatDynamic& other) const {
        return !(*this == other);
    }

    MatDynamic operator+(const MatDynamic& other) const {
        validate_same_shape(other, "+");
        MatDynamic result(rows, cols);
        for (std::size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }

    MatDynamic operator-(const MatDynamic& other) const {
        validate_same_shape(other, "-");
        MatDynamic result(rows, cols);
        for (std::size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }

    MatDynamic operator*(float scalar) const {
        MatDynamic result(rows, cols);
        for (std::size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    MatDynamic operator/(float scalar) const {
        MatDynamic result(rows, cols);
        for (std::size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] / scalar;
        }
        return result;
    }

    MatDynamic& operator+=(const MatDynamic& other) {
        validate_same_shape(other, "+=");
        for (std::size_t i = 0; i < data.size(); ++i) {
            data[i] += other.data[i];
        }
        return *this;
    }

    MatDynamic& operator-=(const MatDynamic& other) {
        validate_same_shape(other, "-=");
        for (std::size_t i = 0; i < data.size(); ++i) {
            data[i] -= other.data[i];
        }
        return *this;
    }

    MatDynamic& operator*=(float scalar) {
        for (float& value : data) {
            value *= scalar;
        }
        return *this;
    }

    MatDynamic& operator/=(float scalar) {
        for (float& value : data) {
            value /= scalar;
        }
        return *this;
    }

    MatDynamic operator*(const MatDynamic& other) const {
        if (cols != other.rows) {
            throw std::invalid_argument("Matrix multiplication shape mismatch");
        }
        MatDynamic result(rows, other.cols);
        for (std::size_t i = 0; i < rows; ++i) {
            for (std::size_t j = 0; j < other.cols; ++j) {
                float sum = 0.0f;
                for (std::size_t k = 0; k < cols; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

    VecDynamic operator*(const VecDynamic& v) const {
        if (cols != v.size()) {
            throw std::invalid_argument("Matrix-vector multiplication dimension mismatch");
        }
        VecDynamic result(rows);
        for (std::size_t i = 0; i < rows; ++i) {
            float sum = 0.0f;
            for (std::size_t k = 0; k < cols; ++k) {
                sum += (*this)(i, k) * v[k];
            }
            result[i] = sum;
        }
        return result;
    }

    MatDynamic transposed() const {
        MatDynamic result(cols, rows);
        for (std::size_t i = 0; i < rows; ++i) {
            for (std::size_t j = 0; j < cols; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    MatDynamic& transpose() {
        *this = transposed();
        return *this;
    }

    static MatDynamic zero(std::size_t r, std::size_t c) {
        return MatDynamic(r, c);
    }

    static MatDynamic identity(std::size_t n) {
        MatDynamic result(n, n);
        for (std::size_t i = 0; i < n; ++i) {
            result(i, i) = 1.0f;
        }
        return result;
    }

private:
    void validate_same_shape(const MatDynamic& other, const char* op) const {
        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument(std::string("Shape mismatch for operator ") + op);
        }
    }
};

inline MatDynamic operator*(float scalar, const MatDynamic& m) {
    return m * scalar;
}

}  // namespace phynity::math::matrices

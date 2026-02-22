#pragma once

#include <core/math/vectors/vec_dynamic.hpp>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include <ostream>
#include <type_traits>

namespace phynity::math::matrices {

using phynity::math::vectors::VecDynamic;

/// Lightweight view proxy for a row in MatDynamic (avoids copying)
template<typename T>
class MatRowView {
    template<typename U>
    friend struct MatDynamic;

public:
    T& operator[](std::size_t col) {
        return (*data_)[row_index_ * col_count_ + col];
    }

    const T& operator[](std::size_t col) const {
        return (*data_)[row_index_ * col_count_ + col];
    }

    std::size_t size() const { return col_count_; }

private:
    MatRowView(std::vector<T>* data, std::size_t row_idx, std::size_t col_count)
        : data_(data), row_index_(row_idx), col_count_(col_count) {}

    std::vector<T>* data_;
    std::size_t row_index_;
    std::size_t col_count_;
};

/// Lightweight view proxy for a column in MatDynamic (avoids copying)
template<typename T>
class MatColView {
    template<typename U>
    friend struct MatDynamic;

public:
    T& operator[](std::size_t row) {
        return (*data_)[row * col_count_ + col_index_];
    }

    const T& operator[](std::size_t row) const {
        return (*data_)[row * col_count_ + col_index_];
    }

    std::size_t size() const { return row_count_; }

private:
    MatColView(std::vector<T>* data, std::size_t col_idx, std::size_t row_count, std::size_t col_count)
        : data_(data), col_index_(col_idx), row_count_(row_count), col_count_(col_count) {}

    std::vector<T>* data_;
    std::size_t col_index_;
    std::size_t row_count_;
    std::size_t col_count_;
};

template<typename T = float>
struct MatDynamic {
    static_assert(std::is_floating_point_v<T>, "MatDynamic template parameter must be a floating-point type");

    MatDynamic() = default;

    MatDynamic(std::size_t r, std::size_t c) : rows(r), cols(c), data(r * c, T(0)) {}

    MatDynamic(std::size_t r, std::size_t c, T scalar)
        : rows(r), cols(c), data(r * c, scalar) {}

    // Move semantics
    MatDynamic(MatDynamic&& other) noexcept = default;
    MatDynamic& operator=(MatDynamic&& other) noexcept = default;

    // Copy semantics (implicitly defined, explicitly for clarity)
    MatDynamic(const MatDynamic& other) = default;
    MatDynamic& operator=(const MatDynamic& other) = default;

    /// Query methods
    std::size_t numRows() const { return rows; }
    std::size_t numCols() const { return cols; }
    std::size_t size() const { return data.size(); }
    bool isEmpty() const { return data.empty(); }

    /// Element access (row, col)
    T& operator()(std::size_t row, std::size_t col) {
        return data[row * cols + col];
    }

    const T& operator()(std::size_t row, std::size_t col) const {
        return data[row * cols + col];
    }

    /// Bounds-checked element access
    T& at(std::size_t row, std::size_t col) {
        if (row >= rows || col >= cols) {
            throw std::out_of_range("MatDynamic index out of range");
        }
        return (*this)(row, col);
    }

    const T& at(std::size_t row, std::size_t col) const {
        if (row >= rows || col >= cols) {
            throw std::out_of_range("MatDynamic index out of range");
        }
        return (*this)(row, col);
    }

    /// Get row as vector
    VecDynamic<T> getRow(std::size_t row) const {
        VecDynamic<T> result(cols);
        for (std::size_t j = 0; j < cols; ++j) {
            result[j] = (*this)(row, j);
        }
        return result;
    }

    /// Get row as lightweight view (no copying)
    MatRowView<T> row(std::size_t row_idx) {
        if (row_idx >= rows) {
            throw std::out_of_range("Row index out of range");
        }
        return MatRowView<T>(&data, row_idx, cols);
    }

    /// Get column as vector
    VecDynamic<T> getColumn(std::size_t col) const {
        VecDynamic<T> result(rows);
        for (std::size_t i = 0; i < rows; ++i) {
            result[i] = (*this)(i, col);
        }
        return result;
    }

    /// Get column as lightweight view (no copying)
    MatColView<T> col(std::size_t col_idx) {
        if (col_idx >= cols) {
            throw std::out_of_range("Column index out of range");
        }
        return MatColView<T>(&data, col_idx, rows, cols);
    }

    /// Set row from vector
    void setRow(std::size_t row, const VecDynamic<T>& v) {
        if (v.size() != cols) {
            throw std::invalid_argument("Vector size does not match number of columns");
        }
        for (std::size_t j = 0; j < cols; ++j) {
            (*this)(row, j) = v[j];
        }
    }

    /// Set column from vector
    void setColumn(std::size_t col, const VecDynamic<T>& v) {
        if (v.size() != rows) {
            throw std::invalid_argument("Vector size does not match number of rows");
        }
        for (std::size_t i = 0; i < rows; ++i) {
            (*this)(i, col) = v[i];
        }
    }

    bool operator==(const MatDynamic& other) const {
        return rows == other.rows && cols == other.cols && data == other.data;
    }

    bool operator!=(const MatDynamic& other) const {
        return !(*this == other);
    }

    MatDynamic operator-() const {
        MatDynamic result(rows, cols);
        for (std::size_t i = 0; i < data.size(); ++i) {
            result.data[i] = -data[i];
        }
        return result;
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

    MatDynamic operator*(T scalar) const {
        MatDynamic result(rows, cols);
        for (std::size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    MatDynamic operator/(T scalar) const {
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

    MatDynamic& operator*=(T scalar) {
        for (T& value : data) {
            value *= scalar;
        }
        return *this;
    }

    MatDynamic& operator/=(T scalar) {
        for (T& value : data) {
            value /= scalar;
        }
        return *this;
    }

    /// Component-wise (Hadamard) multiplication
    MatDynamic& mulComponentWise(const MatDynamic& other) {
        validate_same_shape(other, "mulComponentWise");
        for (std::size_t i = 0; i < data.size(); ++i) {
            data[i] *= other.data[i];
        }
        return *this;
    }

    /// Component-wise division
    MatDynamic& divComponentWise(const MatDynamic& other) {
        validate_same_shape(other, "divComponentWise");
        for (std::size_t i = 0; i < data.size(); ++i) {
            data[i] /= other.data[i];
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
                T sum = T(0);
                for (std::size_t k = 0; k < cols; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

    VecDynamic<T> operator*(const VecDynamic<T>& v) const {
        if (cols != v.size()) {
            throw std::invalid_argument("Matrix-vector multiplication dimension mismatch");
        }
        VecDynamic<T> result(rows);
        for (std::size_t i = 0; i < rows; ++i) {
            T sum = T(0);
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

    /// Approximate equality with epsilon tolerance
    bool approxEqual(const MatDynamic& other, T epsilon = T(1e-5)) const {
        if (rows != other.rows || cols != other.cols) {
            return false;
        }
        for (std::size_t i = 0; i < data.size(); ++i) {
            if (std::abs(data[i] - other.data[i]) >= epsilon) {
                return false;
            }
        }
        return true;
    }

    /// Return matrix with absolute values of all elements
    MatDynamic abs() const {
        MatDynamic result(rows, cols);
        for (std::size_t i = 0; i < data.size(); ++i) {
            result.data[i] = std::abs(data[i]);
        }
        return result;
    }

    static MatDynamic zero(std::size_t r, std::size_t c) {
        return MatDynamic(r, c);
    }

    static MatDynamic identity(std::size_t n) {
        MatDynamic result(n, n);
        for (std::size_t i = 0; i < n; ++i) {
            result(i, i) = T(1);
        }
        return result;
    }

    // Raw pointer to first element (row-major)
    T* dataPtr() {
        return data.data();
    }

    const T* dataPtr() const {
        return data.data();
    }

private:
    std::size_t rows{0};
    std::size_t cols{0};
    std::vector<T> data{};

private:
    void validate_same_shape(const MatDynamic& other, const char* op) const {
        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument(std::string("Shape mismatch for operator ") + op);
        }
    }
};

template<typename T = float>
inline MatDynamic<T> operator*(T scalar, const MatDynamic<T>& m) {
    return m * scalar;
}

/// Stream output
template<typename T = float>
inline std::ostream& operator<<(std::ostream& os, const MatDynamic<T>& m) {
    os << "[";
    for (std::size_t i = 0; i < m.numRows(); ++i) {
        if (i > 0) os << ", ";
        os << "(";
        for (std::size_t j = 0; j < m.numCols(); ++j) {
            if (j > 0) os << ", ";
            os << m(i, j);
        }
        os << ")";
    }
    os << "]";
    return os;
}

// Type aliases
using MatDynamicf = MatDynamic<float>;
using MatDynamicd = MatDynamic<double>;

}  // namespace phynity::math::matrices

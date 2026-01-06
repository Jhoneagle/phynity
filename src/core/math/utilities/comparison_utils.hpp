#pragma once

#include <core/math/matrices/mat2.hpp>
#include <core/math/matrices/mat3.hpp>
#include <core/math/matrices/mat4.hpp>
#include <core/math/matrices/mat_n.hpp>
#include <core/math/matrices/mat_dynamic.hpp>
#include <core/math/vectors/vec2.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/math/vectors/vec4.hpp>
#include <core/math/vectors/vec_n.hpp>
#include <core/math/vectors/vec_dynamic.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <cmath>
#include <cstddef>

namespace phynity::math::utilities {

using phynity::math::matrices::Mat2;
using phynity::math::matrices::Mat3;
using phynity::math::matrices::Mat4;
using phynity::math::matrices::MatN;
using phynity::math::matrices::MatDynamic;
using phynity::math::vectors::Vec2;
using phynity::math::vectors::Vec3;
using phynity::math::vectors::Vec4;
using phynity::math::vectors::VecN;
using phynity::math::vectors::VecDynamic;
using phynity::math::quaternions::Quat;

// ============================================================================
// Validity checks for matrices (NaN/Inf detection)
// ============================================================================

template<typename T = float>
inline bool hasNaN(const Mat3<T>& m) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (std::isnan(m(i, j))) {
                return true;
            }
        }
    }
    return false;
}

template<typename T = float>
inline bool hasInf(const Mat3<T>& m) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (std::isinf(m(i, j))) {
                return true;
            }
        }
    }
    return false;
}

// ============================================================================
// Approximate equality for vectors
// ============================================================================

template<typename T>
inline bool approx_equal(const Vec2<T>& a, const Vec2<T>& b, T tolerance = epsilon<T>()) {
    return equals(a.x, b.x, tolerance) && 
           equals(a.y, b.y, tolerance);
}

template<typename T>
inline bool approx_equal(const Vec3<T>& a, const Vec3<T>& b, T tolerance = epsilon<T>()) {
    return equals(a.x, b.x, tolerance) && 
           equals(a.y, b.y, tolerance) && 
           equals(a.z, b.z, tolerance);
}

template<typename T>
inline bool approx_equal(const Vec4<T>& a, const Vec4<T>& b, T tolerance = epsilon<T>()) {
    return equals(a.x, b.x, tolerance) && 
           equals(a.y, b.y, tolerance) && 
           equals(a.z, b.z, tolerance) && 
           equals(a.w, b.w, tolerance);
}

template<std::size_t N, typename T>
inline bool approx_equal(const VecN<N, T>& a, const VecN<N, T>& b, T tolerance = epsilon<T>()) {
    for (std::size_t i = 0; i < N; ++i) {
        if (!equals(a[i], b[i], tolerance)) {
            return false;
        }
    }
    return true;
}

template<typename T>
inline bool approx_equal(const VecDynamic<T>& a, const VecDynamic<T>& b, T tolerance = epsilon<T>()) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!equals(a[i], b[i], tolerance)) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// Approximate equality for matrices
// ============================================================================

template<typename T>
inline bool approx_equal(const Mat2<T>& a, const Mat2<T>& b, T tolerance = epsilon<T>()) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            if (!equals(a(i, j), b(i, j), tolerance)) {
                return false;
            }
        }
    }
    return true;
}

template<typename T>
inline bool approx_equal(const Mat3<T>& a, const Mat3<T>& b, T tolerance = epsilon<T>()) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!equals(a(i, j), b(i, j), tolerance)) {
                return false;
            }
        }
    }
    return true;
}

template<typename T>
inline bool approx_equal(const Mat4<T>& a, const Mat4<T>& b, T tolerance = epsilon<T>()) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!equals(a(i, j), b(i, j), tolerance)) {
                return false;
            }
        }
    }
    return true;
}

template<std::size_t M, std::size_t N, typename T>
inline bool approx_equal(const MatN<M, N, T>& a, const MatN<M, N, T>& b, T tolerance = epsilon<T>()) {
    for (std::size_t i = 0; i < M; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            if (!equals(a(i, j), b(i, j), tolerance)) {
                return false;
            }
        }
    }
    return true;
}

template<typename T>
inline bool approx_equal(const MatDynamic<T>& a, const MatDynamic<T>& b, T tolerance = epsilon<T>()) {
    if (a.numRows() != b.numRows() || a.numCols() != b.numCols()) {
        return false;
    }
    for (std::size_t i = 0; i < a.numRows(); ++i) {
        for (std::size_t j = 0; j < a.numCols(); ++j) {
            if (!equals(a(i, j), b(i, j), tolerance)) {
                return false;
            }
        }
    }
    return true;
}

// ============================================================================
// Approximate equality for quaternions
// ============================================================================

template<typename T>
inline bool approx_equal(const Quat<T>& a, const Quat<T>& b, T tolerance = epsilon<T>()) {
    return equals(a.w, b.w, tolerance) && 
           equals(a.x, b.x, tolerance) && 
           equals(a.y, b.y, tolerance) && 
           equals(a.z, b.z, tolerance);
}

/// Compare two quaternions accounting for double-cover (q and -q represent same rotation)
template<typename T>
inline bool approx_equal_rotation(const Quat<T>& a, const Quat<T>& b, T tolerance = epsilon<T>()) {
    // Check if they're the same
    bool same = equals(a.w, b.w, tolerance) && 
                equals(a.x, b.x, tolerance) && 
                equals(a.y, b.y, tolerance) && 
                equals(a.z, b.z, tolerance);
    
    // Check if they're negatives (double-cover: q and -q are equivalent rotations)
    bool opposite = equals(a.w, -b.w, tolerance) && 
                    equals(a.x, -b.x, tolerance) && 
                    equals(a.y, -b.y, tolerance) && 
                    equals(a.z, -b.z, tolerance);
    
    return same || opposite;
}

// ============================================================================
// Rotation matrix validation
// ============================================================================

/// Verify that a 3x3 matrix is a valid rotation matrix (orthonormal with det=+1)
template<typename T>
inline bool is_valid_rotation_matrix(const Mat3<T>& m, T tolerance = epsilon<T>() * T(10)) {
    // Check orthonormality: rows should be orthogonal
    Vec3<T> row0 = m.getRow(0);
    Vec3<T> row1 = m.getRow(1);
    Vec3<T> row2 = m.getRow(2);
    
    if (!equals(row0.dot(row1), T(0), tolerance)) return false;
    if (!equals(row1.dot(row2), T(0), tolerance)) return false;
    if (!equals(row2.dot(row0), T(0), tolerance)) return false;
    
    // Check that rows are unit length
    if (!equals(row0.length(), T(1), tolerance)) return false;
    if (!equals(row1.length(), T(1), tolerance)) return false;
    if (!equals(row2.length(), T(1), tolerance)) return false;
    
    // Check determinant is +1 (not -1, which would be a reflection)
    T det = m.determinant();
    if (!equals(det, T(1), tolerance)) return false;
    
    return true;
}

}  // namespace phynity::math::utilities

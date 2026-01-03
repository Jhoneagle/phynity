#pragma once

#include <core/math/matrices/mat3.hpp>
#include <cmath>

namespace phynity::math::utilities {

using phynity::math::matrices::Mat3;

inline static bool hasNaN(const Mat3& m) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (std::isnan(m.m[i][j])) {
                return true;
            }
        }
    }

    return false;
}

inline static bool hasInf(const Mat3& m) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (std::isinf(m.m[i][j])) {
                return true;
            }
        }
    }

    return false;
}

}
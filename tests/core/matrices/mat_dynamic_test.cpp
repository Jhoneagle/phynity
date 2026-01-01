#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat_dynamic.hpp>
#include <cmath>
#include <vector>
#include <stdexcept>

using phynity::math::matrices::MatDynamic;
using phynity::math::vectors::VecDynamic;
using Catch::Matchers::WithinAbs;

bool matdyn_approx_equal(const MatDynamic& a, const MatDynamic& b, float eps = 1e-5f) {
    if (a.rows != b.rows || a.cols != b.cols) return false;
    for (std::size_t i = 0; i < a.rows; ++i) {
        for (std::size_t j = 0; j < a.cols; ++j) {
            if (std::abs(a(i, j) - b(i, j)) > eps) return false;
        }
    }
    return true;
}

TEST_CASE("MatDynamic: constructors", "[MatDynamic][constructor]") {
    SECTION("Zero by default") {
        MatDynamic m(2, 3);
        for (std::size_t i = 0; i < 2; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                REQUIRE_THAT(m(i, j), WithinAbs(0.0f, 1e-6f));
            }
        }
    }

    SECTION("Scalar fill") {
        MatDynamic m(2, 2, 4.0f);
        REQUIRE_THAT(m(0, 0), WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(4.0f, 1e-6f));
    }
}

TEST_CASE("MatDynamic: factories", "[MatDynamic][factory]") {
    MatDynamic zero = MatDynamic::zero(3, 2);
    MatDynamic eye = MatDynamic::identity(3);

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 2; ++j) {
            REQUIRE_THAT(zero(i, j), WithinAbs(0.0f, 1e-6f));
        }
    }

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            float expected = (i == j) ? 1.0f : 0.0f;
            REQUIRE_THAT(eye(i, j), WithinAbs(expected, 1e-6f));
        }
    }
}

TEST_CASE("MatDynamic: checked access", "[MatDynamic][access]") {
    MatDynamic m(2, 2);
    m.at(1, 1) = 4.0f;
    REQUIRE_THAT(m.at(1, 1), WithinAbs(4.0f, 1e-6f));
    REQUIRE_THROWS_AS(m.at(2, 0), std::out_of_range);
    REQUIRE_THROWS_AS(m.at(0, 2), std::out_of_range);
}

TEST_CASE("MatDynamic: arithmetic", "[MatDynamic][arithmetic]") {
    MatDynamic a(2, 2, 1.0f);
    MatDynamic b(2, 2, 2.0f);

    SECTION("Add/Sub") {
        MatDynamic c = a + b;
        MatDynamic d = c - a;
        REQUIRE_THAT(c(0, 0), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(d(1, 1), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Scalar multiply/divide") {
        MatDynamic c = b * 0.5f;
        MatDynamic d = b / 2.0f;
        REQUIRE_THAT(c(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(d(1, 1), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Compound assign") {
        MatDynamic c = a;
        c += a;
        c -= a;
        c *= 3.0f;
        c /= 3.0f;
        REQUIRE(matdyn_approx_equal(c, a));
    }
}

TEST_CASE("MatDynamic: shape mismatch throws", "[MatDynamic][errors]") {
    MatDynamic a(2, 2, 1.0f);
    MatDynamic b(3, 3, 1.0f);

    REQUIRE_THROWS_AS(a + b, std::invalid_argument);
    REQUIRE_THROWS_AS(a - b, std::invalid_argument);
    REQUIRE_THROWS_AS(a * b, std::invalid_argument);
}

TEST_CASE("MatDynamic: matrix multiplication", "[MatDynamic][multiply]") {
    MatDynamic a(2, 3);
    MatDynamic b(3, 2);
    // Fill A
    float val = 1.0f;
    for (std::size_t i = 0; i < 2; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            a(i, j) = val++;
        }
    }
    // Fill B
    val = 7.0f;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 2; ++j) {
            b(i, j) = val++;
        }
    }

    MatDynamic result = a * b;
    MatDynamic expected(2, 2);
    expected(0, 0) = 58.0f;
    expected(0, 1) = 64.0f;
    expected(1, 0) = 139.0f;
    expected(1, 1) = 154.0f;

    REQUIRE(matdyn_approx_equal(result, expected));
}

TEST_CASE("MatDynamic: matrix-vector multiplication", "[MatDynamic][vector]") {
    MatDynamic m(2, 3);
    m(0, 0) = 1.0f; m(0, 1) = 2.0f; m(0, 2) = 3.0f;
    m(1, 0) = 4.0f; m(1, 1) = 5.0f; m(1, 2) = 6.0f;

    VecDynamic v(3, 1.0f);
    VecDynamic r = m * v;

    REQUIRE_THAT(r[0], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(r[1], WithinAbs(15.0f, 1e-6f));

    VecDynamic wrong(2, 1.0f);
    REQUIRE_THROWS_AS(m * wrong, std::invalid_argument);
}

TEST_CASE("MatDynamic: transpose", "[MatDynamic][transpose]") {
    MatDynamic m(2, 3);
    m(0, 1) = 2.0f;
    m(1, 2) = 3.0f;

    MatDynamic t = m.transposed();
    REQUIRE_THAT(t(1, 0), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(t(2, 1), WithinAbs(3.0f, 1e-6f));

    m.transpose();
    REQUIRE_THAT(m(1, 0), WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("MatDynamic: equality", "[MatDynamic][comparison]") {
    MatDynamic a(2, 2, 1.0f);
    MatDynamic b(2, 2, 1.0f);
    MatDynamic c(2, 2, 2.0f);

    REQUIRE(a == b);
    REQUIRE(a != c);
}

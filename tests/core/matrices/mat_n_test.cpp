#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat_n.hpp>
#include <array>
#include <cmath>

using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;
using Catch::Matchers::WithinAbs;

// Helper for approximate equality on MatN
template <std::size_t M, std::size_t N>
bool matn_approx_equal(const MatN<M, N>& a, const MatN<M, N>& b, float eps = 1e-5f) {
    for (std::size_t i = 0; i < M; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            if (std::abs(a(i, j) - b(i, j)) > eps) {
                return false;
            }
        }
    }
    return true;
}

TEST_CASE("MatN: constructors", "[MatN][constructor]") {
    SECTION("Default square is identity") {
        MatN<3, 3> m;
        REQUIRE(matn_approx_equal(m, MatN<3, 3>::identity()));
    }

    SECTION("Default non-square is zero") {
        MatN<2, 3> m;
        for (std::size_t i = 0; i < 2; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                REQUIRE_THAT(m(i, j), WithinAbs(0.0f, 1e-6f));
            }
        }
    }

    SECTION("Scalar fill") {
        MatN<2, 2> m(2.5f);
        for (std::size_t i = 0; i < 2; ++i) {
            for (std::size_t j = 0; j < 2; ++j) {
                REQUIRE_THAT(m(i, j), WithinAbs(2.5f, 1e-6f));
            }
        }
    }

    SECTION("From array") {
        MatN<2, 2> m(std::array<float, 4>{1.0f, 2.0f, 3.0f, 4.0f});
        REQUIRE_THAT(m(0, 1), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(3.0f, 1e-6f));
    }
}

TEST_CASE("MatN: factories", "[MatN][factory]") {
    MatN<3, 3> eye = MatN<3, 3>::identity();
    MatN<3, 3> zero = MatN<3, 3>::zero();

    REQUIRE(matn_approx_equal(eye, MatN<3, 3>::identity()));
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            REQUIRE_THAT(zero(i, j), WithinAbs(0.0f, 1e-6f));
        }
    }
}

TEST_CASE("MatN: trace", "[MatN][trace]") {
    MatN<3, 3> eye = MatN<3, 3>::identity();
    MatN<3, 3> zero = MatN<3, 3>::zero();
    REQUIRE_THAT(eye.trace(), WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(zero.trace(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("MatN: arithmetic", "[MatN][arithmetic]") {
    MatN<3, 3> a = MatN<3, 3>::identity();
    MatN<3, 3> b(2.0f);

    SECTION("Addition and subtraction") {
        MatN<3, 3> c = a + b;
        MatN<3, 3> d = c - a;
        REQUIRE_THAT(c(0, 0), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(d(1, 1), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Scalar multiply/divide") {
        MatN<3, 3> c = b * 0.5f;
        MatN<3, 3> d = b / 2.0f;
        REQUIRE_THAT(c(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(d(2, 2), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Compound assignments") {
        MatN<3, 3> c = a;
        c += a;
        c -= a;
        c *= 3.0f;
        c /= 3.0f;
        REQUIRE(matn_approx_equal(c, a));
    }
}

TEST_CASE("MatN: matrix multiplication", "[MatN][multiply]") {
    MatN<2, 3> a(std::array<float, 6>{1, 2, 3, 4, 5, 6});
    MatN<3, 2> b(std::array<float, 6>{7, 8, 9, 10, 11, 12});
    MatN<2, 2> expected(std::array<float, 4>{58, 64, 139, 154});

    MatN<2, 2> result = a * b;
    REQUIRE(matn_approx_equal(result, expected));
}

TEST_CASE("MatN: determinant and inverse (small N)", "[MatN][determinant][inverse]") {
    SECTION("2x2 determinant") {
        MatN<2, 2> m(std::array<float, 4>{1, 2, 3, 4});
        REQUIRE_THAT(m.determinant(), WithinAbs(-2.0f, 1e-5f));
    }

    SECTION("3x3 determinant") {
        MatN<3, 3> m(std::array<float, 9>{
            2, 0, 0,
            0, 3, 0,
            0, 0, 4});
        REQUIRE_THAT(m.determinant(), WithinAbs(24.0f, 1e-5f));
    }

    SECTION("4x4 determinant") {
        MatN<4, 4> m(std::array<float, 16>{
            2, 0, 0, 0,
            0, 3, 0, 0,
            0, 0, 4, 0,
            0, 0, 0, 5});
        REQUIRE_THAT(m.determinant(), WithinAbs(120.0f, 1e-4f));
    }

    SECTION("2x2 inverse") {
        MatN<2, 2> m(std::array<float, 4>{4, 7, 2, 6});
        MatN<2, 2> inv = m.inverse();
        MatN<2, 2> product = m * inv;
        REQUIRE(matn_approx_equal(product, MatN<2, 2>::identity(), 1e-3f));
    }

    SECTION("Singular returns zero") {
        MatN<2, 2> m(std::array<float, 4>{1, 2, 2, 4});
        MatN<2, 2> inv = m.inverse();
        REQUIRE(matn_approx_equal(inv, MatN<2, 2>::zero(), 1e-6f));
    }
}

TEST_CASE("MatN: minor and cofactor", "[MatN][minor][cofactor]") {
    MatN<3, 3> m(std::array<float, 9>{
        1, 2, 3,
        0, 4, 5,
        1, 0, 6});

    REQUIRE_THAT(m.minor(0, 0), WithinAbs(24.0f, 1e-5f));
    REQUIRE_THAT(m.minor(0, 1), WithinAbs(-5.0f, 1e-5f));
    REQUIRE_THAT(m.minor(1, 0), WithinAbs(12.0f, 1e-5f));

    MatN<3, 3> cof = m.cofactor();
    REQUIRE_THAT(cof(0, 0), WithinAbs(24.0f, 1e-5f));
    REQUIRE_THAT(cof(0, 1), WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(cof(1, 0), WithinAbs(-12.0f, 1e-5f));
}

TEST_CASE("MatN: matrix-vector multiplication", "[MatN][vector]") {
    MatN<3, 3> m = MatN<3, 3>::identity();
    VecN<3> v(1.0f);
    VecN<3> r = m * v;
    REQUIRE(r == v);
}

TEST_CASE("MatN: transpose", "[MatN][transpose]") {
    MatN<2, 3> m(std::array<float, 6>{1, 2, 3, 4, 5, 6});
    MatN<3, 2> t = m.transposed();

    REQUIRE_THAT(t(0, 1), WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(t(2, 0), WithinAbs(3.0f, 1e-6f));

    MatN<3, 3> square = MatN<3, 3>::identity();
    square(0, 1) = 2.0f;
    square.transpose();
    REQUIRE_THAT(square(1, 0), WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("MatN: element access", "[MatN][access]") {
    MatN<2, 2> m;
    m(0, 1) = 3.5f;
    REQUIRE_THAT(m(0, 1), WithinAbs(3.5f, 1e-6f));
}

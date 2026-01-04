#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat_n.hpp>
#include <array>
#include <cmath>
#include <sstream>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::VecN;
using phynity::math::matrices::MatN;

// Helper for approximate equality on MatN
template <std::size_t M, std::size_t N>
bool matn_approx_equal(const phynity::math::matrices::MatN<M, N, float>& a, const phynity::math::matrices::MatN<M, N, float>& b, float eps = 1e-5f) {
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
    SECTION("Default square is zero") {
        phynity::math::matrices::MatN<3, 3> m;
        REQUIRE(matn_approx_equal(m, phynity::math::matrices::MatN<3, 3>::zero()));
    }

    SECTION("Default non-square is zero") {
        phynity::math::matrices::MatN<2, 3> m;
        for (std::size_t i = 0; i < 2; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                REQUIRE_THAT(m(i, j), WithinAbs(0.0f, 1e-6f));
            }
        }
    }

    SECTION("Scalar fill") {
        phynity::math::matrices::MatN<2, 2> m(2.5f);
        for (std::size_t i = 0; i < 2; ++i) {
            for (std::size_t j = 0; j < 2; ++j) {
                REQUIRE_THAT(m(i, j), WithinAbs(2.5f, 1e-6f));
            }
        }
    }

    SECTION("From array") {
        phynity::math::matrices::MatN<2, 2> m(std::array<float, 4>{1.0f, 2.0f, 3.0f, 4.0f});
        REQUIRE_THAT(m(0, 1), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(3.0f, 1e-6f));
    }
}

TEST_CASE("MatN: factories", "[MatN][factory]") {
    phynity::math::matrices::MatN<3, 3> eye = phynity::math::matrices::MatN<3, 3>::identity();
    phynity::math::matrices::MatN<3, 3> zero = phynity::math::matrices::MatN<3, 3>::zero();

    REQUIRE(matn_approx_equal(eye, phynity::math::matrices::MatN<3, 3>::identity()));
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            REQUIRE_THAT(zero(i, j), WithinAbs(0.0f, 1e-6f));
        }
    }
}

TEST_CASE("MatN: trace", "[MatN][trace]") {
    phynity::math::matrices::MatN<3, 3> eye = phynity::math::matrices::MatN<3, 3>::identity();
    phynity::math::matrices::MatN<3, 3> zero = phynity::math::matrices::MatN<3, 3>::zero();
    REQUIRE_THAT(eye.trace(), WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(zero.trace(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("MatN: arithmetic", "[MatN][arithmetic]") {
    phynity::math::matrices::MatN<3, 3> a = phynity::math::matrices::MatN<3, 3>::identity();
    phynity::math::matrices::MatN<3, 3> b(2.0f);

    SECTION("Addition and subtraction") {
        phynity::math::matrices::MatN<3, 3> c = a + b;
        phynity::math::matrices::MatN<3, 3> d = c - a;
        REQUIRE_THAT(c(0, 0), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(d(1, 1), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Scalar multiply/divide") {
        phynity::math::matrices::MatN<3, 3> c = b * 0.5f;
        phynity::math::matrices::MatN<3, 3> d = b / 2.0f;
        REQUIRE_THAT(c(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(d(2, 2), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Compound assignments") {
        phynity::math::matrices::MatN<3, 3> c = a;
        c += a;
        c -= a;
        c *= 3.0f;
        c /= 3.0f;
        REQUIRE(matn_approx_equal(c, a));
    }

    SECTION("Negation") {
        phynity::math::matrices::MatN<2, 2> m = phynity::math::matrices::MatN<2, 2>::identity();
        m(0, 1) = -3.0f;
        phynity::math::matrices::MatN<2, 2> result = -m;
        
        REQUIRE_THAT(result(0, 0), WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(result(0, 1), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result(1, 1), WithinAbs(-1.0f, 1e-6f));
        
        // Double negation
        phynity::math::matrices::MatN<2, 2> double_neg = -(-m);
        REQUIRE_THAT(double_neg(0, 0), WithinAbs(m(0, 0), 1e-6f));
        REQUIRE_THAT(double_neg(0, 1), WithinAbs(m(0, 1), 1e-6f));
    }
}

TEST_CASE("MatN: Approximate equality", "[MatN][comparison]") {
    phynity::math::matrices::MatN<2, 2> m1 = phynity::math::matrices::MatN<2, 2>::identity();
    phynity::math::matrices::MatN<2, 2> m2 = phynity::math::matrices::MatN<2, 2>::identity();
    phynity::math::matrices::MatN<2, 2> m3 = phynity::math::matrices::MatN<2, 2>::identity();
    m3(0, 0) = 1.000001f;
    phynity::math::matrices::MatN<2, 2> m4 = phynity::math::matrices::MatN<2, 2>::zero();

    REQUIRE(m1.approxEqual(m2));
    REQUIRE(m1.approxEqual(m3, 1e-5f));
    REQUIRE_FALSE(m1.approxEqual(m4, 1e-6f));
    REQUIRE(m1.approxEqual(m4, 2.0f));
}

TEST_CASE("MatN: Absolute value", "[MatN][operations]") {
    phynity::math::matrices::MatN<2, 3> m;
    m(0, 0) = -1.0f; m(0, 1) = 2.0f;  m(0, 2) = -3.0f;
    m(1, 0) = 4.0f;  m(1, 1) = -5.0f; m(1, 2) = 6.0f;

    phynity::math::matrices::MatN<2, 3> result = m.abs();

    REQUIRE_THAT(result(0, 0), WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result(0, 1), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result(0, 2), WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result(1, 0), WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result(1, 1), WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result(1, 2), WithinAbs(6.0f, 1e-6f));
}

TEST_CASE("MatN: Component-wise operations", "[MatN][arithmetic]") {
    phynity::math::matrices::MatN<2, 2> m1;
    m1(0, 0) = 4.0f; m1(0, 1) = 6.0f;
    m1(1, 0) = 8.0f; m1(1, 1) = 10.0f;
    phynity::math::matrices::MatN<2, 2> m2;
    m2(0, 0) = 2.0f; m2(0, 1) = 3.0f;
    m2(1, 0) = 4.0f; m2(1, 1) = 5.0f;

    SECTION("Component-wise multiplication") {
        phynity::math::matrices::MatN<2, 2> m = m1;
        m.mulComponentWise(m2);
        REQUIRE_THAT(m(0, 0), WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(m(0, 1), WithinAbs(18.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(32.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(50.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        phynity::math::matrices::MatN<2, 2> m = m1;
        m.divComponentWise(m2);
        REQUIRE_THAT(m(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(0, 1), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("MatN: matrix multiplication", "[MatN][multiply]") {
    phynity::math::matrices::MatN<2, 3> a(std::array<float, 6>{1, 2, 3, 4, 5, 6});
    phynity::math::matrices::MatN<3, 2> b(std::array<float, 6>{7, 8, 9, 10, 11, 12});
    phynity::math::matrices::MatN<2, 2> expected(std::array<float, 4>{58, 64, 139, 154});

    phynity::math::matrices::MatN<2, 2> result = a * b;
    REQUIRE(matn_approx_equal(result, expected));
}

TEST_CASE("MatN: determinant and inverse (small N)", "[MatN][determinant][inverse]") {
    SECTION("2x2 determinant") {
        phynity::math::matrices::MatN<2, 2> m(std::array<float, 4>{1, 2, 3, 4});
        REQUIRE_THAT(m.determinant(), WithinAbs(-2.0f, 1e-5f));
    }

    SECTION("3x3 determinant") {
        phynity::math::matrices::MatN<3, 3> m(std::array<float, 9>{
            2, 0, 0,
            0, 3, 0,
            0, 0, 4});
        REQUIRE_THAT(m.determinant(), WithinAbs(24.0f, 1e-5f));
    }

    SECTION("4x4 determinant") {
        phynity::math::matrices::MatN<4, 4> m(std::array<float, 16>{
            2, 0, 0, 0,
            0, 3, 0, 0,
            0, 0, 4, 0,
            0, 0, 0, 5});
        REQUIRE_THAT(m.determinant(), WithinAbs(120.0f, 1e-4f));
    }

    SECTION("2x2 inverse") {
        phynity::math::matrices::MatN<2, 2> m(std::array<float, 4>{4, 7, 2, 6});
        phynity::math::matrices::MatN<2, 2> inv = m.inverse();
        phynity::math::matrices::MatN<2, 2> product = m * inv;
        REQUIRE(matn_approx_equal(product, phynity::math::matrices::MatN<2, 2>::identity(), 1e-3f));
    }

    SECTION("Singular returns zero") {
        phynity::math::matrices::MatN<2, 2> m(std::array<float, 4>{1, 2, 2, 4});
        phynity::math::matrices::MatN<2, 2> inv = m.inverse();
        REQUIRE(matn_approx_equal(inv, phynity::math::matrices::MatN<2, 2>::zero(), 1e-6f));
    }
}

TEST_CASE("MatN: minor and cofactor", "[MatN][minor][cofactor]") {
    phynity::math::matrices::MatN<3, 3> m(std::array<float, 9>{
        1, 2, 3,
        0, 4, 5,
        1, 0, 6});

    REQUIRE_THAT(m.minor(0, 0), WithinAbs(24.0f, 1e-5f));
    REQUIRE_THAT(m.minor(0, 1), WithinAbs(-5.0f, 1e-5f));
    REQUIRE_THAT(m.minor(1, 0), WithinAbs(12.0f, 1e-5f));

    phynity::math::matrices::MatN<3, 3> cof = m.cofactor();
    REQUIRE_THAT(cof(0, 0), WithinAbs(24.0f, 1e-5f));
    REQUIRE_THAT(cof(0, 1), WithinAbs(5.0f, 1e-5f));
    REQUIRE_THAT(cof(1, 0), WithinAbs(-12.0f, 1e-5f));
}

TEST_CASE("MatN: matrix-vector multiplication", "[MatN][vector]") {
    phynity::math::matrices::MatN<3, 3> m = phynity::math::matrices::MatN<3, 3>::identity();
    VecN<3> v(1.0f);
    VecN<3> r = m * v;
    REQUIRE(r == v);
}

TEST_CASE("MatN: transpose", "[MatN][transpose]") {
    phynity::math::matrices::MatN<2, 3> m(std::array<float, 6>{1, 2, 3, 4, 5, 6});
    phynity::math::matrices::MatN<3, 2> t = m.transposed();

    REQUIRE_THAT(t(0, 1), WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(t(2, 0), WithinAbs(3.0f, 1e-6f));

    phynity::math::matrices::MatN<3, 3> square = phynity::math::matrices::MatN<3, 3>::identity();
    square(0, 1) = 2.0f;
    square.transpose();
    REQUIRE_THAT(square(1, 0), WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("MatN: element access", "[MatN][access]") {
    phynity::math::matrices::MatN<2, 2> m;
    m(0, 1) = 3.5f;
    REQUIRE_THAT(m(0, 1), WithinAbs(3.5f, 1e-6f));
}

TEST_CASE("MatN: row and column accessors", "[MatN][access][rowcol]") {
    phynity::math::matrices::MatN<2, 3> m(std::array<float, 6>{1, 2, 3, 4, 5, 6});

    VecN<3> row0 = m.getRow(0);
    REQUIRE_THAT(row0[0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(row0[2], WithinAbs(3.0f, 1e-6f));

    VecN<2> col1 = m.getColumn(1);
    REQUIRE_THAT(col1[0], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(col1[1], WithinAbs(5.0f, 1e-6f));

    VecN<3> newRow{};
    newRow[0] = 7.0f; newRow[1] = 8.0f; newRow[2] = 9.0f;
    m.setRow(0, newRow);
    REQUIRE_THAT(m(0, 2), WithinAbs(9.0f, 1e-6f));

    VecN<2> newCol{};
    newCol[0] = 10.0f; newCol[1] = 11.0f;
    m.setColumn(1, newCol);
    REQUIRE_THAT(m(1, 1), WithinAbs(11.0f, 1e-6f));
}

TEST_CASE("MatN: stream output", "[MatN][stream]") {
    phynity::math::matrices::MatN<2, 2> m(std::array<float, 4>{1.0f, 2.0f, 3.0f, 4.0f});
    std::ostringstream oss;
    oss << m;
    REQUIRE(oss.str() == "[(1, 2), (3, 4)]");
}

TEST_CASE("MatN: Edge cases", "[MatN][edge]") {
    SECTION("Division by zero scalar") {
        phynity::math::matrices::MatN<2, 2> m = phynity::math::matrices::MatN<2, 2>::identity();
        phynity::math::matrices::MatN<2, 2> result = m / 0.0f;
        REQUIRE((std::isinf(result(0, 0)) || std::isnan(result(0, 0))));
    }

    SECTION("Very large numbers") {
        phynity::math::matrices::MatN<3, 3> m(1e20f);
        float trace = m.trace();
        REQUIRE(trace > 1e20f);
    }

    SECTION("Very small numbers") {
        phynity::math::matrices::MatN<2, 2> m(1e-20f);
        phynity::math::matrices::MatN<2, 2> scaled = m * 2.0f;
        REQUIRE(scaled(0, 0) > 0.0f);
    }

    SECTION("Non-square matrix operations") {
        phynity::math::matrices::MatN<2, 3> m(1.0f);
        phynity::math::matrices::MatN<2, 3> sum = m + m;
        REQUIRE_THAT(sum(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(sum(1, 2), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Large dimension matrix") {
        phynity::math::matrices::MatN<10, 10> m = phynity::math::matrices::MatN<10, 10>::identity();
        float trace = m.trace();
        REQUIRE_THAT(trace, WithinAbs(10.0f, 1e-6f));
    }

    SECTION("Transpose of non-square matrix") {
        phynity::math::matrices::MatN<2, 3> m(1.0f);
        m(0, 0) = 1.0f; m(0, 1) = 2.0f; m(0, 2) = 3.0f;
        m(1, 0) = 4.0f; m(1, 1) = 5.0f; m(1, 2) = 6.0f;
        
        phynity::math::matrices::MatN<3, 2> t = m.transposed();
        REQUIRE_THAT(t(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(t(1, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(t(2, 1), WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Zero matrix determinant") {
        phynity::math::matrices::MatN<3, 3> m = phynity::math::matrices::MatN<3, 3>::zero();
        float det = m.determinant();
        REQUIRE_THAT(det, WithinAbs(0.0f, 1e-6f));
    }
}

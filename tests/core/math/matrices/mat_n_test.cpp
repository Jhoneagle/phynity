#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat_n.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <array>
#include <cmath>
#include <sstream>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::VecN;
using phynity::math::matrices::MatN;
using phynity::math::utilities::approx_equal;

TEST_CASE("MatN: constructors", "[MatN][constructor]") {
    SECTION("Default square is zero") {
        MatN<3, 3> m;
        REQUIRE(approx_equal(m, MatN<3, 3>::zero()));
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

    REQUIRE(approx_equal(eye, MatN<3, 3>::identity()));
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
        REQUIRE(approx_equal(c, a));
    }

    SECTION("Negation") {
        MatN<2, 2> m = MatN<2, 2>::identity();
        m(0, 1) = -3.0f;
        MatN<2, 2> result = -m;
        
        REQUIRE_THAT(result(0, 0), WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(result(0, 1), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result(1, 1), WithinAbs(-1.0f, 1e-6f));
        
        // Double negation
        MatN<2, 2> double_neg = -(-m);
        REQUIRE_THAT(double_neg(0, 0), WithinAbs(m(0, 0), 1e-6f));
        REQUIRE_THAT(double_neg(0, 1), WithinAbs(m(0, 1), 1e-6f));
    }
}

TEST_CASE("MatN: Approximate equality", "[MatN][comparison]") {
    MatN<2, 2> m1 = MatN<2, 2>::identity();
    MatN<2, 2> m2 = MatN<2, 2>::identity();
    MatN<2, 2> m3 = MatN<2, 2>::identity();
    m3(0, 0) = 1.000001f;
    MatN<2, 2> m4 = MatN<2, 2>::zero();

    REQUIRE(m1.approxEqual(m2));
    REQUIRE(m1.approxEqual(m3, 1e-5f));
    REQUIRE_FALSE(m1.approxEqual(m4, 1e-6f));
    REQUIRE(m1.approxEqual(m4, 2.0f));
}

TEST_CASE("MatN: Absolute value", "[MatN][operations]") {
    MatN<2, 3> m;
    m(0, 0) = -1.0f; m(0, 1) = 2.0f;  m(0, 2) = -3.0f;
    m(1, 0) = 4.0f;  m(1, 1) = -5.0f; m(1, 2) = 6.0f;

    MatN<2, 3> result = m.abs();

    REQUIRE_THAT(result(0, 0), WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result(0, 1), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result(0, 2), WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result(1, 0), WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result(1, 1), WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result(1, 2), WithinAbs(6.0f, 1e-6f));
}

TEST_CASE("MatN: Component-wise operations", "[MatN][arithmetic]") {
    MatN<2, 2> m1;
    m1(0, 0) = 4.0f; m1(0, 1) = 6.0f;
    m1(1, 0) = 8.0f; m1(1, 1) = 10.0f;
    MatN<2, 2> m2;
    m2(0, 0) = 2.0f; m2(0, 1) = 3.0f;
    m2(1, 0) = 4.0f; m2(1, 1) = 5.0f;

    SECTION("Component-wise multiplication") {
        MatN<2, 2> m = m1;
        m.mulComponentWise(m2);
        REQUIRE_THAT(m(0, 0), WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(m(0, 1), WithinAbs(18.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(32.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(50.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        MatN<2, 2> m = m1;
        m.divComponentWise(m2);
        REQUIRE_THAT(m(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(0, 1), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("MatN: matrix multiplication", "[MatN][multiply]") {
    MatN<2, 3> a(std::array<float, 6>{1, 2, 3, 4, 5, 6});
    MatN<3, 2> b(std::array<float, 6>{7, 8, 9, 10, 11, 12});
    MatN<2, 2> expected(std::array<float, 4>{58, 64, 139, 154});

    MatN<2, 2> result = a * b;
    REQUIRE(approx_equal(result, expected));
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
        REQUIRE(approx_equal(product, MatN<2, 2>::identity(), 1e-3f));
    }

    SECTION("Singular returns zero") {
        MatN<2, 2> m(std::array<float, 4>{1, 2, 2, 4});
        MatN<2, 2> inv = m.inverse();
        REQUIRE(approx_equal(inv, MatN<2, 2>::zero(), 1e-6f));
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

TEST_CASE("MatN: row and column accessors", "[MatN][access][rowcol]") {
    MatN<2, 3> m(std::array<float, 6>{1, 2, 3, 4, 5, 6});

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
    MatN<2, 2> m(std::array<float, 4>{1.0f, 2.0f, 3.0f, 4.0f});
    std::ostringstream oss;
    oss << m;
    REQUIRE(oss.str() == "[(1, 2), (3, 4)]");
}

TEST_CASE("MatN: Edge cases", "[MatN][edge]") {
    SECTION("Division by zero scalar") {
        MatN<2, 2> m = MatN<2, 2>::identity();
        MatN<2, 2> result = m / 0.0f;
        REQUIRE((std::isinf(result(0, 0)) || std::isnan(result(0, 0))));
    }

    SECTION("Very large numbers") {
        MatN<3, 3> m(1e20f);
        float trace = m.trace();
        REQUIRE(trace > 1e20f);
    }

    SECTION("Very small numbers") {
        MatN<2, 2> m(1e-20f);
        MatN<2, 2> scaled = m * 2.0f;
        REQUIRE(scaled(0, 0) > 0.0f);
    }

    SECTION("Non-square matrix operations") {
        MatN<2, 3> m(1.0f);
        MatN<2, 3> sum = m + m;
        REQUIRE_THAT(sum(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(sum(1, 2), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Large dimension matrix") {
        MatN<10, 10> m = MatN<10, 10>::identity();
        float trace = m.trace();
        REQUIRE_THAT(trace, WithinAbs(10.0f, 1e-6f));
    }

    SECTION("Transpose of non-square matrix") {
        MatN<2, 3> m(1.0f);
        m(0, 0) = 1.0f; m(0, 1) = 2.0f; m(0, 2) = 3.0f;
        m(1, 0) = 4.0f; m(1, 1) = 5.0f; m(1, 2) = 6.0f;
        
        MatN<3, 2> t = m.transposed();
        REQUIRE_THAT(t(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(t(1, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(t(2, 1), WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Zero matrix determinant") {
        MatN<3, 3> m = MatN<3, 3>::zero();
        float det = m.determinant();
        REQUIRE_THAT(det, WithinAbs(0.0f, 1e-6f));
    }
}

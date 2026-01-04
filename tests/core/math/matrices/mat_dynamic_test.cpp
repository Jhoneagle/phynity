#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat_dynamic.hpp>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <sstream>

using phynity::math::matrices::MatDynamicf;
using phynity::math::vectors::VecDynamicf;
using Catch::Matchers::WithinAbs;

bool matdyn_approx_equal(const MatDynamicf& a, const MatDynamicf& b, float eps = 1e-5f) {
    if (a.rows != b.rows || a.cols != b.cols) return false;
    for (std::size_t i = 0; i < a.rows; ++i) {
        for (std::size_t j = 0; j < a.cols; ++j) {
            if (std::abs(a(i, j) - b(i, j)) > eps) return false;
        }
    }
    return true;
}

TEST_CASE("MatDynamicf: constructors", "[MatDynamicf][constructor]") {
    SECTION("Zero by default") {
        MatDynamicf m(2, 3);
        for (std::size_t i = 0; i < 2; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                REQUIRE_THAT(m(i, j), WithinAbs(0.0f, 1e-6f));
            }
        }
    }

    SECTION("Scalar fill") {
        MatDynamicf m(2, 2, 4.0f);
        REQUIRE_THAT(m(0, 0), WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(4.0f, 1e-6f));
    }
}

TEST_CASE("MatDynamicf: queries", "[MatDynamicf][query]") {
    MatDynamicf empty;
    REQUIRE(empty.isEmpty());
    REQUIRE(empty.numRows() == 0);
    REQUIRE(empty.numCols() == 0);
    REQUIRE(empty.size() == 0);

    MatDynamicf filled(2, 3, 1.0f);
    REQUIRE_FALSE(filled.isEmpty());
    REQUIRE(filled.numRows() == 2);
    REQUIRE(filled.numCols() == 3);
    REQUIRE(filled.size() == 6);
}

TEST_CASE("MatDynamicf: factories", "[MatDynamicf][factory]") {
    MatDynamicf zero = MatDynamicf::zero(3, 2);
    MatDynamicf eye = MatDynamicf::identity(3);

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

TEST_CASE("MatDynamicf: checked access", "[MatDynamicf][access]") {
    MatDynamicf m(2, 2);
    m.at(1, 1) = 4.0f;
    REQUIRE_THAT(m.at(1, 1), WithinAbs(4.0f, 1e-6f));
    REQUIRE_THROWS_AS(m.at(2, 0), std::out_of_range);
    REQUIRE_THROWS_AS(m.at(0, 2), std::out_of_range);
}

TEST_CASE("MatDynamicf: arithmetic", "[MatDynamicf][arithmetic]") {
    MatDynamicf a(2, 2, 1.0f);
    MatDynamicf b(2, 2, 2.0f);

    SECTION("Add/Sub") {
        MatDynamicf c = a + b;
        MatDynamicf d = c - a;
        REQUIRE_THAT(c(0, 0), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(d(1, 1), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Scalar multiply/divide") {
        MatDynamicf c = b * 0.5f;
        MatDynamicf d = b / 2.0f;
        REQUIRE_THAT(c(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(d(1, 1), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Compound assign") {
        MatDynamicf c = a;
        c += a;
        c -= a;
        c *= 3.0f;
        c /= 3.0f;
        REQUIRE(matdyn_approx_equal(c, a));
    }

    SECTION("Negation") {
        MatDynamicf m(2, 2);
        m(0, 0) = 1.0f; m(0, 1) = -2.0f;
        m(1, 0) = 3.0f; m(1, 1) = -4.0f;
        
        MatDynamicf result = -m;
        
        REQUIRE_THAT(result(0, 0), WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(result(0, 1), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result(1, 0), WithinAbs(-3.0f, 1e-6f));
        REQUIRE_THAT(result(1, 1), WithinAbs(4.0f, 1e-6f));
        
        // Double negation
        MatDynamicf double_neg = -(-m);
        REQUIRE_THAT(double_neg(0, 0), WithinAbs(m(0, 0), 1e-6f));
        REQUIRE_THAT(double_neg(1, 1), WithinAbs(m(1, 1), 1e-6f));
    }
}

TEST_CASE("MatDynamicf: Approximate equality", "[MatDynamicf][comparison]") {
    MatDynamicf m1(2, 2, 1.0f);
    MatDynamicf m2(2, 2, 1.0f);
    MatDynamicf m3(2, 2, 1.000001f);
    MatDynamicf m4(2, 2, 2.0f);
    MatDynamicf m5(3, 3, 1.0f); // Different size

    REQUIRE(m1.approxEqual(m2));
    REQUIRE(m1.approxEqual(m3, 1e-5f));
    REQUIRE_FALSE(m1.approxEqual(m4, 1e-6f));
    REQUIRE(m1.approxEqual(m4, 1.5f));
    REQUIRE_FALSE(m1.approxEqual(m5)); // Different dimensions
}

TEST_CASE("MatDynamicf: Absolute value", "[MatDynamicf][operations]") {
    MatDynamicf m(2, 3);
    m(0, 0) = -1.0f; m(0, 1) = 2.0f;  m(0, 2) = -3.0f;
    m(1, 0) = 4.0f;  m(1, 1) = -5.0f; m(1, 2) = 6.0f;

    MatDynamicf result = m.abs();

    REQUIRE_THAT(result(0, 0), WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result(0, 1), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result(0, 2), WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result(1, 0), WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result(1, 1), WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result(1, 2), WithinAbs(6.0f, 1e-6f));
}

TEST_CASE("MatDynamicf: Component-wise operations", "[MatDynamicf][arithmetic]") {
    MatDynamicf m1(2, 2);
    m1(0, 0) = 4.0f; m1(0, 1) = 6.0f;
    m1(1, 0) = 8.0f; m1(1, 1) = 10.0f;
    MatDynamicf m2(2, 2);
    m2(0, 0) = 2.0f; m2(0, 1) = 3.0f;
    m2(1, 0) = 4.0f; m2(1, 1) = 5.0f;

    SECTION("Component-wise multiplication") {
        MatDynamicf m = m1;
        m.mulComponentWise(m2);
        REQUIRE_THAT(m(0, 0), WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(m(0, 1), WithinAbs(18.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(32.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(50.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        MatDynamicf m = m1;
        m.divComponentWise(m2);
        REQUIRE_THAT(m(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(0, 1), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Shape mismatch throws") {
        MatDynamicf m3(3, 3, 1.0f);
        REQUIRE_THROWS_AS(m1.mulComponentWise(m3), std::invalid_argument);
        REQUIRE_THROWS_AS(m1.divComponentWise(m3), std::invalid_argument);
    }
}

TEST_CASE("MatDynamicf: shape mismatch throws", "[MatDynamicf][errors]") {
    MatDynamicf a(2, 2, 1.0f);
    MatDynamicf b(3, 3, 1.0f);

    REQUIRE_THROWS_AS(a + b, std::invalid_argument);
    REQUIRE_THROWS_AS(a - b, std::invalid_argument);
    REQUIRE_THROWS_AS(a * b, std::invalid_argument);
}

TEST_CASE("MatDynamicf: matrix multiplication", "[MatDynamicf][multiply]") {
    MatDynamicf a(2, 3);
    MatDynamicf b(3, 2);
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

    MatDynamicf result = a * b;
    MatDynamicf expected(2, 2);
    expected(0, 0) = 58.0f;
    expected(0, 1) = 64.0f;
    expected(1, 0) = 139.0f;
    expected(1, 1) = 154.0f;

    REQUIRE(matdyn_approx_equal(result, expected));
}

TEST_CASE("MatDynamicf: matrix-vector multiplication", "[MatDynamicf][vector]") {
    MatDynamicf m(2, 3);
    m(0, 0) = 1.0f; m(0, 1) = 2.0f; m(0, 2) = 3.0f;
    m(1, 0) = 4.0f; m(1, 1) = 5.0f; m(1, 2) = 6.0f;

    VecDynamicf v(3, 1.0f);
    VecDynamicf r = m * v;

    REQUIRE_THAT(r[0], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(r[1], WithinAbs(15.0f, 1e-6f));

    VecDynamicf wrong(2, 1.0f);
    REQUIRE_THROWS_AS(m * wrong, std::invalid_argument);
}

TEST_CASE("MatDynamicf: row and column accessors", "[MatDynamicf][access][rowcol]") {
    MatDynamicf m(2, 3);
    float val = 1.0f;
    for (std::size_t i = 0; i < 2; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            m(i, j) = val++;
        }
    }

    VecDynamicf row0 = m.getRow(0);
    REQUIRE_THAT(row0[0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(row0[2], WithinAbs(3.0f, 1e-6f));

    VecDynamicf col1 = m.getColumn(1);
    REQUIRE_THAT(col1[0], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(col1[1], WithinAbs(5.0f, 1e-6f));

    VecDynamicf newRow(3);
    newRow[0] = 7.0f; newRow[1] = 8.0f; newRow[2] = 9.0f;
    m.setRow(1, newRow);
    REQUIRE_THAT(m(1, 2), WithinAbs(9.0f, 1e-6f));

    VecDynamicf wrongRow(2, 1.0f);
    REQUIRE_THROWS_AS(m.setRow(0, wrongRow), std::invalid_argument);

    VecDynamicf newCol(2);
    newCol[0] = -1.0f; newCol[1] = -2.0f;
    m.setColumn(0, newCol);
    REQUIRE_THAT(m(1, 0), WithinAbs(-2.0f, 1e-6f));

    VecDynamicf wrongCol(3, 1.0f);
    REQUIRE_THROWS_AS(m.setColumn(2, wrongCol), std::invalid_argument);
}

TEST_CASE("MatDynamicf: transpose", "[MatDynamicf][transpose]") {
    MatDynamicf m(2, 3);
    m(0, 1) = 2.0f;
    m(1, 2) = 3.0f;

    MatDynamicf t = m.transposed();
    REQUIRE_THAT(t(1, 0), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(t(2, 1), WithinAbs(3.0f, 1e-6f));

    m.transpose();
    REQUIRE_THAT(m(1, 0), WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("MatDynamicf: equality", "[MatDynamicf][comparison]") {
    MatDynamicf a(2, 2, 1.0f);
    MatDynamicf b(2, 2, 1.0f);
    MatDynamicf c(2, 2, 2.0f);

    REQUIRE(a == b);
    REQUIRE(a != c);
}

TEST_CASE("MatDynamicf: stream output", "[MatDynamicf][stream]") {
    MatDynamicf m(2, 2);
    m(0, 0) = 1.0f; m(0, 1) = 2.0f;
    m(1, 0) = 3.0f; m(1, 1) = 4.0f;
    std::ostringstream oss;
    oss << m;
    REQUIRE(oss.str() == "[(1, 2), (3, 4)]");
}

TEST_CASE("MatDynamicf: Edge cases", "[MatDynamicf][edge]") {
    SECTION("Division by zero scalar") {
        MatDynamicf m(2, 2, 1.0f);
        MatDynamicf result = m / 0.0f;
        REQUIRE((std::isinf(result(0, 0)) || std::isnan(result(0, 0))));
    }

    SECTION("Empty matrix operations") {
        MatDynamicf m(0, 0);
        REQUIRE(m.isEmpty());
        REQUIRE(m.numRows() == 0);
        REQUIRE(m.numCols() == 0);
    }

    SECTION("Non-square matrix operations") {
        MatDynamicf m(2, 3, 1.0f);
        MatDynamicf n(2, 3, 2.0f);
        MatDynamicf sum = m + n;
        REQUIRE_THAT(sum(0, 0), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(sum(1, 2), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Very large numbers") {
        MatDynamicf m(3, 3, 1e20f);
        MatDynamicf product = m * 2.0f;
        REQUIRE(product(1, 1) > 1e20f);
    }

    SECTION("Very small numbers") {
        MatDynamicf m(2, 2, 1e-20f);
        MatDynamicf sum = m + m;
        REQUIRE(sum(0, 0) > 0.0f);
    }

    SECTION("Transpose of non-square matrix") {
        MatDynamicf m(2, 3, 0.0f);
        m(0, 0) = 1.0f; m(0, 1) = 2.0f; m(0, 2) = 3.0f;
        m(1, 0) = 4.0f; m(1, 1) = 5.0f; m(1, 2) = 6.0f;
        
        MatDynamicf t = m.transposed();
        REQUIRE(t.numRows() == 3);
        REQUIRE(t.numCols() == 2);
        REQUIRE_THAT(t(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(t(1, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(t(2, 1), WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Single element matrix") {
        MatDynamicf m(1, 1, 42.0f);
        REQUIRE_THAT(m(0, 0), WithinAbs(42.0f, 1e-6f));
        REQUIRE(m.size() == 1);
    }
}

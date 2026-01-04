#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat2.hpp>
#include <core/math/utilities/constants.hpp>
#include <cmath>
#include <sstream>

using phynity::math::matrices::Mat2f;
using phynity::math::vectors::Vec2f;
using phynity::math::utilities::mathf;
using Catch::Matchers::WithinAbs;

// Helper function to check if two matrices are approximately equal
bool mat2_approx_equal(const Mat2f& a, const Mat2f& b, float epsilon = 1e-4f) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            if (std::abs(a.m[i][j] - b.m[i][j]) > epsilon) {
                return false;
            }
        }
    }
    return true;
}

// ============================================================================
// Constructors and Basic Properties
// ============================================================================

TEST_CASE("Mat2f: Constructors", "[Mat2f][constructor]") {
    SECTION("Default constructor creates identity matrix") {
        Mat2f m;
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Scalar constructor fills all elements") {
        Mat2f m(5.0f);
        REQUIRE_THAT(m.m[0][0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Element-wise constructor") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Column vector constructor") {
        Vec2f col0(1.0f, 3.0f);
        Vec2f col1(2.0f, 4.0f);
        Mat2f m(col0, col1);
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(4.0f, 1e-6f));
    }
}

TEST_CASE("Mat2f: Static factory methods", "[Mat2f][factory]") {
    SECTION("Identity matrix") {
        Mat2f m = Mat2f::identity();
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Zero matrix") {
        Mat2f m = Mat2f::zero();
        REQUIRE_THAT(m.m[0][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Mat2f: Addition", "[Mat2f][arithmetic]") {
    Mat2f a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f b(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2f c = a + b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(c.m[0][1], WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][0], WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(12.0f, 1e-6f));
}

TEST_CASE("Mat2f: Subtraction", "[Mat2f][arithmetic]") {
    Mat2f a(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2f b(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f c = a - b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.m[0][1], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(4.0f, 1e-6f));
}
TEST_CASE("Mat2f: Negation", "[Mat2f][arithmetic]") {
    Mat2f m(1.0f, -2.0f, 3.0f, -4.0f);
    Mat2f result = -m;

    REQUIRE_THAT(result.m[0][0], WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(result.m[0][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][0], WithinAbs(-3.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(4.0f, 1e-6f));

    // Double negation
    Mat2f double_neg = -(-m);
    REQUIRE(mat2_approx_equal(double_neg, m));
}
TEST_CASE("Mat2f: Scalar multiplication", "[Mat2f][arithmetic]") {
    Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
    
    SECTION("Matrix * scalar") {
        Mat2f result = m * 2.0f;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[0][1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][0], WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][1], WithinAbs(8.0f, 1e-6f));
    }

    SECTION("Scalar * matrix") {
        Mat2f result = 2.0f * m;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[0][1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][0], WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][1], WithinAbs(8.0f, 1e-6f));
    }
}

TEST_CASE("Mat2f: Scalar division", "[Mat2f][arithmetic]") {
    Mat2f m(2.0f, 4.0f, 6.0f, 8.0f);
    Mat2f result = m / 2.0f;

    REQUIRE_THAT(result.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.m[0][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][0], WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat2f: Matrix multiplication", "[Mat2f][arithmetic]") {
    Mat2f m1(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f m2(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2f result = m1 * m2;

    // [1*5 + 2*7, 1*6 + 2*8] = [19, 22]
    // [3*5 + 4*7, 3*6 + 4*8] = [43, 50]
    REQUIRE_THAT(result.m[0][0], WithinAbs(19.0f, 1e-6f));
    REQUIRE_THAT(result.m[0][1], WithinAbs(22.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][0], WithinAbs(43.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(50.0f, 1e-6f));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Mat2f: Addition assignment", "[Mat2f][assignment]") {
    Mat2f a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f b(5.0f, 6.0f, 7.0f, 8.0f);
    a += b;

    REQUIRE_THAT(a.m[0][0], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(a.m[0][1], WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(a.m[1][0], WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(a.m[1][1], WithinAbs(12.0f, 1e-6f));
}

TEST_CASE("Mat2f: Subtraction assignment", "[Mat2f][assignment]") {
    Mat2f a(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2f b(1.0f, 2.0f, 3.0f, 4.0f);
    a -= b;

    REQUIRE_THAT(a.m[0][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(a.m[0][1], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(a.m[1][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(a.m[1][1], WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat2f: Scalar multiplication assignment", "[Mat2f][assignment]") {
    Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
    m *= 2.0f;

    REQUIRE_THAT(m.m[0][0], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][1], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][0], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][1], WithinAbs(8.0f, 1e-6f));
}

TEST_CASE("Mat2f: Scalar division assignment", "[Mat2f][assignment]") {
    Mat2f m(2.0f, 4.0f, 6.0f, 8.0f);
    m /= 2.0f;

    REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][0], WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][1], WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat2f: Matrix multiplication assignment", "[Mat2f][assignment]") {
    Mat2f m1(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f m2(5.0f, 6.0f, 7.0f, 8.0f);
    m1 *= m2;

    REQUIRE_THAT(m1.m[0][0], WithinAbs(19.0f, 1e-6f));
    REQUIRE_THAT(m1.m[0][1], WithinAbs(22.0f, 1e-6f));
    REQUIRE_THAT(m1.m[1][0], WithinAbs(43.0f, 1e-6f));
    REQUIRE_THAT(m1.m[1][1], WithinAbs(50.0f, 1e-6f));
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Mat2f: Equality comparison", "[Mat2f][comparison]") {
    Mat2f a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f b(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f c(5.0f, 6.0f, 7.0f, 8.0f);

    REQUIRE(a == b);
    REQUIRE_FALSE(a == c);
}

TEST_CASE("Mat2f: Inequality comparison", "[Mat2f][comparison]") {
    Mat2f a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f b(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2f c(1.0f, 2.0f, 3.0f, 4.0f);

    REQUIRE(a != b);
    REQUIRE_FALSE(a != c);
}

// ============================================================================
// Matrix-Vector Operations
// ============================================================================

TEST_CASE("Mat2f: Matrix-vector multiplication", "[Mat2f][vector]") {
    SECTION("Matrix * vector (column vector)") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        Vec2f v(5.0f, 6.0f);
        Vec2f result = m * v;

        // [1*5 + 2*6] = [17]
        // [3*5 + 4*6] = [39]
        REQUIRE_THAT(result.x, WithinAbs(17.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(39.0f, 1e-6f));
    }

    SECTION("Vector * matrix (row vector)") {
        Vec2f v(5.0f, 6.0f);
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        Vec2f result = v * m;

        // [5*1 + 6*3, 5*2 + 6*4] = [23, 34]
        REQUIRE_THAT(result.x, WithinAbs(23.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(34.0f, 1e-6f));
    }
}

// ============================================================================
// Approximate Equality
// ============================================================================

TEST_CASE("Mat2f: Approximate equality", "[Mat2f][comparison]") {
    Mat2f m1(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f m2(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f m3(1.00001f, 2.00001f, 3.00001f, 4.00001f);
    Mat2f m4(1.1f, 2.1f, 3.1f, 4.1f);

    REQUIRE(m1.approxEqual(m2));
    REQUIRE(m1.approxEqual(m3, 1e-4f));
    REQUIRE_FALSE(m1.approxEqual(m4, 1e-6f));
    REQUIRE(m1.approxEqual(m4, 0.2f));
}

TEST_CASE("Mat2f: Absolute value", "[Mat2f][operations]") {
    Mat2f m(-1.0f, 2.0f, -3.0f, 4.0f);
    Mat2f result = m.abs();

    REQUIRE_THAT(result.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.m[0][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][0], WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat2f: Component-wise operations", "[Mat2f][arithmetic]") {
    Mat2f m1(2.0f, 4.0f, 6.0f, 8.0f);
    Mat2f m2(1.0f, 2.0f, 3.0f, 4.0f);

    SECTION("Component-wise multiplication") {
        Mat2f m = m1;
        m.mulComponentWise(m2);
        REQUIRE_THAT(m.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(18.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(32.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        Mat2f m = m1;
        m.divComponentWise(m2);
        REQUIRE_THAT(m.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(2.0f, 1e-6f));
    }
}

// ============================================================================
// Matrix Operations
// ============================================================================

TEST_CASE("Mat2f: Determinant", "[Mat2f][operations]") {
    SECTION("Identity has determinant 1") {
        Mat2f m = Mat2f::identity();
        REQUIRE_THAT(m.determinant(), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("General matrix determinant") {
        Mat2f m(3.0f, 8.0f, 4.0f, 6.0f);
        // det = 3*6 - 8*4 = 18 - 32 = -14
        REQUIRE_THAT(m.determinant(), WithinAbs(-14.0f, 1e-6f));
    }

    SECTION("Zero matrix has determinant 0") {
        Mat2f m = Mat2f::zero();
        REQUIRE_THAT(m.determinant(), WithinAbs(0.0f, 1e-6f));
    }
}

TEST_CASE("Mat2f: Inverse", "[Mat2f][operations]") {
    SECTION("Identity inverse is identity") {
        Mat2f m = Mat2f::identity();
        Mat2f inv = m.inverse();
        REQUIRE(mat2_approx_equal(inv, Mat2f::identity()));
    }

    SECTION("General matrix inverse") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        Mat2f inv = m.inverse();
        Mat2f product = m * inv;
        REQUIRE(mat2_approx_equal(product, Mat2f::identity(), 1e-3f));
    }

    SECTION("Inverse of inverse returns original") {
        Mat2f m(2.0f, 3.0f, 1.0f, 4.0f);
        Mat2f inv = m.inverse();
        Mat2f inv_inv = inv.inverse();
        REQUIRE(mat2_approx_equal(inv_inv, m, 1e-3f));
    }

    SECTION("Singular matrix returns zero") {
        Mat2f m(1.0f, 2.0f, 2.0f, 4.0f); // Singular (det = 0)
        Mat2f inv = m.inverse();
        REQUIRE(mat2_approx_equal(inv, Mat2f::zero()));
    }
}

TEST_CASE("Mat2f: Transpose", "[Mat2f][operations]") {
    SECTION("Transpose of identity is identity") {
        Mat2f m = Mat2f::identity();
        Mat2f t = m.transposed();
        REQUIRE(mat2_approx_equal(t, Mat2f::identity()));
    }

    SECTION("Transpose swaps rows and columns") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        Mat2f t = m.transposed();
        
        REQUIRE_THAT(t.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(t.m[0][1], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(t.m[1][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(t.m[1][1], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("In-place transpose") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        m.transpose();
        
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Double transpose returns original") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        Mat2f tt = m.transposed().transposed();
        REQUIRE(mat2_approx_equal(tt, m));
    }
}

TEST_CASE("Mat2f: Trace", "[Mat2f][operations]") {
    SECTION("Trace of identity is 2") {
        Mat2f m = Mat2f::identity();
        REQUIRE_THAT(m.trace(), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Trace is sum of diagonal") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(m.trace(), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Trace of zero is 0") {
        Mat2f m = Mat2f::zero();
        REQUIRE_THAT(m.trace(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Element Access
// ============================================================================

TEST_CASE("Mat2f: Element access", "[Mat2f][access]") {
    Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);

    SECTION("Read access") {
        REQUIRE_THAT(m(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m(0, 1), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Write access") {
        m(0, 0) = 10.0f;
        m(1, 1) = 20.0f;
        REQUIRE_THAT(m(0, 0), WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(20.0f, 1e-6f));
    }
}

TEST_CASE("Mat2f: Row and column accessors", "[Mat2f][access][rowcol]") {
    Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);

    Vec2f row0 = m.getRow(0);
    REQUIRE_THAT(row0.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(row0.y, WithinAbs(2.0f, 1e-6f));

    Vec2f col1 = m.getColumn(1);
    REQUIRE_THAT(col1.x, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(col1.y, WithinAbs(4.0f, 1e-6f));

    Vec2f newRow(9.0f, 8.0f);
    m.setRow(1, newRow);
    REQUIRE_THAT(m.m[1][0], WithinAbs(9.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][1], WithinAbs(8.0f, 1e-6f));

    Vec2f newCol(5.0f, 6.0f);
    m.setColumn(0, newCol);
    REQUIRE_THAT(m.m[0][0], WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][0], WithinAbs(6.0f, 1e-6f));
}

// ============================================================================
// Transformation Matrices
// ============================================================================

TEST_CASE("Mat2f: Rotation matrix", "[Mat2f][transform]") {
    SECTION("90 degree rotation") {
        Mat2f rot = Mat2f::rotation(static_cast<float>(mathf::pi) / 2.0f);
        Vec2f v(1.0f, 0.0f);
        Vec2f rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("45 degree rotation") {
        Mat2f rot = Mat2f::rotation(static_cast<float>(mathf::pi) / 4.0f);
        float expected = std::sqrt(2.0f) / 2.0f;
        
        REQUIRE_THAT(rot.m[0][0], WithinAbs(expected, 1e-5f));
        REQUIRE_THAT(rot.m[0][1], WithinAbs(-expected, 1e-5f));
        REQUIRE_THAT(rot.m[1][0], WithinAbs(expected, 1e-5f));
        REQUIRE_THAT(rot.m[1][1], WithinAbs(expected, 1e-5f));
    }

    SECTION("180 degree rotation") {
        Mat2f rot = Mat2f::rotation(static_cast<float>(mathf::pi));
        Vec2f v(1.0f, 0.0f);
        Vec2f rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(-1.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Rotation matrix has determinant 1") {
        Mat2f rot = Mat2f::rotation(static_cast<float>(mathf::pi) / 3.0f);
        REQUIRE_THAT(rot.determinant(), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("Mat2f: Scale matrix", "[Mat2f][transform]") {
    SECTION("Non-uniform scale") {
        Mat2f scale = Mat2f::scale(2.0f, 3.0f);
        Vec2f v(1.0f, 1.0f);
        Vec2f scaled = scale * v;
        
        REQUIRE_THAT(scaled.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(scaled.y, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Uniform scale") {
        Mat2f scale = Mat2f::scale(5.0f);
        Vec2f v(2.0f, 3.0f);
        Vec2f scaled = scale * v;
        
        REQUIRE_THAT(scaled.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(scaled.y, WithinAbs(15.0f, 1e-6f));
    }

    SECTION("Scale matrix structure") {
        Mat2f scale = Mat2f::scale(2.0f, 3.0f);
        
        REQUIRE_THAT(scale.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(scale.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(scale.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(scale.m[1][1], WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Scale determinant equals product") {
        Mat2f scale = Mat2f::scale(2.0f, 3.0f);
        REQUIRE_THAT(scale.determinant(), WithinAbs(6.0f, 1e-6f));
    }
}

// ============================================================================
// Stream Output
// ============================================================================

TEST_CASE("Mat2f: Stream output", "[Mat2f][stream]") {
    Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
    std::ostringstream oss;
    oss << m;
    // Verify output contains matrix elements (exact format may vary)
    std::string output = oss.str();
    REQUIRE(output.find("1") != std::string::npos);
    REQUIRE(output.find("2") != std::string::npos);
    REQUIRE(output.find("3") != std::string::npos);
    REQUIRE(output.find("4") != std::string::npos);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("Mat2f: Edge cases", "[Mat2f][edge]") {
    SECTION("Division by zero scalar") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        Mat2f result = m / 0.0f;
        REQUIRE((std::isinf(result.m[0][0]) || std::isnan(result.m[0][0])));
    }

    SECTION("Operations with very large numbers") {
        Mat2f m(1e20f, 1e20f, 1e20f, 1e20f);
        Mat2f sum = m + m;
        REQUIRE(sum.m[0][0] > 1e20f);
    }

    SECTION("Operations with very small numbers") {
        Mat2f m(1e-20f, 1e-20f, 1e-20f, 1e-20f);
        Mat2f product = m * 2.0f;
        REQUIRE(product.m[0][0] > 0.0f);
    }

    SECTION("Singular matrix determinant is zero") {
        Mat2f m(1.0f, 2.0f, 2.0f, 4.0f);
        REQUIRE_THAT(m.determinant(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Zero matrix operations") {
        Mat2f zero = Mat2f::zero();
        Mat2f identity = Mat2f::identity();
        Mat2f result = zero + identity;
        REQUIRE(mat2_approx_equal(result, identity));
    }

    SECTION("Matrix multiplication by zero") {
        Mat2f m(1.0f, 2.0f, 3.0f, 4.0f);
        Mat2f zero = Mat2f::zero();
        Mat2f result = m * zero;
        REQUIRE(mat2_approx_equal(result, zero));
    }
}

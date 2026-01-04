#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat3.hpp>
#include <core/math/utilities/constants.hpp>
#include <cmath>
#include <sstream>

using phynity::math::matrices::Mat3f;
using phynity::math::vectors::Vec3f;
using phynity::math::utilities::mathf;
using Catch::Matchers::WithinAbs;

// Helper function to check if two matrices are approximately equal
bool mat3_approx_equal(const Mat3f& a, const Mat3f& b, float epsilon = 1e-4f) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
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

TEST_CASE("Mat3f: Constructors", "[Mat3f][constructor]") {
    SECTION("Default constructor creates identity matrix") {
        Mat3f m;
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][2], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][2], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][2], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][1], WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Scalar constructor fills all elements") {
        Mat3f m(5.0f);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                REQUIRE_THAT(m.m[i][j], WithinAbs(5.0f, 1e-6f));
            }
        }
    }

    SECTION("Element-wise constructor") {
        Mat3f m(1.0f, 2.0f, 3.0f,
               4.0f, 5.0f, 6.0f,
               7.0f, 8.0f, 9.0f);
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][2], WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Column vector constructor") {
        Vec3f col0(1.0f, 4.0f, 7.0f);
        Vec3f col1(2.0f, 5.0f, 8.0f);
        Vec3f col2(3.0f, 6.0f, 9.0f);
        Mat3f m(col0, col1, col2);
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][2], WithinAbs(9.0f, 1e-6f));
    }
}

TEST_CASE("Mat3f: Static factory methods", "[Mat3f][factory]") {
    SECTION("Identity matrix") {
        Mat3f m = Mat3f::identity();
        REQUIRE(mat3_approx_equal(m, Mat3f::identity()));
    }

    SECTION("Zero matrix") {
        Mat3f m = Mat3f::zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                REQUIRE_THAT(m.m[i][j], WithinAbs(0.0f, 1e-6f));
            }
        }
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Mat3f: Addition", "[Mat3f][arithmetic]") {
    Mat3f a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3f b(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);
    Mat3f c = a + b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(c.m[2][2], WithinAbs(10.0f, 1e-6f));
}

TEST_CASE("Mat3f: Subtraction", "[Mat3f][arithmetic]") {
    Mat3f a(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);
    Mat3f b(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3f c = a - b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(c.m[2][2], WithinAbs(-8.0f, 1e-6f));
}

TEST_CASE("Mat3f: Scalar multiplication", "[Mat3f][arithmetic]") {
    Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    
    SECTION("Matrix * scalar") {
        Mat3f result = m * 2.0f;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][1], WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.m[2][2], WithinAbs(18.0f, 1e-6f));
    }

    SECTION("Scalar * matrix") {
        Mat3f result = 2.0f * m;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][1], WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.m[2][2], WithinAbs(18.0f, 1e-6f));
    }
}

TEST_CASE("Mat3f: Scalar division", "[Mat3f][arithmetic]") {
    Mat3f m(2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 18.0f);
    Mat3f result = m / 2.0f;

    REQUIRE_THAT(result.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result.m[2][2], WithinAbs(9.0f, 1e-6f));
}

TEST_CASE("Mat3f: Negation", "[Mat3f][arithmetic]") {
    Mat3f m(1.0f, -2.0f, 3.0f, -4.0f, 5.0f, -6.0f, 7.0f, -8.0f, 9.0f);
    Mat3f result = -m;

    REQUIRE_THAT(result.m[0][0], WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(result.m[0][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result.m[2][2], WithinAbs(-9.0f, 1e-6f));

    // Double negation
    Mat3f double_neg = -(-m);
    REQUIRE(mat3_approx_equal(double_neg, m));
}

TEST_CASE("Mat3f: Matrix multiplication", "[Mat3f][arithmetic]") {
    Mat3f a(1.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 3.0f);
    Mat3f b(2.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 4.0f);
    Mat3f result = a * b;

    REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(result.m[2][2], WithinAbs(12.0f, 1e-6f));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Mat3f: Assignment operators", "[Mat3f][assignment]") {
    Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3f a = m;

    SECTION("Addition assignment") {
        a += m;
        REQUIRE_THAT(a.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(a.m[1][1], WithinAbs(10.0f, 1e-6f));
    }

    SECTION("Subtraction assignment") {
        a -= m;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                REQUIRE_THAT(a.m[i][j], WithinAbs(0.0f, 1e-6f));
            }
        }
    }

    SECTION("Scalar multiplication assignment") {
        a *= 2.0f;
        REQUIRE_THAT(a.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(a.m[1][1], WithinAbs(10.0f, 1e-6f));
    }

    SECTION("Scalar division assignment") {
        a /= 2.0f;
        REQUIRE_THAT(a.m[0][0], WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(a.m[1][1], WithinAbs(2.5f, 1e-6f));
    }
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Mat3f: Equality comparison", "[Mat3f][comparison]") {
    Mat3f a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3f b(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3f c(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);

    REQUIRE(a == b);
    REQUIRE_FALSE(a == c);
}

TEST_CASE("Mat3f: Inequality comparison", "[Mat3f][comparison]") {
    Mat3f a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3f b(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);
    Mat3f c(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

    REQUIRE(a != b);
    REQUIRE_FALSE(a != c);
}

// ============================================================================
// Matrix-Vector Operations
// ============================================================================

TEST_CASE("Mat3f: Matrix-vector multiplication", "[Mat3f][vector]") {
    SECTION("Matrix * vector (column vector)") {
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Vec3f v(1.0f, 2.0f, 3.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(14.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(32.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(50.0f, 1e-6f));
    }

    SECTION("Vector * matrix (row vector)") {
        Vec3f v(1.0f, 2.0f, 3.0f);
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Vec3f result = v * m;

        REQUIRE_THAT(result.x, WithinAbs(30.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(36.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(42.0f, 1e-6f));
    }
}

// ============================================================================
// Matrix Operations
// ============================================================================

TEST_CASE("Mat3f: Determinant", "[Mat3f][operations]") {
    SECTION("Identity has determinant 1") {
        Mat3f m = Mat3f::identity();
        REQUIRE_THAT(m.determinant(), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Zero matrix has determinant 0") {
        Mat3f m = Mat3f::zero();
        REQUIRE_THAT(m.determinant(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("General matrix determinant") {
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 10.0f);
        float det = m.determinant();
        // det = 1*(5*10 - 6*8) - 2*(4*10 - 6*7) + 3*(4*8 - 5*7)
        // det = 1*2 - 2*(-2) + 3*(-3) = 2 + 4 - 9 = -3
        REQUIRE_THAT(det, WithinAbs(-3.0f, 1e-6f));
    }
}

TEST_CASE("Mat3f: Approximate equality", "[Mat3f][comparison]") {
    Mat3f m1 = Mat3f::identity();
    Mat3f m2 = Mat3f::identity();
    Mat3f m3 = Mat3f::identity();
    m3.m[0][0] = 1.00001f;
    Mat3f m4 = Mat3f::identity();
    m4.m[0][0] = 2.0f;

    REQUIRE(m1.approxEqual(m2));
    REQUIRE(m1.approxEqual(m3, 1e-4f));
    REQUIRE_FALSE(m1.approxEqual(m4, 1e-6f));
    REQUIRE(m1.approxEqual(m4, 1.5f));
}

TEST_CASE("Mat3f: Absolute value", "[Mat3f][operations]") {
    Mat3f m;
    m.m[0][0] = -1.0f; m.m[0][1] = 2.0f;  m.m[0][2] = -3.0f;
    m.m[1][0] = 4.0f;  m.m[1][1] = -5.0f; m.m[1][2] = 6.0f;
    m.m[2][0] = -7.0f; m.m[2][1] = 8.0f;  m.m[2][2] = -9.0f;

    Mat3f result = m.abs();

    REQUIRE_THAT(result.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.m[0][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.m[0][2], WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][2], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(result.m[2][0], WithinAbs(7.0f, 1e-6f));
    REQUIRE_THAT(result.m[2][1], WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(result.m[2][2], WithinAbs(9.0f, 1e-6f));
}

TEST_CASE("Mat3f: Component-wise operations", "[Mat3f][arithmetic]") {
    Mat3f m1(2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 18.0f);
    Mat3f m2(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

    SECTION("Component-wise multiplication") {
        Mat3f m = m1;
        m.mulComponentWise(m2);
        REQUIRE_THAT(m.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][2], WithinAbs(18.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(50.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][2], WithinAbs(162.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        Mat3f m = m1;
        m.divComponentWise(m2);
        REQUIRE_THAT(m.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][2], WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("Mat3f: Inverse", "[Mat3f][operations]") {
    SECTION("Identity inverse is identity") {
        Mat3f m = Mat3f::identity();
        Mat3f inv = m.inverse();
        REQUIRE(mat3_approx_equal(inv, Mat3f::identity()));
    }

    SECTION("Matrix times its inverse equals identity") {
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 2.0f, 1.0f, 8.0f);
        Mat3f inv = m.inverse();
        Mat3f product = m * inv;
        REQUIRE(mat3_approx_equal(product, Mat3f::identity(), 1e-3f));
    }

    SECTION("Singular matrix returns zero") {
        Mat3f m(1.0f, 2.0f, 3.0f, 2.0f, 4.0f, 6.0f, 3.0f, 6.0f, 9.0f);
        Mat3f inv = m.inverse();
        REQUIRE(mat3_approx_equal(inv, Mat3f::zero()));
    }
}

TEST_CASE("Mat3f: Transpose", "[Mat3f][operations]") {
    SECTION("Transpose of identity is identity") {
        Mat3f m = Mat3f::identity();
        Mat3f t = m.transposed();
        REQUIRE(mat3_approx_equal(t, Mat3f::identity()));
    }

    SECTION("Transpose swaps rows and columns") {
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Mat3f t = m.transposed();
        
        REQUIRE_THAT(t.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(t.m[0][1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(t.m[1][0], WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Double transpose returns original") {
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Mat3f tt = m.transposed().transposed();
        REQUIRE(mat3_approx_equal(tt, m));
    }
}

TEST_CASE("Mat3f: Trace", "[Mat3f][operations]") {
    SECTION("Trace of identity is 3") {
        Mat3f m = Mat3f::identity();
        REQUIRE_THAT(m.trace(), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Trace is sum of diagonal") {
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        REQUIRE_THAT(m.trace(), WithinAbs(15.0f, 1e-6f));
    }

    SECTION("Trace of zero is 0") {
        Mat3f m = Mat3f::zero();
        REQUIRE_THAT(m.trace(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Element Access
// ============================================================================

TEST_CASE("Mat3f: Element access", "[Mat3f][access]") {
    Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

    SECTION("Read access") {
        REQUIRE_THAT(m(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m(2, 2), WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Write access") {
        m(0, 0) = 10.0f;
        m(2, 2) = 20.0f;
        REQUIRE_THAT(m(0, 0), WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(m(2, 2), WithinAbs(20.0f, 1e-6f));
    }
}

TEST_CASE("Mat3f: Row and column accessors", "[Mat3f][access][rowcol]") {
    Mat3f m(1.0f, 2.0f, 3.0f,
           4.0f, 5.0f, 6.0f,
           7.0f, 8.0f, 9.0f);

    Vec3f row1 = m.getRow(1);
    REQUIRE_THAT(row1.x, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(row1.y, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(row1.z, WithinAbs(6.0f, 1e-6f));

    Vec3f col2 = m.getColumn(2);
    REQUIRE_THAT(col2.x, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(col2.y, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(col2.z, WithinAbs(9.0f, 1e-6f));

    Vec3f newRow(9.0f, 8.0f, 7.0f);
    m.setRow(0, newRow);
    REQUIRE_THAT(m.m[0][0], WithinAbs(9.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][2], WithinAbs(7.0f, 1e-6f));

    Vec3f newCol(1.0f, 2.0f, 3.0f);
    m.setColumn(1, newCol);
    REQUIRE_THAT(m.m[1][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(m.m[2][1], WithinAbs(3.0f, 1e-6f));
}

// ============================================================================
// Rotation Matrices
// ============================================================================

TEST_CASE("Mat3f: Rotation matrices", "[Mat3f][transform]") {
    SECTION("Rotation X axis by 90 degrees") {
        Mat3f rot = Mat3f::rotationX(static_cast<float>(mathf::pi) / 2.0f);
        Vec3f v(0.0f, 1.0f, 0.0f);
        Vec3f rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.z, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Rotation Y axis by 90 degrees") {
        Mat3f rot = Mat3f::rotationY(static_cast<float>(mathf::pi) / 2.0f);
        Vec3f v(1.0f, 0.0f, 0.0f);
        Vec3f rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.z, WithinAbs(-1.0f, 1e-5f));
    }

    SECTION("Rotation Z axis by 90 degrees") {
        Mat3f rot = Mat3f::rotationZ(static_cast<float>(mathf::pi) / 2.0f);
        Vec3f v(1.0f, 0.0f, 0.0f);
        Vec3f rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Rotation around arbitrary axis") {
        Vec3f axis(1.0f, 0.0f, 0.0f);
        Mat3f rot = Mat3f::rotationAxis(axis, static_cast<float>(mathf::pi) / 2.0f);
        Vec3f v(0.0f, 1.0f, 0.0f);
        Vec3f rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.z, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Rotation matrices have determinant 1") {
        Mat3f rotX = Mat3f::rotationX(0.5f);
        Mat3f rotY = Mat3f::rotationY(0.7f);
        Mat3f rotZ = Mat3f::rotationZ(0.3f);
        
        REQUIRE_THAT(rotX.determinant(), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotY.determinant(), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotZ.determinant(), WithinAbs(1.0f, 1e-5f));
    }
}

// ============================================================================
// Scale Matrices
// ============================================================================

TEST_CASE("Mat3f: Scale matrix", "[Mat3f][transform]") {
    SECTION("Non-uniform scale") {
        Mat3f scale = Mat3f::scale(2.0f, 3.0f, 4.0f);
        Vec3f v(1.0f, 1.0f, 1.0f);
        Vec3f scaled = scale * v;
        
        REQUIRE_THAT(scaled.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(scaled.y, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(scaled.z, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Uniform scale") {
        Mat3f scale = Mat3f::scale(5.0f);
        Vec3f v(2.0f, 3.0f, 4.0f);
        Vec3f scaled = scale * v;
        
        REQUIRE_THAT(scaled.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(scaled.y, WithinAbs(15.0f, 1e-6f));
        REQUIRE_THAT(scaled.z, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Scale matrix structure") {
        Mat3f scale = Mat3f::scale(2.0f, 3.0f, 4.0f);
        
        REQUIRE_THAT(scale.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(scale.m[1][1], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(scale.m[2][2], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Scale determinant equals product") {
        Mat3f scale = Mat3f::scale(2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(scale.determinant(), WithinAbs(24.0f, 1e-6f));
    }
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("Mat3f: Edge cases", "[Mat3f][edge]") {
    SECTION("Division by zero scalar") {
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Mat3f result = m / 0.0f;
        REQUIRE((std::isinf(result.m[0][0]) || std::isnan(result.m[0][0])));
    }

    SECTION("Operations with very large numbers") {
        Mat3f m(1e20f);
        Mat3f sum = m + m;
        REQUIRE(sum.m[0][0] > 1e20f);
    }

    SECTION("Operations with very small numbers") {
        Mat3f m(1e-20f);
        Mat3f product = m * 2.0f;
        REQUIRE(product.m[1][1] > 0.0f);
    }

    SECTION("Singular matrix determinant is zero") {
        Mat3f m(1.0f, 2.0f, 3.0f,
               2.0f, 4.0f, 6.0f,
               3.0f, 6.0f, 9.0f);
        REQUIRE_THAT(m.determinant(), WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Identity matrix properties") {
        Mat3f identity = Mat3f::identity();
        Mat3f m(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f);
        Mat3f result = m * identity;
        REQUIRE(mat3_approx_equal(result, m));
    }

    SECTION("Transpose of transpose returns original") {
        Mat3f m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Mat3f result = m.transposed().transposed();
        REQUIRE(mat3_approx_equal(result, m));
    }
}

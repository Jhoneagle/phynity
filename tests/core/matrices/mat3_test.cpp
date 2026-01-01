#define _USE_MATH_DEFINES
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat3.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using phynity::math::matrices::Mat3;
using phynity::math::vectors::Vec3;
using Catch::Matchers::WithinAbs;

// Helper function to check if two matrices are approximately equal
bool mat3_approx_equal(const Mat3& a, const Mat3& b, float epsilon = 1e-4f) {
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

TEST_CASE("Mat3: Constructors", "[Mat3][constructor]") {
    SECTION("Default constructor creates identity matrix") {
        Mat3 m;
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
        Mat3 m(5.0f);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                REQUIRE_THAT(m.m[i][j], WithinAbs(5.0f, 1e-6f));
            }
        }
    }

    SECTION("Element-wise constructor") {
        Mat3 m(1.0f, 2.0f, 3.0f,
               4.0f, 5.0f, 6.0f,
               7.0f, 8.0f, 9.0f);
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][2], WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Column vector constructor") {
        Vec3 col0(1.0f, 4.0f, 7.0f);
        Vec3 col1(2.0f, 5.0f, 8.0f);
        Vec3 col2(3.0f, 6.0f, 9.0f);
        Mat3 m(col0, col1, col2);
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][2], WithinAbs(9.0f, 1e-6f));
    }
}

TEST_CASE("Mat3: Static factory methods", "[Mat3][factory]") {
    SECTION("Identity matrix") {
        Mat3 m = Mat3::identity();
        REQUIRE(mat3_approx_equal(m, Mat3::identity()));
    }

    SECTION("Zero matrix") {
        Mat3 m = Mat3::zero();
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

TEST_CASE("Mat3: Addition", "[Mat3][arithmetic]") {
    Mat3 a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3 b(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);
    Mat3 c = a + b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(c.m[2][2], WithinAbs(10.0f, 1e-6f));
}

TEST_CASE("Mat3: Subtraction", "[Mat3][arithmetic]") {
    Mat3 a(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);
    Mat3 b(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3 c = a - b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(c.m[2][2], WithinAbs(-8.0f, 1e-6f));
}

TEST_CASE("Mat3: Scalar multiplication", "[Mat3][arithmetic]") {
    Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    
    SECTION("Matrix * scalar") {
        Mat3 result = m * 2.0f;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][1], WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.m[2][2], WithinAbs(18.0f, 1e-6f));
    }

    SECTION("Scalar * matrix") {
        Mat3 result = 2.0f * m;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][1], WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.m[2][2], WithinAbs(18.0f, 1e-6f));
    }
}

TEST_CASE("Mat3: Scalar division", "[Mat3][arithmetic]") {
    Mat3 m(2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 18.0f);
    Mat3 result = m / 2.0f;

    REQUIRE_THAT(result.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result.m[2][2], WithinAbs(9.0f, 1e-6f));
}

TEST_CASE("Mat3: Matrix multiplication", "[Mat3][arithmetic]") {
    Mat3 a(1.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 3.0f);
    Mat3 b(2.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 4.0f);
    Mat3 result = a * b;

    REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(result.m[2][2], WithinAbs(12.0f, 1e-6f));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Mat3: Assignment operators", "[Mat3][assignment]") {
    Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3 a = m;

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

TEST_CASE("Mat3: Equality comparison", "[Mat3][comparison]") {
    Mat3 a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3 b(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3 c(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);

    REQUIRE(a == b);
    REQUIRE_FALSE(a == c);
}

TEST_CASE("Mat3: Inequality comparison", "[Mat3][comparison]") {
    Mat3 a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Mat3 b(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);
    Mat3 c(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

    REQUIRE(a != b);
    REQUIRE_FALSE(a != c);
}

// ============================================================================
// Matrix-Vector Operations
// ============================================================================

TEST_CASE("Mat3: Matrix-vector multiplication", "[Mat3][vector]") {
    SECTION("Matrix * vector (column vector)") {
        Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Vec3 v(1.0f, 2.0f, 3.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(14.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(32.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(50.0f, 1e-6f));
    }

    SECTION("Vector * matrix (row vector)") {
        Vec3 v(1.0f, 2.0f, 3.0f);
        Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Vec3 result = v * m;

        REQUIRE_THAT(result.x, WithinAbs(30.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(36.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(42.0f, 1e-6f));
    }
}

// ============================================================================
// Matrix Operations
// ============================================================================

TEST_CASE("Mat3: Determinant", "[Mat3][operations]") {
    SECTION("Identity has determinant 1") {
        Mat3 m = Mat3::identity();
        REQUIRE_THAT(m.determinant(), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Zero matrix has determinant 0") {
        Mat3 m = Mat3::zero();
        REQUIRE_THAT(m.determinant(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("General matrix determinant") {
        Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 10.0f);
        float det = m.determinant();
        // det = 1*(5*10 - 6*8) - 2*(4*10 - 6*7) + 3*(4*8 - 5*7)
        // det = 1*2 - 2*(-2) + 3*(-3) = 2 + 4 - 9 = -3
        REQUIRE_THAT(det, WithinAbs(-3.0f, 1e-6f));
    }
}

TEST_CASE("Mat3: Inverse", "[Mat3][operations]") {
    SECTION("Identity inverse is identity") {
        Mat3 m = Mat3::identity();
        Mat3 inv = m.inverse();
        REQUIRE(mat3_approx_equal(inv, Mat3::identity()));
    }

    SECTION("Matrix times its inverse equals identity") {
        Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 2.0f, 1.0f, 8.0f);
        Mat3 inv = m.inverse();
        Mat3 product = m * inv;
        REQUIRE(mat3_approx_equal(product, Mat3::identity(), 1e-3f));
    }

    SECTION("Singular matrix returns zero") {
        Mat3 m(1.0f, 2.0f, 3.0f, 2.0f, 4.0f, 6.0f, 3.0f, 6.0f, 9.0f);
        Mat3 inv = m.inverse();
        REQUIRE(mat3_approx_equal(inv, Mat3::zero()));
    }
}

TEST_CASE("Mat3: Transpose", "[Mat3][operations]") {
    SECTION("Transpose of identity is identity") {
        Mat3 m = Mat3::identity();
        Mat3 t = m.transposed();
        REQUIRE(mat3_approx_equal(t, Mat3::identity()));
    }

    SECTION("Transpose swaps rows and columns") {
        Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Mat3 t = m.transposed();
        
        REQUIRE_THAT(t.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(t.m[0][1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(t.m[1][0], WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Double transpose returns original") {
        Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        Mat3 tt = m.transposed().transposed();
        REQUIRE(mat3_approx_equal(tt, m));
    }
}

TEST_CASE("Mat3: Trace", "[Mat3][operations]") {
    SECTION("Trace of identity is 3") {
        Mat3 m = Mat3::identity();
        REQUIRE_THAT(m.trace(), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Trace is sum of diagonal") {
        Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
        REQUIRE_THAT(m.trace(), WithinAbs(15.0f, 1e-6f));
    }

    SECTION("Trace of zero is 0") {
        Mat3 m = Mat3::zero();
        REQUIRE_THAT(m.trace(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Element Access
// ============================================================================

TEST_CASE("Mat3: Element access", "[Mat3][access]") {
    Mat3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

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

TEST_CASE("Mat3: Row and column accessors", "[Mat3][access][rowcol]") {
    Mat3 m(1.0f, 2.0f, 3.0f,
           4.0f, 5.0f, 6.0f,
           7.0f, 8.0f, 9.0f);

    Vec3 row1 = m.getRow(1);
    REQUIRE_THAT(row1.x, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(row1.y, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(row1.z, WithinAbs(6.0f, 1e-6f));

    Vec3 col2 = m.getColumn(2);
    REQUIRE_THAT(col2.x, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(col2.y, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(col2.z, WithinAbs(9.0f, 1e-6f));

    Vec3 newRow(9.0f, 8.0f, 7.0f);
    m.setRow(0, newRow);
    REQUIRE_THAT(m.m[0][0], WithinAbs(9.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][2], WithinAbs(7.0f, 1e-6f));

    Vec3 newCol(1.0f, 2.0f, 3.0f);
    m.setColumn(1, newCol);
    REQUIRE_THAT(m.m[1][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(m.m[2][1], WithinAbs(3.0f, 1e-6f));
}

// ============================================================================
// Rotation Matrices
// ============================================================================

TEST_CASE("Mat3: Rotation matrices", "[Mat3][transform]") {
    SECTION("Rotation X axis by 90 degrees") {
        Mat3 rot = Mat3::rotationX(static_cast<float>(M_PI) / 2.0f);
        Vec3 v(0.0f, 1.0f, 0.0f);
        Vec3 rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.z, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Rotation Y axis by 90 degrees") {
        Mat3 rot = Mat3::rotationY(static_cast<float>(M_PI) / 2.0f);
        Vec3 v(1.0f, 0.0f, 0.0f);
        Vec3 rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.z, WithinAbs(-1.0f, 1e-5f));
    }

    SECTION("Rotation Z axis by 90 degrees") {
        Mat3 rot = Mat3::rotationZ(static_cast<float>(M_PI) / 2.0f);
        Vec3 v(1.0f, 0.0f, 0.0f);
        Vec3 rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Rotation around arbitrary axis") {
        Vec3 axis(1.0f, 0.0f, 0.0f);
        Mat3 rot = Mat3::rotationAxis(axis, static_cast<float>(M_PI) / 2.0f);
        Vec3 v(0.0f, 1.0f, 0.0f);
        Vec3 rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.z, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Rotation matrices have determinant 1") {
        Mat3 rotX = Mat3::rotationX(0.5f);
        Mat3 rotY = Mat3::rotationY(0.7f);
        Mat3 rotZ = Mat3::rotationZ(0.3f);
        
        REQUIRE_THAT(rotX.determinant(), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotY.determinant(), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotZ.determinant(), WithinAbs(1.0f, 1e-5f));
    }
}

// ============================================================================
// Scale Matrices
// ============================================================================

TEST_CASE("Mat3: Scale matrix", "[Mat3][transform]") {
    SECTION("Non-uniform scale") {
        Mat3 scale = Mat3::scale(2.0f, 3.0f, 4.0f);
        Vec3 v(1.0f, 1.0f, 1.0f);
        Vec3 scaled = scale * v;
        
        REQUIRE_THAT(scaled.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(scaled.y, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(scaled.z, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Uniform scale") {
        Mat3 scale = Mat3::scale(5.0f);
        Vec3 v(2.0f, 3.0f, 4.0f);
        Vec3 scaled = scale * v;
        
        REQUIRE_THAT(scaled.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(scaled.y, WithinAbs(15.0f, 1e-6f));
        REQUIRE_THAT(scaled.z, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Scale matrix structure") {
        Mat3 scale = Mat3::scale(2.0f, 3.0f, 4.0f);
        
        REQUIRE_THAT(scale.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(scale.m[1][1], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(scale.m[2][2], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Scale determinant equals product") {
        Mat3 scale = Mat3::scale(2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(scale.determinant(), WithinAbs(24.0f, 1e-6f));
    }
}

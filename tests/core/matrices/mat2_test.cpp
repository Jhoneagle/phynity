#define _USE_MATH_DEFINES
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat2.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using phynity::math::matrices::Mat2;
using phynity::math::Vec2;
using Catch::Matchers::WithinAbs;

// Helper function to check if two matrices are approximately equal
bool mat2_approx_equal(const Mat2& a, const Mat2& b, float epsilon = 1e-4f) {
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

TEST_CASE("Mat2: Constructors", "[Mat2][constructor]") {
    SECTION("Default constructor creates identity matrix") {
        Mat2 m;
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Scalar constructor fills all elements") {
        Mat2 m(5.0f);
        REQUIRE_THAT(m.m[0][0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Element-wise constructor") {
        Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Column vector constructor") {
        Vec2 col0(1.0f, 3.0f);
        Vec2 col1(2.0f, 4.0f);
        Mat2 m(col0, col1);
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(4.0f, 1e-6f));
    }
}

TEST_CASE("Mat2: Static factory methods", "[Mat2][factory]") {
    SECTION("Identity matrix") {
        Mat2 m = Mat2::identity();
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Zero matrix") {
        Mat2 m = Mat2::zero();
        REQUIRE_THAT(m.m[0][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Mat2: Addition", "[Mat2][arithmetic]") {
    Mat2 a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 b(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2 c = a + b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(c.m[0][1], WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][0], WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(12.0f, 1e-6f));
}

TEST_CASE("Mat2: Subtraction", "[Mat2][arithmetic]") {
    Mat2 a(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2 b(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 c = a - b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.m[0][1], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat2: Scalar multiplication", "[Mat2][arithmetic]") {
    Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
    
    SECTION("Matrix * scalar") {
        Mat2 result = m * 2.0f;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[0][1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][0], WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][1], WithinAbs(8.0f, 1e-6f));
    }

    SECTION("Scalar * matrix") {
        Mat2 result = 2.0f * m;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[0][1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][0], WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.m[1][1], WithinAbs(8.0f, 1e-6f));
    }
}

TEST_CASE("Mat2: Scalar division", "[Mat2][arithmetic]") {
    Mat2 m(2.0f, 4.0f, 6.0f, 8.0f);
    Mat2 result = m / 2.0f;

    REQUIRE_THAT(result.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.m[0][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][0], WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result.m[1][1], WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat2: Matrix multiplication", "[Mat2][arithmetic]") {
    Mat2 m1(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 m2(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2 result = m1 * m2;

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

TEST_CASE("Mat2: Addition assignment", "[Mat2][assignment]") {
    Mat2 a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 b(5.0f, 6.0f, 7.0f, 8.0f);
    a += b;

    REQUIRE_THAT(a.m[0][0], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(a.m[0][1], WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(a.m[1][0], WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(a.m[1][1], WithinAbs(12.0f, 1e-6f));
}

TEST_CASE("Mat2: Subtraction assignment", "[Mat2][assignment]") {
    Mat2 a(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2 b(1.0f, 2.0f, 3.0f, 4.0f);
    a -= b;

    REQUIRE_THAT(a.m[0][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(a.m[0][1], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(a.m[1][0], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(a.m[1][1], WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat2: Scalar multiplication assignment", "[Mat2][assignment]") {
    Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
    m *= 2.0f;

    REQUIRE_THAT(m.m[0][0], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][1], WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][0], WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][1], WithinAbs(8.0f, 1e-6f));
}

TEST_CASE("Mat2: Scalar division assignment", "[Mat2][assignment]") {
    Mat2 m(2.0f, 4.0f, 6.0f, 8.0f);
    m /= 2.0f;

    REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][0], WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][1], WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat2: Matrix multiplication assignment", "[Mat2][assignment]") {
    Mat2 m1(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 m2(5.0f, 6.0f, 7.0f, 8.0f);
    m1 *= m2;

    REQUIRE_THAT(m1.m[0][0], WithinAbs(19.0f, 1e-6f));
    REQUIRE_THAT(m1.m[0][1], WithinAbs(22.0f, 1e-6f));
    REQUIRE_THAT(m1.m[1][0], WithinAbs(43.0f, 1e-6f));
    REQUIRE_THAT(m1.m[1][1], WithinAbs(50.0f, 1e-6f));
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Mat2: Equality comparison", "[Mat2][comparison]") {
    Mat2 a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 b(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 c(5.0f, 6.0f, 7.0f, 8.0f);

    REQUIRE(a == b);
    REQUIRE_FALSE(a == c);
}

TEST_CASE("Mat2: Inequality comparison", "[Mat2][comparison]") {
    Mat2 a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 b(5.0f, 6.0f, 7.0f, 8.0f);
    Mat2 c(1.0f, 2.0f, 3.0f, 4.0f);

    REQUIRE(a != b);
    REQUIRE_FALSE(a != c);
}

// ============================================================================
// Matrix-Vector Operations
// ============================================================================

TEST_CASE("Mat2: Matrix-vector multiplication", "[Mat2][vector]") {
    SECTION("Matrix * vector (column vector)") {
        Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
        Vec2 v(5.0f, 6.0f);
        Vec2 result = m * v;

        // [1*5 + 2*6] = [17]
        // [3*5 + 4*6] = [39]
        REQUIRE_THAT(result.x, WithinAbs(17.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(39.0f, 1e-6f));
    }

    SECTION("Vector * matrix (row vector)") {
        Vec2 v(5.0f, 6.0f);
        Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
        Vec2 result = v * m;

        // [5*1 + 6*3, 5*2 + 6*4] = [23, 34]
        REQUIRE_THAT(result.x, WithinAbs(23.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(34.0f, 1e-6f));
    }
}

// ============================================================================
// Matrix Operations
// ============================================================================

TEST_CASE("Mat2: Determinant", "[Mat2][operations]") {
    SECTION("Identity has determinant 1") {
        Mat2 m = Mat2::identity();
        REQUIRE_THAT(m.determinant(), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("General matrix determinant") {
        Mat2 m(3.0f, 8.0f, 4.0f, 6.0f);
        // det = 3*6 - 8*4 = 18 - 32 = -14
        REQUIRE_THAT(m.determinant(), WithinAbs(-14.0f, 1e-6f));
    }

    SECTION("Zero matrix has determinant 0") {
        Mat2 m = Mat2::zero();
        REQUIRE_THAT(m.determinant(), WithinAbs(0.0f, 1e-6f));
    }
}

TEST_CASE("Mat2: Inverse", "[Mat2][operations]") {
    SECTION("Identity inverse is identity") {
        Mat2 m = Mat2::identity();
        Mat2 inv = m.inverse();
        REQUIRE(mat2_approx_equal(inv, Mat2::identity()));
    }

    SECTION("General matrix inverse") {
        Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
        Mat2 inv = m.inverse();
        Mat2 product = m * inv;
        REQUIRE(mat2_approx_equal(product, Mat2::identity(), 1e-3f));
    }

    SECTION("Inverse of inverse returns original") {
        Mat2 m(2.0f, 3.0f, 1.0f, 4.0f);
        Mat2 inv = m.inverse();
        Mat2 inv_inv = inv.inverse();
        REQUIRE(mat2_approx_equal(inv_inv, m, 1e-3f));
    }

    SECTION("Singular matrix returns zero") {
        Mat2 m(1.0f, 2.0f, 2.0f, 4.0f); // Singular (det = 0)
        Mat2 inv = m.inverse();
        REQUIRE(mat2_approx_equal(inv, Mat2::zero()));
    }
}

TEST_CASE("Mat2: Transpose", "[Mat2][operations]") {
    SECTION("Transpose of identity is identity") {
        Mat2 m = Mat2::identity();
        Mat2 t = m.transposed();
        REQUIRE(mat2_approx_equal(t, Mat2::identity()));
    }

    SECTION("Transpose swaps rows and columns") {
        Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
        Mat2 t = m.transposed();
        
        REQUIRE_THAT(t.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(t.m[0][1], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(t.m[1][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(t.m[1][1], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("In-place transpose") {
        Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
        m.transpose();
        
        REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[0][1], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Double transpose returns original") {
        Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
        Mat2 tt = m.transposed().transposed();
        REQUIRE(mat2_approx_equal(tt, m));
    }
}

TEST_CASE("Mat2: Trace", "[Mat2][operations]") {
    SECTION("Trace of identity is 2") {
        Mat2 m = Mat2::identity();
        REQUIRE_THAT(m.trace(), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Trace is sum of diagonal") {
        Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(m.trace(), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Trace of zero is 0") {
        Mat2 m = Mat2::zero();
        REQUIRE_THAT(m.trace(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Element Access
// ============================================================================

TEST_CASE("Mat2: Element access", "[Mat2][access]") {
    Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);

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

// ============================================================================
// Transformation Matrices
// ============================================================================

TEST_CASE("Mat2: Rotation matrix", "[Mat2][transform]") {
    SECTION("90 degree rotation") {
        Mat2 rot = Mat2::rotation(static_cast<float>(M_PI) / 2.0f);
        Vec2 v(1.0f, 0.0f);
        Vec2 rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("45 degree rotation") {
        Mat2 rot = Mat2::rotation(static_cast<float>(M_PI) / 4.0f);
        float expected = std::sqrt(2.0f) / 2.0f;
        
        REQUIRE_THAT(rot.m[0][0], WithinAbs(expected, 1e-5f));
        REQUIRE_THAT(rot.m[0][1], WithinAbs(-expected, 1e-5f));
        REQUIRE_THAT(rot.m[1][0], WithinAbs(expected, 1e-5f));
        REQUIRE_THAT(rot.m[1][1], WithinAbs(expected, 1e-5f));
    }

    SECTION("180 degree rotation") {
        Mat2 rot = Mat2::rotation(static_cast<float>(M_PI));
        Vec2 v(1.0f, 0.0f);
        Vec2 rotated = rot * v;
        
        REQUIRE_THAT(rotated.x, WithinAbs(-1.0f, 1e-5f));
        REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Rotation matrix has determinant 1") {
        Mat2 rot = Mat2::rotation(static_cast<float>(M_PI) / 3.0f);
        REQUIRE_THAT(rot.determinant(), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("Mat2: Scale matrix", "[Mat2][transform]") {
    SECTION("Non-uniform scale") {
        Mat2 scale = Mat2::scale(2.0f, 3.0f);
        Vec2 v(1.0f, 1.0f);
        Vec2 scaled = scale * v;
        
        REQUIRE_THAT(scaled.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(scaled.y, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Uniform scale") {
        Mat2 scale = Mat2::scale(5.0f);
        Vec2 v(2.0f, 3.0f);
        Vec2 scaled = scale * v;
        
        REQUIRE_THAT(scaled.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(scaled.y, WithinAbs(15.0f, 1e-6f));
    }

    SECTION("Scale matrix structure") {
        Mat2 scale = Mat2::scale(2.0f, 3.0f);
        
        REQUIRE_THAT(scale.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(scale.m[0][1], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(scale.m[1][0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(scale.m[1][1], WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Scale determinant equals product") {
        Mat2 scale = Mat2::scale(2.0f, 3.0f);
        REQUIRE_THAT(scale.determinant(), WithinAbs(6.0f, 1e-6f));
    }
}

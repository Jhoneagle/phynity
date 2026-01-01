#define _USE_MATH_DEFINES
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat4.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using phynity::math::matrices::Mat4;
using phynity::math::vectors::Vec3;
using phynity::math::vectors::Vec4;
using Catch::Matchers::WithinAbs;

// Helper function to check if two matrices are approximately equal
bool mat4_approx_equal(const Mat4& a, const Mat4& b, float epsilon = 1e-4f) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
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

TEST_CASE("Mat4: Constructors", "[Mat4][constructor]") {
    SECTION("Default constructor creates identity matrix") {
        Mat4 m;
        REQUIRE(mat4_approx_equal(m, Mat4::identity()));
    }

    SECTION("Scalar constructor fills all elements") {
        Mat4 m(5.0f);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                REQUIRE_THAT(m.m[i][j], WithinAbs(5.0f, 1e-6f));
            }
        }
    }

    SECTION("Element-wise constructor") {
        Mat4 m(1.0f, 0.0f, 0.0f, 1.0f,
               0.0f, 1.0f, 0.0f, 2.0f,
               0.0f, 0.0f, 1.0f, 3.0f,
               0.0f, 0.0f, 0.0f, 1.0f);
        REQUIRE_THAT(m.m[0][3], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][3], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][3], WithinAbs(3.0f, 1e-6f));
    }
}

TEST_CASE("Mat4: Static factory methods", "[Mat4][factory]") {
    SECTION("Identity matrix") {
        Mat4 m = Mat4::identity();
        REQUIRE(mat4_approx_equal(m, Mat4::identity()));
    }

    SECTION("Zero matrix") {
        Mat4 m = Mat4::zero();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                REQUIRE_THAT(m.m[i][j], WithinAbs(0.0f, 1e-6f));
            }
        }
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Mat4: Matrix addition and subtraction", "[Mat4][arithmetic]") {
    Mat4 a = Mat4::identity();
    Mat4 b = Mat4::identity();
    Mat4 c = a + b;

    REQUIRE_THAT(c.m[0][0], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(c.m[1][1], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(c.m[2][2], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(c.m[3][3], WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("Mat4: Scalar multiplication", "[Mat4][arithmetic]") {
    Mat4 m = Mat4::identity();
    
    SECTION("Matrix * scalar") {
        Mat4 result = m * 2.0f;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[3][3], WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Scalar * matrix") {
        Mat4 result = 2.0f * m;
        REQUIRE_THAT(result.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.m[3][3], WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("Mat4: Matrix multiplication", "[Mat4][arithmetic]") {
    Mat4 a = Mat4::identity();
    Mat4 b = Mat4::identity();
    Mat4 result = a * b;

    REQUIRE(mat4_approx_equal(result, Mat4::identity()));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Mat4: Assignment operators", "[Mat4][assignment]") {
    Mat4 m = Mat4::identity();
    Mat4 a = m;

    SECTION("Scalar multiplication assignment") {
        a *= 2.0f;
        REQUIRE_THAT(a.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(a.m[3][3], WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Matrix multiplication assignment") {
        Mat4 b = Mat4::scale(2.0f);
        a *= b;
        REQUIRE(mat4_approx_equal(a, Mat4::scale(2.0f)));
    }
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Mat4: Comparison operators", "[Mat4][comparison]") {
    Mat4 a = Mat4::identity();
    Mat4 b = Mat4::identity();
    Mat4 c = Mat4::zero();

    REQUIRE(a == b);
    REQUIRE(a != c);
}

// ============================================================================
// Matrix-Vector Operations
// ============================================================================

TEST_CASE("Mat4: Matrix-vector multiplication", "[Mat4][vector]") {
    SECTION("Matrix * Vec4") {
        Mat4 m = Mat4::identity();
        Vec4 v(1.0f, 2.0f, 3.0f, 4.0f);
        Vec4 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.w, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Matrix * Vec3 (homogeneous)") {
        Mat4 m = Mat4::translation(1.0f, 2.0f, 3.0f);
        Vec3 v(5.0f, 6.0f, 7.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(10.0f, 1e-6f));
    }

    SECTION("Vec4 * Matrix (row vector)") {
        Mat4 m = Mat4::identity();
        Vec4 v(1.0f, 2.0f, 3.0f, 4.0f);
        Vec4 result = v * m;

        REQUIRE_THAT(result.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.w, WithinAbs(4.0f, 1e-6f));
    }
}

// ============================================================================
// Matrix Operations
// ============================================================================

TEST_CASE("Mat4: Determinant", "[Mat4][operations]") {
    SECTION("Identity has determinant 1") {
        Mat4 m = Mat4::identity();
        REQUIRE_THAT(m.determinant(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Zero matrix has determinant 0") {
        Mat4 m = Mat4::zero();
        REQUIRE_THAT(m.determinant(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Scale matrix determinant") {
        Mat4 m = Mat4::scale(2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(m.determinant(), WithinAbs(24.0f, 1e-5f));
    }
}

TEST_CASE("Mat4: Inverse", "[Mat4][operations]") {
    SECTION("Identity inverse is identity") {
        Mat4 m = Mat4::identity();
        Mat4 inv = m.inverse();
        REQUIRE(mat4_approx_equal(inv, Mat4::identity()));
    }

    SECTION("Translation matrix inverse") {
        Mat4 m = Mat4::translation(5.0f, 6.0f, 7.0f);
        Mat4 inv = m.inverse();
        Mat4 product = m * inv;
        REQUIRE(mat4_approx_equal(product, Mat4::identity(), 1e-3f));
    }

    SECTION("Scale matrix inverse") {
        Mat4 m = Mat4::scale(2.0f, 3.0f, 4.0f);
        Mat4 inv = m.inverse();
        Mat4 product = m * inv;
        REQUIRE(mat4_approx_equal(product, Mat4::identity(), 1e-3f));
    }

    SECTION("Rotation matrix inverse (is transpose)") {
        Mat4 m = Mat4::rotationZ(0.5f);
        Mat4 inv = m.inverse();
        Mat4 trans = m.transposed();
        REQUIRE(mat4_approx_equal(inv, trans, 1e-3f));
    }
}

TEST_CASE("Mat4: Transpose", "[Mat4][operations]") {
    SECTION("Double transpose returns original") {
        Mat4 m = Mat4::translation(1.0f, 2.0f, 3.0f);
        Mat4 tt = m.transposed().transposed();
        REQUIRE(mat4_approx_equal(tt, m));
    }

    SECTION("In-place transpose") {
        Mat4 m(1.0f, 2.0f, 3.0f, 4.0f,
               5.0f, 6.0f, 7.0f, 8.0f,
               9.0f, 10.0f, 11.0f, 12.0f,
               13.0f, 14.0f, 15.0f, 16.0f);
        m.transpose();
        REQUIRE_THAT(m.m[0][1], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][0], WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("Mat4: Trace", "[Mat4][operations]") {
    SECTION("Trace of identity is 4") {
        Mat4 m = Mat4::identity();
        REQUIRE_THAT(m.trace(), WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Trace of zero is 0") {
        Mat4 m = Mat4::zero();
        REQUIRE_THAT(m.trace(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Element Access
// ============================================================================

TEST_CASE("Mat4: Element access", "[Mat4][access]") {
    Mat4 m = Mat4::identity();

    SECTION("Read access") {
        REQUIRE_THAT(m(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m(3, 3), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m(0, 3), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Write access") {
        m(0, 3) = 5.0f;
        REQUIRE_THAT(m(0, 3), WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("Mat4: Row and column accessors", "[Mat4][access][rowcol]") {
    Mat4 m(1.0f, 2.0f, 3.0f, 4.0f,
           5.0f, 6.0f, 7.0f, 8.0f,
           9.0f, 10.0f, 11.0f, 12.0f,
           13.0f, 14.0f, 15.0f, 16.0f);

    Vec4 row2 = m.getRow(2);
    REQUIRE_THAT(row2.x, WithinAbs(9.0f, 1e-6f));
    REQUIRE_THAT(row2.y, WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(row2.z, WithinAbs(11.0f, 1e-6f));
    REQUIRE_THAT(row2.w, WithinAbs(12.0f, 1e-6f));

    Vec4 col3 = m.getColumn(3);
    REQUIRE_THAT(col3.x, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(col3.y, WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(col3.z, WithinAbs(12.0f, 1e-6f));
    REQUIRE_THAT(col3.w, WithinAbs(16.0f, 1e-6f));

    Vec4 newRow(16.0f, 15.0f, 14.0f, 13.0f);
    m.setRow(0, newRow);
    REQUIRE_THAT(m.m[0][0], WithinAbs(16.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][3], WithinAbs(13.0f, 1e-6f));

    Vec4 newCol(1.0f, 2.0f, 3.0f, 4.0f);
    m.setColumn(2, newCol);
    REQUIRE_THAT(m.m[1][2], WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(m.m[3][2], WithinAbs(4.0f, 1e-6f));
}

// ============================================================================
// Translation Matrices
// ============================================================================

TEST_CASE("Mat4: Translation matrix", "[Mat4][transform]") {
    SECTION("Translation with floats") {
        Mat4 m = Mat4::translation(1.0f, 2.0f, 3.0f);
        Vec3 v(0.0f, 0.0f, 0.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Translation with Vec3") {
        Vec3 t(5.0f, 6.0f, 7.0f);
        Mat4 m = Mat4::translation(t);
        Vec3 v(1.0f, 1.0f, 1.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(7.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(8.0f, 1e-6f));
    }

    SECTION("Translation matrix structure") {
        Mat4 m = Mat4::translation(1.0f, 2.0f, 3.0f);
        REQUIRE_THAT(m.m[0][3], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][3], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][3], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m.m[3][3], WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Scale Matrices
// ============================================================================

TEST_CASE("Mat4: Scale matrix", "[Mat4][transform]") {
    SECTION("Non-uniform scale") {
        Mat4 m = Mat4::scale(2.0f, 3.0f, 4.0f);
        Vec3 v(1.0f, 1.0f, 1.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Uniform scale") {
        Mat4 m = Mat4::scale(5.0f);
        Vec3 v(2.0f, 3.0f, 4.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(15.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Scale with Vec3") {
        Vec3 s(2.0f, 3.0f, 4.0f);
        Mat4 m = Mat4::scale(s);
        REQUIRE_THAT(m.m[0][0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m.m[1][1], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m.m[2][2], WithinAbs(4.0f, 1e-6f));
    }
}

// ============================================================================
// Rotation Matrices
// ============================================================================

TEST_CASE("Mat4: Rotation matrices", "[Mat4][transform]") {
    SECTION("Rotation X by 90 degrees") {
        Mat4 m = Mat4::rotationX(static_cast<float>(M_PI) / 2.0f);
        Vec3 v(0.0f, 1.0f, 0.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Rotation Y by 90 degrees") {
        Mat4 m = Mat4::rotationY(static_cast<float>(M_PI) / 2.0f);
        Vec3 v(1.0f, 0.0f, 0.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(-1.0f, 1e-5f));
    }

    SECTION("Rotation Z by 90 degrees") {
        Mat4 m = Mat4::rotationZ(static_cast<float>(M_PI) / 2.0f);
        Vec3 v(1.0f, 0.0f, 0.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Rotation around arbitrary axis") {
        Vec3 axis(1.0f, 0.0f, 0.0f);
        Mat4 m = Mat4::rotationAxis(axis, static_cast<float>(M_PI) / 2.0f);
        Vec3 v(0.0f, 1.0f, 0.0f);
        Vec3 result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Rotation matrices have determinant 1") {
        Mat4 rotX = Mat4::rotationX(0.5f);
        Mat4 rotY = Mat4::rotationY(0.7f);
        Mat4 rotZ = Mat4::rotationZ(0.3f);

        REQUIRE_THAT(rotX.determinant(), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotY.determinant(), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotZ.determinant(), WithinAbs(1.0f, 1e-5f));
    }
}

// ============================================================================
// Projection Matrices
// ============================================================================

TEST_CASE("Mat4: Perspective projection", "[Mat4][projection]") {
    SECTION("Perspective matrix creation") {
        Mat4 m = Mat4::perspective(static_cast<float>(M_PI) / 4.0f, 16.0f / 9.0f, 0.1f, 100.0f);
        REQUIRE_THAT(m.m[3][2], WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(m.m[3][3], WithinAbs(0.0f, 1e-6f));
    }
}

TEST_CASE("Mat4: Orthographic projection", "[Mat4][projection]") {
    SECTION("Orthographic matrix creation") {
        Mat4 m = Mat4::orthographic(-1.0f, 1.0f, -1.0f, 1.0f, 0.1f, 100.0f);
        REQUIRE_THAT(m.m[3][3], WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Combined Transformations
// ============================================================================

TEST_CASE("Mat4: Combined transformations", "[Mat4][combined]") {
    SECTION("Translation then scale") {
        Mat4 t = Mat4::translation(1.0f, 2.0f, 3.0f);
        Mat4 s = Mat4::scale(2.0f);
        Mat4 m = t * s;
        
        Vec3 v(1.0f, 1.0f, 1.0f);
        Vec3 result = m * v;
        
        // First scaled: (2, 2, 2)
        // Then translated: (3, 4, 5)
        REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-5f));
    }

    SECTION("Scale then translation") {
        Mat4 s = Mat4::scale(2.0f);
        Mat4 t = Mat4::translation(1.0f, 2.0f, 3.0f);
        Mat4 m = t * s;
        
        Vec3 v(1.0f, 1.0f, 1.0f);
        Vec3 result = m * v;
        
        // First scaled: (2, 2, 2)
        // Then translated: (3, 4, 5)
        REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-5f));
    }
}

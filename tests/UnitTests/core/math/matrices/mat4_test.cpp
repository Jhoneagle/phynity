#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/matrices/mat4.hpp>
#include <core/math/utilities/constants.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <cmath>
#include <sstream>

using phynity::math::matrices::Mat4f;
using phynity::math::vectors::Vec3f;
using phynity::math::vectors::Vec4f;
using phynity::math::utilities::mathf;
using phynity::math::utilities::approx_equal;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Constructors and Basic Properties
// ============================================================================

TEST_CASE("Mat4f: Constructors", "[Mat4f][constructor]") {
    SECTION("Default constructor creates identity matrix") {
        Mat4f m;
        REQUIRE(approx_equal(m, Mat4f::identity()));
    }

    SECTION("Scalar constructor fills all elements") {
        Mat4f m(5.0f);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                REQUIRE_THAT(m(i, j), WithinAbs(5.0f, 1e-6f));
            }
        }
    }

    SECTION("Element-wise constructor") {
        Mat4f m(1.0f, 0.0f, 0.0f, 1.0f,
               0.0f, 1.0f, 0.0f, 2.0f,
               0.0f, 0.0f, 1.0f, 3.0f,
               0.0f, 0.0f, 0.0f, 1.0f);
        REQUIRE_THAT(m(0, 3), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m(1, 3), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(2, 3), WithinAbs(3.0f, 1e-6f));
    }
}

TEST_CASE("Mat4f: Static factory methods", "[Mat4f][factory]") {
    SECTION("Identity matrix") {
        Mat4f m = Mat4f::identity();
        REQUIRE(approx_equal(m, Mat4f::identity()));
    }

    SECTION("Zero matrix") {
        Mat4f m = Mat4f::zero();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                REQUIRE_THAT(m(i, j), WithinAbs(0.0f, 1e-6f));
            }
        }
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Mat4f: Matrix addition and subtraction", "[Mat4f][arithmetic]") {
    Mat4f a = Mat4f::identity();
    Mat4f b = Mat4f::identity();
    Mat4f c = a + b;

    REQUIRE_THAT(c(0, 0), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(c(1, 1), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(c(2, 2), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(c(3, 3), WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("Mat4f: Scalar multiplication", "[Mat4f][arithmetic]") {
    Mat4f m = Mat4f::identity();
    
    SECTION("Matrix * scalar") {
        Mat4f result = m * 2.0f;
        REQUIRE_THAT(result(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result(3, 3), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Scalar * matrix") {
        Mat4f result = 2.0f * m;
        REQUIRE_THAT(result(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result(3, 3), WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("Mat4f: Negation", "[Mat4f][arithmetic]") {
    Mat4f m = Mat4f::identity();
    m(0, 1) = -2.0f;
    m(3, 3) = 5.0f;
    
    Mat4f result = -m;

    REQUIRE_THAT(result(0, 0), WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(result(0, 1), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result(3, 3), WithinAbs(-5.0f, 1e-6f));

    // Double negation
    Mat4f double_neg = -(-m);
    REQUIRE(approx_equal(double_neg, m));
}

TEST_CASE("Mat4f: Matrix multiplication", "[Mat4f][arithmetic]") {
    Mat4f a = Mat4f::identity();
    Mat4f b = Mat4f::identity();
    Mat4f result = a * b;

    REQUIRE(approx_equal(result, Mat4f::identity()));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Mat4f: Assignment operators", "[Mat4f][assignment]") {
    Mat4f m = Mat4f::identity();
    Mat4f a = m;

    SECTION("Scalar multiplication assignment") {
        a *= 2.0f;
        REQUIRE_THAT(a(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(a(3, 3), WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Matrix multiplication assignment") {
        Mat4f b = Mat4f::scale(2.0f);
        a *= b;
        REQUIRE(approx_equal(a, Mat4f::scale(2.0f)));
    }
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Mat4f: Comparison operators", "[Mat4f][comparison]") {
    Mat4f a = Mat4f::identity();
    Mat4f b = Mat4f::identity();
    Mat4f c = Mat4f::zero();

    REQUIRE(a == b);
    REQUIRE(a != c);
}

// ============================================================================
// Matrix-Vector Operations
// ============================================================================

TEST_CASE("Mat4f: Matrix-vector multiplication", "[Mat4f][vector]") {
    SECTION("Matrix * Vec4") {
        Mat4f m = Mat4f::identity();
        Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
        Vec4f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.w, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Matrix * Vec3f (homogeneous)") {
        Mat4f m = Mat4f::translation(1.0f, 2.0f, 3.0f);
        Vec3f v(5.0f, 6.0f, 7.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(10.0f, 1e-6f));
    }

    SECTION("Vec4f * Matrix (row vector)") {
        Mat4f m = Mat4f::identity();
        Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
        Vec4f result = v * m;

        REQUIRE_THAT(result.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.w, WithinAbs(4.0f, 1e-6f));
    }
}

// ============================================================================
// Matrix Operations
// ============================================================================

TEST_CASE("Mat4f: Determinant", "[Mat4f][operations]") {
    SECTION("Identity has determinant 1") {
        Mat4f m = Mat4f::identity();
        REQUIRE_THAT(m.determinant(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Zero matrix has determinant 0") {
        Mat4f m = Mat4f::zero();
        REQUIRE_THAT(m.determinant(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Scale matrix determinant") {
        Mat4f m = Mat4f::scale(2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(m.determinant(), WithinAbs(24.0f, 1e-5f));
    }
}

TEST_CASE("Mat4f: Inverse", "[Mat4f][operations]") {
    SECTION("Identity inverse is identity") {
        Mat4f m = Mat4f::identity();
        Mat4f inv = m.inverse();
        REQUIRE(approx_equal(inv, Mat4f::identity()));
    }

    SECTION("Translation matrix inverse") {
        Mat4f m = Mat4f::translation(5.0f, 6.0f, 7.0f);
        Mat4f inv = m.inverse();
        Mat4f product = m * inv;
        REQUIRE(approx_equal(product, Mat4f::identity(), 1e-3f));
    }

    SECTION("Scale matrix inverse") {
        Mat4f m = Mat4f::scale(2.0f, 3.0f, 4.0f);
        Mat4f inv = m.inverse();
        Mat4f product = m * inv;
        REQUIRE(approx_equal(product, Mat4f::identity(), 1e-3f));
    }

    SECTION("Rotation matrix inverse (is transpose)") {
        Mat4f m = Mat4f::rotationZ(0.5f);
        Mat4f inv = m.inverse();
        Mat4f trans = m.transposed();
        REQUIRE(approx_equal(inv, trans, 1e-3f));
    }
}

TEST_CASE("Mat4f: Transpose", "[Mat4f][operations]") {
    SECTION("Double transpose returns original") {
        Mat4f m = Mat4f::translation(1.0f, 2.0f, 3.0f);
        Mat4f tt = m.transposed().transposed();
        REQUIRE(approx_equal(tt, m));
    }

    SECTION("In-place transpose") {
        Mat4f m(1.0f, 2.0f, 3.0f, 4.0f,
               5.0f, 6.0f, 7.0f, 8.0f,
               9.0f, 10.0f, 11.0f, 12.0f,
               13.0f, 14.0f, 15.0f, 16.0f);
        m.transpose();
        REQUIRE_THAT(m(0, 1), WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(m(1, 0), WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("Mat4f: Trace", "[Mat4f][operations]") {
    SECTION("Trace of identity is 4") {
        Mat4f m = Mat4f::identity();
        REQUIRE_THAT(m.trace(), WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Trace of zero is 0") {
        Mat4f m = Mat4f::zero();
        REQUIRE_THAT(m.trace(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Element Access
// ============================================================================

TEST_CASE("Mat4f: Element access", "[Mat4f][access]") {
    Mat4f m = Mat4f::identity();

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

TEST_CASE("Mat4f: Row and column accessors", "[Mat4f][access][rowcol]") {
    Mat4f m(1.0f, 2.0f, 3.0f, 4.0f,
           5.0f, 6.0f, 7.0f, 8.0f,
           9.0f, 10.0f, 11.0f, 12.0f,
           13.0f, 14.0f, 15.0f, 16.0f);

    Vec4f row2 = m.getRow(2);
    REQUIRE_THAT(row2.x, WithinAbs(9.0f, 1e-6f));
    REQUIRE_THAT(row2.y, WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(row2.z, WithinAbs(11.0f, 1e-6f));
    REQUIRE_THAT(row2.w, WithinAbs(12.0f, 1e-6f));

    Vec4f col3 = m.getColumn(3);
    REQUIRE_THAT(col3.x, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(col3.y, WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(col3.z, WithinAbs(12.0f, 1e-6f));
    REQUIRE_THAT(col3.w, WithinAbs(16.0f, 1e-6f));

    Vec4f newRow(16.0f, 15.0f, 14.0f, 13.0f);
    m.setRow(0, newRow);
    REQUIRE_THAT(m(0, 0), WithinAbs(16.0f, 1e-6f));
    REQUIRE_THAT(m(0, 3), WithinAbs(13.0f, 1e-6f));

    Vec4f newCol(1.0f, 2.0f, 3.0f, 4.0f);
    m.setColumn(2, newCol);
    REQUIRE_THAT(m(1, 2), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(m(3, 2), WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Mat4f: Approximate equality", "[Mat4f][comparison]") {
    Mat4f m1 = Mat4f::identity();
    Mat4f m2 = Mat4f::identity();
    Mat4f m3 = Mat4f::identity();
    m3(2, 2) = 1.000001f;
    Mat4f m4 = Mat4f::translation(1.0f, 2.0f, 3.0f);

    REQUIRE(m1.approxEqual(m2));
    REQUIRE(m1.approxEqual(m3, 1e-5f));
    REQUIRE_FALSE(m1.approxEqual(m4, 1e-6f));
    REQUIRE(m1.approxEqual(m4, 4.0f));
}

TEST_CASE("Mat4f: Absolute value", "[Mat4f][operations]") {
    Mat4f m = Mat4f::identity();
    m(0, 3) = -5.0f;
    m(1, 2) = -3.0f;
    m(2, 1) = -2.0f;
    m(3, 0) = -1.0f;

    Mat4f result = m.abs();

    REQUIRE_THAT(result(0, 0), WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result(0, 3), WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result(1, 2), WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result(2, 1), WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result(3, 0), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Mat4f: Component-wise operations", "[Mat4f][arithmetic]") {
    Mat4f m1 = Mat4f::identity();
    m1(0, 0) = 4.0f; m1(1, 1) = 6.0f; m1(2, 2) = 8.0f; m1(3, 3) = 10.0f;
    Mat4f m2 = Mat4f::identity();
    m2(0, 0) = 2.0f; m2(1, 1) = 3.0f; m2(2, 2) = 4.0f; m2(3, 3) = 5.0f;

    SECTION("Component-wise multiplication") {
        Mat4f m = m1;
        m.mulComponentWise(m2);
        REQUIRE_THAT(m(0, 0), WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(18.0f, 1e-6f));
        REQUIRE_THAT(m(2, 2), WithinAbs(32.0f, 1e-6f));
        REQUIRE_THAT(m(3, 3), WithinAbs(50.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        Mat4f m = m1;
        m.divComponentWise(m2);
        REQUIRE_THAT(m(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(2, 2), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(3, 3), WithinAbs(2.0f, 1e-6f));
    }
}

// ============================================================================
// Translation Matrices
// ============================================================================

TEST_CASE("Mat4f: Translation matrix", "[Mat4f][transform]") {
    SECTION("Translation with floats") {
        Mat4f m = Mat4f::translation(1.0f, 2.0f, 3.0f);
        Vec3f v(0.0f, 0.0f, 0.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Translation with Vec3") {
        Vec3f t(5.0f, 6.0f, 7.0f);
        Mat4f m = Mat4f::translation(t);
        Vec3f v(1.0f, 1.0f, 1.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(7.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(8.0f, 1e-6f));
    }

    SECTION("Translation matrix structure") {
        Mat4f m = Mat4f::translation(1.0f, 2.0f, 3.0f);
        REQUIRE_THAT(m(0, 3), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(m(1, 3), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(2, 3), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m(3, 3), WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Scale Matrices
// ============================================================================

TEST_CASE("Mat4f: Scale matrix", "[Mat4f][transform]") {
    SECTION("Non-uniform scale") {
        Mat4f m = Mat4f::scale(2.0f, 3.0f, 4.0f);
        Vec3f v(1.0f, 1.0f, 1.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Uniform scale") {
        Mat4f m = Mat4f::scale(5.0f);
        Vec3f v(2.0f, 3.0f, 4.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(15.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Scale with Vec3") {
        Vec3f s(2.0f, 3.0f, 4.0f);
        Mat4f m = Mat4f::scale(s);
        REQUIRE_THAT(m(0, 0), WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(m(1, 1), WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(m(2, 2), WithinAbs(4.0f, 1e-6f));
    }
}

// ============================================================================
// Rotation Matrices
// ============================================================================

TEST_CASE("Mat4f: Rotation matrices", "[Mat4f][transform]") {
    SECTION("Rotation X by 90 degrees") {
        Mat4f m = Mat4f::rotationX(static_cast<float>(mathf::pi) / 2.0f);
        Vec3f v(0.0f, 1.0f, 0.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Rotation Y by 90 degrees") {
        Mat4f m = Mat4f::rotationY(static_cast<float>(mathf::pi) / 2.0f);
        Vec3f v(1.0f, 0.0f, 0.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(-1.0f, 1e-5f));
    }

    SECTION("Rotation Z by 90 degrees") {
        Mat4f m = Mat4f::rotationZ(static_cast<float>(mathf::pi) / 2.0f);
        Vec3f v(1.0f, 0.0f, 0.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Rotation around arbitrary axis") {
        Vec3f axis(1.0f, 0.0f, 0.0f);
        Mat4f m = Mat4f::rotationAxis(axis, static_cast<float>(mathf::pi) / 2.0f);
        Vec3f v(0.0f, 1.0f, 0.0f);
        Vec3f result = m * v;

        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Rotation matrices have determinant 1") {
        Mat4f rotX = Mat4f::rotationX(0.5f);
        Mat4f rotY = Mat4f::rotationY(0.7f);
        Mat4f rotZ = Mat4f::rotationZ(0.3f);

        REQUIRE_THAT(rotX.determinant(), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotY.determinant(), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(rotZ.determinant(), WithinAbs(1.0f, 1e-5f));
    }
}

// ============================================================================
// Projection Matrices
// ============================================================================

TEST_CASE("Mat4f: Perspective projection", "[Mat4f][projection]") {
    SECTION("Perspective matrix creation") {
        Mat4f m = Mat4f::perspective(static_cast<float>(mathf::pi) / 4.0f, 16.0f / 9.0f, 0.1f, 100.0f);
        REQUIRE_THAT(m(3, 2), WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(m(3, 3), WithinAbs(0.0f, 1e-6f));
    }
}

TEST_CASE("Mat4f: Orthographic projection", "[Mat4f][projection]") {
    SECTION("Orthographic matrix creation") {
        Mat4f m = Mat4f::orthographic(-1.0f, 1.0f, -1.0f, 1.0f, 0.1f, 100.0f);
        REQUIRE_THAT(m(3, 3), WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Combined Transformations
// ============================================================================

TEST_CASE("Mat4f: Combined transformations", "[Mat4f][combined]") {
    SECTION("Translation then scale") {
        Mat4f t = Mat4f::translation(1.0f, 2.0f, 3.0f);
        Mat4f s = Mat4f::scale(2.0f);
        Mat4f m = t * s;
        
        Vec3f v(1.0f, 1.0f, 1.0f);
        Vec3f result = m * v;
        
        // First scaled: (2, 2, 2)
        // Then translated: (3, 4, 5)
        REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-5f));
    }

    SECTION("Scale then translation") {
        Mat4f s = Mat4f::scale(2.0f);
        Mat4f t = Mat4f::translation(1.0f, 2.0f, 3.0f);
        Mat4f m = t * s;
        
        Vec3f v(1.0f, 1.0f, 1.0f);
        Vec3f result = m * v;
        
        // First scaled: (2, 2, 2)
        // Then translated: (3, 4, 5)
        REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-5f));
    }
}

// ============================================================================
// Stream Output
// ============================================================================

TEST_CASE("Mat4f: Stream output", "[Mat4f][stream]") {
    Mat4f m(1.0f, 2.0f, 3.0f, 4.0f,
           5.0f, 6.0f, 7.0f, 8.0f,
           9.0f, 10.0f, 11.0f, 12.0f,
           13.0f, 14.0f, 15.0f, 16.0f);
    std::ostringstream oss;
    oss << m;
    std::string output = oss.str();
    REQUIRE(output.find("1") != std::string::npos);
    REQUIRE(output.find("6") != std::string::npos);
    REQUIRE(output.find("11") != std::string::npos);
    REQUIRE(output.find("16") != std::string::npos);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("Mat4f: Edge cases", "[Mat4f][edge]") {
    SECTION("Division by zero scalar") {
        Mat4f m = Mat4f::identity();
        Mat4f result = m / 0.0f;
        REQUIRE((std::isinf(result(0, 0)) || std::isnan(result(0, 0))));
    }

    SECTION("Operations with very large numbers") {
        Mat4f m(1e20f);
        Mat4f sum = m + m;
        REQUIRE(sum(2, 2) > 1e20f);
    }

    SECTION("Operations with very small numbers") {
        Mat4f m(1e-20f);
        float trace = m.trace();
        REQUIRE(trace > 0.0f);
    }

    SECTION("Singular matrix returns zero inverse") {
        Mat4f m = Mat4f::zero();
        Mat4f inv = m.inverse();
        REQUIRE(approx_equal(inv, Mat4f::zero()));
    }

    SECTION("Identity matrix inverse is itself") {
        Mat4f identity = Mat4f::identity();
        Mat4f inv = identity.inverse();
        REQUIRE(approx_equal(inv, identity));
    }

    SECTION("Matrix times inverse is identity") {
        Mat4f m(2.0f, 0.0f, 0.0f, 0.0f,
               0.0f, 3.0f, 0.0f, 0.0f,
               0.0f, 0.0f, 4.0f, 0.0f,
               0.0f, 0.0f, 0.0f, 5.0f);
        Mat4f inv = m.inverse();
        Mat4f result = m * inv;
        REQUIRE(approx_equal(result, Mat4f::identity(), 1e-5f));
    }

    SECTION("Transpose properties") {
        Mat4f m = Mat4f::identity();
        Mat4f t = m.transposed();
        REQUIRE(approx_equal(t, m));
    }
}

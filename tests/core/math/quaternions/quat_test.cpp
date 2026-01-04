#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/utilities/constants.hpp>
#include <cmath>

using namespace phynity::math::quaternions;
using namespace phynity::math::vectors;
using namespace phynity::math::utilities;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRelMatcher;

// ============================================================================
// Constructors
// ============================================================================

TEST_CASE("Quat: Default constructor", "[Quat][constructor]") {
    Quat q;
    REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Quat: Component constructor", "[Quat][constructor]") {
    Quat q(0.707f, 0.707f, 0.0f, 0.0f);
    REQUIRE_THAT(q.w, WithinAbs(0.707f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(0.707f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Quat: Axis-angle constructor", "[Quat][constructor]") {
    SECTION("90° rotation around Z-axis") {
        Vec3 axis(0.0f, 0.0f, 1.0f);
        float angle = static_cast<float>(mathf::pi / 2.0);  // 90 degrees
        Quat q(axis, angle);

        float expectedW = std::cos(angle / 2.0f);
        float expectedXYZ = std::sin(angle / 2.0f);

        REQUIRE_THAT(q.w, WithinAbs(expectedW, 1e-6f));
        REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(q.z, WithinAbs(expectedXYZ, 1e-6f));
    }

    SECTION("180° rotation around X-axis") {
        Vec3 axis(1.0f, 0.0f, 0.0f);
        float angle = static_cast<float>(mathf::pi);
        Quat q(axis, angle);

        REQUIRE_THAT(q.w, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(q.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Zero rotation") {
        Vec3 axis(1.0f, 0.0f, 0.0f);
        float angle = 0.0f;
        Quat q(axis, angle);

        REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Quat: Addition", "[Quat][arithmetic]") {
    Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
    Quat result = q1 + q2;

    REQUIRE_THAT(result.w, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(result.x, WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(12.0f, 1e-6f));
}

TEST_CASE("Quat: Subtraction", "[Quat][arithmetic]") {
    Quat q1(5.0f, 8.0f, 10.0f, 12.0f);
    Quat q2(1.0f, 2.0f, 3.0f, 4.0f);
    Quat result = q1 - q2;

    REQUIRE_THAT(result.w, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result.x, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(7.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(8.0f, 1e-6f));
}

TEST_CASE("Quat: Scalar multiplication", "[Quat][arithmetic]") {
    SECTION("Direct multiplication") {
        Quat q(2.0f, 3.0f, 4.0f, 5.0f);
        Quat result = q * 2.0f;

        REQUIRE_THAT(result.w, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(10.0f, 1e-6f));
    }

    SECTION("Reverse multiplication") {
        Quat q(2.0f, 3.0f, 4.0f, 5.0f);
        Quat result = 3.0f * q;

        REQUIRE_THAT(result.w, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(result.x, WithinAbs(9.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(12.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(15.0f, 1e-6f));
    }
}

TEST_CASE("Quat: Scalar division", "[Quat][arithmetic]") {
    Quat q(4.0f, 6.0f, 8.0f, 10.0f);
    Quat result = q / 2.0f;

    REQUIRE_THAT(result.w, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("Quat: Negation", "[Quat][arithmetic]") {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    Quat result = -q;

    REQUIRE_THAT(result.w, WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(result.x, WithinAbs(-2.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(-3.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(-4.0f, 1e-6f));
}

TEST_CASE("Quat: Quaternion multiplication (basic)", "[Quat][arithmetic]") {
    SECTION("Identity quaternion is multiplicative identity") {
        Quat identity(1.0f, 0.0f, 0.0f, 0.0f);
        Quat q(0.5f, 0.5f, 0.5f, 0.5f);
        Quat result = identity * q;

        REQUIRE_THAT(result.w, WithinAbs(q.w, 1e-6f));
        REQUIRE_THAT(result.x, WithinAbs(q.x, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(q.y, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(q.z, 1e-6f));
    }

    SECTION("q * q⁻¹ ≈ identity") {
        Quat q(0.5f, 0.5f, 0.5f, 0.5f);
        Quat qInv = q.inverse();
        Quat result = q * qInv;

        REQUIRE_THAT(result.w, WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(result.z, WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Non-commutative: q1 * q2 ≠ q2 * q1 (in general)") {
        Quat q1(1.0f, 1.0f, 0.0f, 0.0f);
        Quat q2(1.0f, 0.0f, 1.0f, 0.0f);
        Quat result1 = q1 * q2;
        Quat result2 = q2 * q1;

        // They should differ (not always identity)
        bool different = (result1.w != result2.w) || (result1.x != result2.x) ||
                         (result1.y != result2.y) || (result1.z != result2.z);
        REQUIRE(different);
    }
}

// ============================================================================
// Compound Assignment Operators
// ============================================================================

TEST_CASE("Quat: Compound addition (+=)", "[Quat][compound]") {
    Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quat q2(5.0f, 6.0f, 7.0f, 8.0f);
    q1 += q2;

    REQUIRE_THAT(q1.w, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(q1.x, WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(q1.y, WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(q1.z, WithinAbs(12.0f, 1e-6f));
}

TEST_CASE("Quat: Compound subtraction (-=)", "[Quat][compound]") {
    Quat q1(5.0f, 8.0f, 10.0f, 12.0f);
    Quat q2(1.0f, 2.0f, 3.0f, 4.0f);
    q1 -= q2;

    REQUIRE_THAT(q1.w, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(q1.x, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(q1.y, WithinAbs(7.0f, 1e-6f));
    REQUIRE_THAT(q1.z, WithinAbs(8.0f, 1e-6f));
}

TEST_CASE("Quat: Compound multiplication (*=)", "[Quat][compound]") {
    Quat q1(0.707f, 0.707f, 0.0f, 0.0f);
    Quat q2(0.707f, 0.707f, 0.0f, 0.0f);
    Quat expected = q1 * q2;
    q1 *= q2;

    REQUIRE_THAT(q1.w, WithinAbs(expected.w, 1e-5f));
    REQUIRE_THAT(q1.x, WithinAbs(expected.x, 1e-5f));
    REQUIRE_THAT(q1.y, WithinAbs(expected.y, 1e-5f));
    REQUIRE_THAT(q1.z, WithinAbs(expected.z, 1e-5f));
}

TEST_CASE("Quat: Compound scalar multiplication (*=)", "[Quat][compound]") {
    Quat q(2.0f, 3.0f, 4.0f, 5.0f);
    q *= 2.0f;

    REQUIRE_THAT(q.w, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(10.0f, 1e-6f));
}

TEST_CASE("Quat: Compound scalar division (/=)", "[Quat][compound]") {
    Quat q(4.0f, 6.0f, 8.0f, 10.0f);
    q /= 2.0f;

    REQUIRE_THAT(q.w, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(5.0f, 1e-6f));
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Quat: Equality comparison", "[Quat][comparison]") {
    Quat a(1.0f, 2.0f, 3.0f, 4.0f);
    Quat b(1.0f, 2.0f, 3.0f, 4.0f);
    Quat c(5.0f, 6.0f, 7.0f, 8.0f);

    REQUIRE(a == b);
    REQUIRE_FALSE(a == c);
}

TEST_CASE("Quat: Inequality comparison", "[Quat][comparison]") {
    Quat a(1.0f, 2.0f, 3.0f, 4.0f);
    Quat b(5.0f, 6.0f, 7.0f, 8.0f);
    Quat c(1.0f, 2.0f, 3.0f, 4.0f);

    REQUIRE(a != b);
    REQUIRE_FALSE(a != c);
}

// ============================================================================
// Magnitude and Normalization
// ============================================================================

TEST_CASE("Quat: Magnitude squared", "[Quat][magnitude]") {
    Quat q(1.0f, 2.0f, 2.0f, 0.0f);  // 1 + 4 + 4 + 0 = 9
    REQUIRE_THAT(q.magnitudeSquared(), WithinAbs(9.0f, 1e-6f));
}

TEST_CASE("Quat: Magnitude", "[Quat][magnitude]") {
    Quat q(1.0f, 2.0f, 2.0f, 0.0f);  // √(1 + 4 + 4 + 0) = 3
    REQUIRE_THAT(q.magnitude(), WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("Quat: Norm (alias for magnitude)", "[Quat][magnitude]") {
    Quat q(1.0f, 2.0f, 2.0f, 0.0f);
    REQUIRE_THAT(q.norm(), WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("Quat: Normalized", "[Quat][magnitude]") {
    Quat q(1.0f, 2.0f, 2.0f, 0.0f);
    Quat normalized = q.normalized();

    float mag = normalized.magnitude();
    REQUIRE_THAT(mag, WithinAbs(1.0f, 1e-6f));

    // Check components are scaled correctly
    REQUIRE_THAT(normalized.w, WithinAbs(1.0f / 3.0f, 1e-6f));
    REQUIRE_THAT(normalized.x, WithinAbs(2.0f / 3.0f, 1e-6f));
    REQUIRE_THAT(normalized.y, WithinAbs(2.0f / 3.0f, 1e-6f));
    REQUIRE_THAT(normalized.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Quat: Normalize in-place", "[Quat][magnitude]") {
    Quat q(1.0f, 2.0f, 2.0f, 0.0f);
    q.normalize();

    float mag = q.magnitude();
    REQUIRE_THAT(mag, WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Quat: Near-zero magnitude handling", "[Quat][magnitude]") {
    Quat q(1e-7f, 1e-7f, 1e-7f, 1e-7f);
    Quat normalized = q.normalized();

    // Should return identity for degenerate quaternion
    REQUIRE_THAT(normalized.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(normalized.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(normalized.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(normalized.z, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Conjugate and Inverse
// ============================================================================

TEST_CASE("Quat: Conjugate", "[Quat][conjugate]") {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    Quat conj = q.conjugate();

    REQUIRE_THAT(conj.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(conj.x, WithinAbs(-2.0f, 1e-6f));
    REQUIRE_THAT(conj.y, WithinAbs(-3.0f, 1e-6f));
    REQUIRE_THAT(conj.z, WithinAbs(-4.0f, 1e-6f));
}

TEST_CASE("Quat: Conjugate of conjugate is original", "[Quat][conjugate]") {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    Quat original = q.conjugate().conjugate();

    REQUIRE_THAT(original.w, WithinAbs(q.w, 1e-6f));
    REQUIRE_THAT(original.x, WithinAbs(q.x, 1e-6f));
    REQUIRE_THAT(original.y, WithinAbs(q.y, 1e-6f));
    REQUIRE_THAT(original.z, WithinAbs(q.z, 1e-6f));
}

TEST_CASE("Quat: Inverse", "[Quat][inverse]") {
    Quat q = Quat(Vec3(0.0f, 0.0f, 1.0f), static_cast<float>(mathf::pi / 2.0)).normalized();
    Quat inv = q.inverse();

    // q * q⁻¹ should be identity
    Quat product = q * inv;

    REQUIRE_THAT(product.w, WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(product.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(product.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(product.z, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Quat: Inverse of unit quaternion is conjugate", "[Quat][inverse]") {
    // Create a unit quaternion
    Quat q = Quat(Vec3(1.0f, 0.0f, 0.0f), static_cast<float>(mathf::pi / 4.0f)).normalized();
    Quat inv = q.inverse();
    Quat conj = q.conjugate();

    // For unit quaternions: q⁻¹ = q*
    REQUIRE_THAT(inv.w, WithinAbs(conj.w, 1e-6f));
    REQUIRE_THAT(inv.x, WithinAbs(conj.x, 1e-6f));
    REQUIRE_THAT(inv.y, WithinAbs(conj.y, 1e-6f));
    REQUIRE_THAT(inv.z, WithinAbs(conj.z, 1e-6f));
}

// ============================================================================
// Dot Product
// ============================================================================

TEST_CASE("Quat: Dot product", "[Quat][dot]") {
    Quat q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quat q2(2.0f, 3.0f, 4.0f, 5.0f);

    // 1*2 + 2*3 + 3*4 + 4*5 = 2 + 6 + 12 + 20 = 40
    float result = dot(q1, q2);
    REQUIRE_THAT(result, WithinAbs(40.0f, 1e-6f));
}

TEST_CASE("Quat: Dot product with self equals magnitude squared", "[Quat][dot]") {
    Quat q(1.0f, 2.0f, 2.0f, 0.0f);
    float dotProduct = dot(q, q);
    float magSq = q.magnitudeSquared();

    REQUIRE_THAT(dotProduct, WithinAbs(magSq, 1e-6f));
}

// ============================================================================
// Output Stream
// ============================================================================

TEST_CASE("Quat: Stream output", "[Quat][output]") {
    Quat q(1.0f, 2.0f, 3.0f, 4.0f);
    std::ostringstream oss;
    oss << q;
    std::string output = oss.str();

    // Check that output contains quaternion components
    REQUIRE(output.find("1") != std::string::npos);
    REQUIRE(output.find("2") != std::string::npos);
    REQUIRE(output.find("3") != std::string::npos);
    REQUIRE(output.find("4") != std::string::npos);
}

// ============================================================================
// Vector Rotation
// ============================================================================

TEST_CASE("Quat: Rotate vector by identity", "[Quat][rotation]") {
    Quat identity;
    Vec3 v(1.0f, 2.0f, 3.0f);
    Vec3 rotated = identity.rotateVector(v);

    REQUIRE_THAT(rotated.x, WithinAbs(v.x, 1e-5f));
    REQUIRE_THAT(rotated.y, WithinAbs(v.y, 1e-5f));
    REQUIRE_THAT(rotated.z, WithinAbs(v.z, 1e-5f));
}

TEST_CASE("Quat: 90° rotation around X-axis", "[Quat][rotation]") {
    // 90° rotation around X-axis: (1, 0, 0)
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q = Quat(axis, static_cast<float>(mathf::pi / 2.0)).normalized();

    // Rotate unit vector along Y-axis (0, 1, 0)
    // Should result in (0, 0, 1)
    Vec3 v(0.0f, 1.0f, 0.0f);
    Vec3 rotated = q.rotateVector(v);

    REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated.z, WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Quat: 90° rotation around Y-axis", "[Quat][rotation]") {
    // 90° rotation around Y-axis: (0, 1, 0)
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q = Quat(axis, static_cast<float>(mathf::pi / 2.0)).normalized();

    // Rotate unit vector along Z-axis (0, 0, 1)
    // Should result in (1, 0, 0)
    Vec3 v(0.0f, 0.0f, 1.0f);
    Vec3 rotated = q.rotateVector(v);

    REQUIRE_THAT(rotated.x, WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Quat: 90° rotation around Z-axis", "[Quat][rotation]") {
    // 90° rotation around Z-axis: (0, 0, 1)
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q = Quat(axis, static_cast<float>(mathf::pi / 2.0)).normalized();

    // Rotate unit vector along X-axis (1, 0, 0)
    // Should result in (0, 1, 0)
    Vec3 v(1.0f, 0.0f, 0.0f);
    Vec3 rotated = q.rotateVector(v);

    REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated.y, WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Quat: 180° rotation around axis", "[Quat][rotation]") {
    // 180° rotation around Z-axis: (0, 0, 1)
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q = Quat(axis, static_cast<float>(mathf::pi)).normalized();

    Vec3 v(1.0f, 0.0f, 0.0f);
    Vec3 rotated = q.rotateVector(v);

    // 180° rotation around Z-axis should flip X and Y components
    // For v=(1,0,0), result should be approximately (-1, 0, 0)
    REQUIRE_THAT(rotated.x, WithinAbs(-1.0f, 1e-4f));
    REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("Quat: Rotate then unrotate returns original", "[Quat][rotation]") {
    Vec3 axis(1.0f, 2.0f, 3.0f);
    axis = axis.normalized();
    Quat q = Quat(axis, static_cast<float>(mathf::pi / 3.0)).normalized();

    Vec3 original(4.5f, -2.3f, 7.1f);
    Vec3 rotated = q.rotateVector(original);
    Vec3 unrotated = q.unrotateVector(rotated);

    REQUIRE_THAT(unrotated.x, WithinAbs(original.x, 1e-4f));
    REQUIRE_THAT(unrotated.y, WithinAbs(original.y, 1e-4f));
    REQUIRE_THAT(unrotated.z, WithinAbs(original.z, 1e-4f));
}

TEST_CASE("Quat: Unrotate reverses rotate", "[Quat][rotation]") {
    Vec3 axis(0.707f, 0.707f, 0.0f);
    Quat q = Quat(axis, static_cast<float>(mathf::pi / 4.0)).normalized();

    Vec3 v(1.0f, 2.0f, 3.0f);
    Vec3 backward = q.unrotateVector(v);

    // unrotateVector should be equivalent to rotateVector with q inverse
    Quat qInv = q.inverse();
    Vec3 expected = qInv.rotateVector(v);

    REQUIRE_THAT(backward.x, WithinAbs(expected.x, 1e-5f));
    REQUIRE_THAT(backward.y, WithinAbs(expected.y, 1e-5f));
    REQUIRE_THAT(backward.z, WithinAbs(expected.z, 1e-5f));
}

TEST_CASE("Quat: Vector magnitude preserved under rotation", "[Quat][rotation]") {
    Vec3 axis(1.0f, 0.0f, 1.0f);
    axis = axis.normalized();
    Quat q = Quat(axis, static_cast<float>(mathf::pi / 6.0)).normalized();

    Vec3 v(3.0f, 4.0f, 5.0f);
    float originalMag = v.length();

    Vec3 rotated = q.rotateVector(v);
    float rotatedMag = rotated.length();

    REQUIRE_THAT(rotatedMag, WithinAbs(originalMag, 1e-4f));
}

TEST_CASE("Quat: Composition of rotations via quaternion multiplication", "[Quat][rotation]") {
    // Create two rotations
    Vec3 axis1(1.0f, 0.0f, 0.0f);
    Quat q1 = Quat(axis1, static_cast<float>(mathf::pi / 4.0)).normalized();

    Vec3 axis2(0.0f, 1.0f, 0.0f);
    Quat q2 = Quat(axis2, static_cast<float>(mathf::pi / 4.0)).normalized();

    // Compose rotations
    Quat qComposed = q2 * q1;

    Vec3 v(1.0f, 0.0f, 0.0f);
    
    // Apply composed rotation
    Vec3 composedResult = qComposed.rotateVector(v);

    // Apply rotations sequentially
    Vec3 sequentialResult = q1.rotateVector(v);
    sequentialResult = q2.rotateVector(sequentialResult);

    REQUIRE_THAT(composedResult.x, WithinAbs(sequentialResult.x, 1e-4f));
    REQUIRE_THAT(composedResult.y, WithinAbs(sequentialResult.y, 1e-4f));
    REQUIRE_THAT(composedResult.z, WithinAbs(sequentialResult.z, 1e-4f));
}

TEST_CASE("Quat: Full rotation (360°) returns to original", "[Quat][rotation]") {
    Vec3 axis(1.0f, 2.0f, 3.0f);
    axis = axis.normalized();
    Quat q = Quat(axis, static_cast<float>(2.0 * mathf::pi)).normalized();

    Vec3 v(5.0f, -3.0f, 2.0f);
    Vec3 rotated = q.rotateVector(v);

    // 360° rotation should return to original (approximately)
    REQUIRE_THAT(rotated.x, WithinAbs(v.x, 1e-4f));
    REQUIRE_THAT(rotated.y, WithinAbs(v.y, 1e-4f));
    REQUIRE_THAT(rotated.z, WithinAbs(v.z, 1e-4f));
}

TEST_CASE("Quat: Rotating orthogonal vectors", "[Quat][rotation]") {
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q = Quat(axis, static_cast<float>(mathf::pi / 2.0)).normalized();

    Vec3 v1(1.0f, 0.0f, 0.0f);
    Vec3 v2(0.0f, 0.0f, 1.0f);

    Vec3 r1 = q.rotateVector(v1);
    Vec3 r2 = q.rotateVector(v2);

    // Rotated vectors should still be orthogonal
    float dotProduct = r1.x * r2.x + r1.y * r2.y + r1.z * r2.z;
    REQUIRE_THAT(dotProduct, WithinAbs(0.0f, 1e-5f));
}

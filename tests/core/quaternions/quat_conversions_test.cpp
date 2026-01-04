#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_conversions.hpp>
#include <core/math/matrices/mat3.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace phynity::math::quaternions;
using namespace phynity::math::matrices;
using namespace phynity::math::vectors;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Helper Functions
// ============================================================================

/// Compare two quaternions accounting for double-cover (q and -q are equivalent)
bool quaternionsEqual(const Quat& q1, const Quat& q2, float tolerance = 1e-5f) {
    // Check if they're the same
    bool same = (std::abs(q1.w - q2.w) < tolerance) &&
                (std::abs(q1.x - q2.x) < tolerance) &&
                (std::abs(q1.y - q2.y) < tolerance) &&
                (std::abs(q1.z - q2.z) < tolerance);
    
    // Check if they're negatives (double-cover)
    bool opposite = (std::abs(q1.w + q2.w) < tolerance) &&
                    (std::abs(q1.x + q2.x) < tolerance) &&
                    (std::abs(q1.y + q2.y) < tolerance) &&
                    (std::abs(q1.z + q2.z) < tolerance);
    
    return same || opposite;
}

/// Verify that a matrix is a valid rotation matrix
bool isValidRotationMatrix(const Mat3& m, float tolerance = 1e-4f) {
    // Check orthonormality: rows should be orthogonal
    Vec3 row0 = m.getRow(0);
    Vec3 row1 = m.getRow(1);
    Vec3 row2 = m.getRow(2);
    
    if (std::abs(row0.dot(row1)) > tolerance) return false;
    if (std::abs(row1.dot(row2)) > tolerance) return false;
    if (std::abs(row2.dot(row0)) > tolerance) return false;
    
    // Check that rows are unit length
    if (std::abs(row0.length() - 1.0f) > tolerance) return false;
    if (std::abs(row1.length() - 1.0f) > tolerance) return false;
    if (std::abs(row2.length() - 1.0f) > tolerance) return false;
    
    // Check determinant is +1 (not -1, which would be a reflection)
    float det = m.determinant();
    if (std::abs(det - 1.0f) > tolerance) return false;
    
    return true;
}

/// Compare two matrices element-wise
bool matricesEqual(const Mat3& m1, const Mat3& m2, float tolerance = 1e-5f) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (std::abs(m1.m[i][j] - m2.m[i][j]) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

// ============================================================================
// Quaternion to Rotation Matrix Tests
// ============================================================================

TEST_CASE("Conversion: Identity quaternion to identity matrix", "[conversion][quat-to-matrix]") {
    Quat q;  // Identity: (1, 0, 0, 0)
    Mat3 m = toRotationMatrix(q);
    
    // Should produce identity matrix
    REQUIRE_THAT(m.m[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][1], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(m.m[0][2], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][0], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][1], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(m.m[1][2], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(m.m[2][0], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(m.m[2][1], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(m.m[2][2], WithinAbs(1.0f, 1e-6f));
    
    REQUIRE(isValidRotationMatrix(m));
}

TEST_CASE("Conversion: 90° rotation around X-axis (quat to matrix)", "[conversion][quat-to-matrix]") {
    // 90° around X: q = (cos(45°), sin(45°), 0, 0)
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q(axis, static_cast<float>(M_PI / 2.0));
    
    Mat3 m = toRotationMatrix(q);
    
    // Test by rotating a vector (0, 1, 0) should give (0, 0, 1)
    Vec3 v(0.0f, 1.0f, 0.0f);
    Vec3 rotated = m * v;
    
    REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated.z, WithinAbs(1.0f, 1e-5f));
    
    REQUIRE(isValidRotationMatrix(m));
}

TEST_CASE("Conversion: 90° rotation around Y-axis (quat to matrix)", "[conversion][quat-to-matrix]") {
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q(axis, static_cast<float>(M_PI / 2.0));
    
    Mat3 m = toRotationMatrix(q);
    
    // Test by rotating a vector (0, 0, 1) should give (1, 0, 0)
    Vec3 v(0.0f, 0.0f, 1.0f);
    Vec3 rotated = m * v;
    
    REQUIRE_THAT(rotated.x, WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(rotated.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-5f));
    
    REQUIRE(isValidRotationMatrix(m));
}

TEST_CASE("Conversion: 90° rotation around Z-axis (quat to matrix)", "[conversion][quat-to-matrix]") {
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q(axis, static_cast<float>(M_PI / 2.0));
    
    Mat3 m = toRotationMatrix(q);
    
    // Test by rotating a vector (1, 0, 0) should give (0, 1, 0)
    Vec3 v(1.0f, 0.0f, 0.0f);
    Vec3 rotated = m * v;
    
    REQUIRE_THAT(rotated.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(rotated.y, WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-5f));
    
    REQUIRE(isValidRotationMatrix(m));
}

TEST_CASE("Conversion: 180° rotation around X-axis (quat to matrix)", "[conversion][quat-to-matrix]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q(axis, static_cast<float>(M_PI));
    
    Mat3 m = toRotationMatrix(q);
    
    // 180° around X: Y -> -Y, Z -> -Z, X unchanged
    Vec3 vY(0.0f, 1.0f, 0.0f);
    Vec3 rotatedY = m * vY;
    
    REQUIRE_THAT(rotatedY.x, WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(rotatedY.y, WithinAbs(-1.0f, 1e-4f));
    REQUIRE_THAT(rotatedY.z, WithinAbs(0.0f, 1e-4f));
    
    REQUIRE(isValidRotationMatrix(m));
}

TEST_CASE("Conversion: 180° rotation around Y-axis (quat to matrix)", "[conversion][quat-to-matrix]") {
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q(axis, static_cast<float>(M_PI));
    
    Mat3 m = toRotationMatrix(q);
    
    // 180° around Y: X -> -X, Z -> -Z, Y unchanged
    Vec3 vX(1.0f, 0.0f, 0.0f);
    Vec3 rotatedX = m * vX;
    
    REQUIRE_THAT(rotatedX.x, WithinAbs(-1.0f, 1e-4f));
    REQUIRE_THAT(rotatedX.y, WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(rotatedX.z, WithinAbs(0.0f, 1e-4f));
    
    REQUIRE(isValidRotationMatrix(m));
}

TEST_CASE("Conversion: 180° rotation around Z-axis (quat to matrix)", "[conversion][quat-to-matrix]") {
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q(axis, static_cast<float>(M_PI));
    
    Mat3 m = toRotationMatrix(q);
    
    // 180° around Z: X -> -X, Y -> -Y, Z unchanged
    Vec3 vX(1.0f, 0.0f, 0.0f);
    Vec3 rotatedX = m * vX;
    
    REQUIRE_THAT(rotatedX.x, WithinAbs(-1.0f, 1e-4f));
    REQUIRE_THAT(rotatedX.y, WithinAbs(0.0f, 1e-4f));
    REQUIRE_THAT(rotatedX.z, WithinAbs(0.0f, 1e-4f));
    
    REQUIRE(isValidRotationMatrix(m));
}

TEST_CASE("Conversion: Arbitrary rotation (quat to matrix)", "[conversion][quat-to-matrix]") {
    // 45° around axis (1, 1, 1) normalized
    Vec3 axis(1.0f, 1.0f, 1.0f);
    axis = axis.normalized();
    Quat q(axis, static_cast<float>(M_PI / 4.0));
    
    Mat3 m = toRotationMatrix(q);
    
    // Verify it's a valid rotation matrix
    REQUIRE(isValidRotationMatrix(m));
    
    // Verify rotation consistency: rotating a vector with quat and matrix should match
    Vec3 testVec(3.0f, -2.0f, 5.0f);
    Vec3 quatRotated = q.rotateVector(testVec);
    Vec3 matRotated = m * testVec;
    
    REQUIRE_THAT(matRotated.x, WithinAbs(quatRotated.x, 1e-4f));
    REQUIRE_THAT(matRotated.y, WithinAbs(quatRotated.y, 1e-4f));
    REQUIRE_THAT(matRotated.z, WithinAbs(quatRotated.z, 1e-4f));
}

TEST_CASE("Conversion: Non-unit quaternion handles normalization", "[conversion][quat-to-matrix]") {
    // Create a non-unit quaternion
    Quat q(2.0f, 1.0f, 0.0f, 0.0f);  // Magnitude != 1
    
    Mat3 m = toRotationMatrix(q);
    
    // Should still produce valid rotation matrix (function normalizes internally)
    REQUIRE(isValidRotationMatrix(m));
}

TEST_CASE("Conversion: Very small rotation (quat to matrix)", "[conversion][quat-to-matrix]") {
    // Very small angle (0.1 degrees)
    Vec3 axis(0.0f, 1.0f, 0.0f);
    float angle = 0.1f * static_cast<float>(M_PI) / 180.0f;
    Quat q(axis, angle);
    
    Mat3 m = toRotationMatrix(q);
    
    // Should still produce valid rotation matrix
    REQUIRE(isValidRotationMatrix(m));
    
    // Should be very close to identity
    Mat3 identity;
    REQUIRE(matricesEqual(m, identity, 1e-2f));
}

// ============================================================================
// Rotation Matrix to Quaternion Tests
// ============================================================================

TEST_CASE("Conversion: Identity matrix to identity quaternion", "[conversion][matrix-to-quat]") {
    Mat3 m;  // Identity matrix
    Quat q = toQuaternion(m);
    
    // Should produce identity quaternion (1, 0, 0, 0)
    REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Conversion: 90° rotation matrix around X-axis to quaternion", "[conversion][matrix-to-quat]") {
    // Construct 90° rotation matrix around X-axis manually
    // Rx(90°) = [1   0    0  ]
    //           [0   0   -1  ]
    //           [0   1    0  ]
    Mat3 m(1.0f,  0.0f,  0.0f,
           0.0f,  0.0f, -1.0f,
           0.0f,  1.0f,  0.0f);
    
    Quat q = toQuaternion(m);
    
    // Expected quaternion for 90° around X: (cos(45°), sin(45°), 0, 0)
    float expected_w = std::cos(static_cast<float>(M_PI / 4.0));
    float expected_x = std::sin(static_cast<float>(M_PI / 4.0));
    
    REQUIRE_THAT(std::abs(q.w), WithinAbs(expected_w, 1e-5f));
    REQUIRE_THAT(std::abs(q.x), WithinAbs(expected_x, 1e-5f));
    REQUIRE_THAT(std::abs(q.y), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(q.z), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Conversion: 90° rotation matrix around Y-axis to quaternion", "[conversion][matrix-to-quat]") {
    // Ry(90°) = [ 0   0   1  ]
    //           [ 0   1   0  ]
    //           [-1   0   0  ]
    Mat3 m(0.0f,  0.0f,  1.0f,
           0.0f,  1.0f,  0.0f,
          -1.0f,  0.0f,  0.0f);
    
    Quat q = toQuaternion(m);
    
    float expected_w = std::cos(static_cast<float>(M_PI / 4.0));
    float expected_y = std::sin(static_cast<float>(M_PI / 4.0));
    
    REQUIRE_THAT(std::abs(q.w), WithinAbs(expected_w, 1e-5f));
    REQUIRE_THAT(std::abs(q.x), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(q.y), WithinAbs(expected_y, 1e-5f));
    REQUIRE_THAT(std::abs(q.z), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Conversion: 90° rotation matrix around Z-axis to quaternion", "[conversion][matrix-to-quat]") {
    // Rz(90°) = [ 0  -1   0  ]
    //           [ 1   0   0  ]
    //           [ 0   0   1  ]
    Mat3 m(0.0f, -1.0f,  0.0f,
           1.0f,  0.0f,  0.0f,
           0.0f,  0.0f,  1.0f);
    
    Quat q = toQuaternion(m);
    
    float expected_w = std::cos(static_cast<float>(M_PI / 4.0));
    float expected_z = std::sin(static_cast<float>(M_PI / 4.0));
    
    REQUIRE_THAT(std::abs(q.w), WithinAbs(expected_w, 1e-5f));
    REQUIRE_THAT(std::abs(q.x), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(q.y), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(q.z), WithinAbs(expected_z, 1e-5f));
}

TEST_CASE("Conversion: 180° rotation matrix around X-axis to quaternion", "[conversion][matrix-to-quat]") {
    // Rx(180°) = [1   0    0  ]
    //            [0  -1    0  ]
    //            [0   0   -1  ]
    Mat3 m(1.0f,  0.0f,  0.0f,
           0.0f, -1.0f,  0.0f,
           0.0f,  0.0f, -1.0f);
    
    Quat q = toQuaternion(m);
    
    // For 180° rotation: w ≈ 0, one component ≈ 1
    REQUIRE_THAT(std::abs(q.w), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(q.x), WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(std::abs(q.y), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(q.z), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Conversion: Invalid matrix with NaN returns identity", "[conversion][matrix-to-quat][validation]") {
    Mat3 m(std::nan(""), 0.0f, 0.0f,
           0.0f, 1.0f, 0.0f,
           0.0f, 0.0f, 1.0f);
    
    Quat q = toQuaternion(m);
    
    // Should return identity quaternion as fallback
    REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Conversion: Invalid matrix with Inf returns identity", "[conversion][matrix-to-quat][validation]") {
    Mat3 m(std::numeric_limits<float>::infinity(), 0.0f, 0.0f,
           0.0f, 1.0f, 0.0f,
           0.0f, 0.0f, 1.0f);
    
    Quat q = toQuaternion(m);
    
    // Should return identity quaternion as fallback
    REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Conversion: Reflection matrix (negative determinant) returns identity", "[conversion][matrix-to-quat][validation]") {
    // Reflection matrix (determinant = -1)
    Mat3 m(-1.0f,  0.0f,  0.0f,
            0.0f,  1.0f,  0.0f,
            0.0f,  0.0f,  1.0f);
    
    Quat q = toQuaternion(m);
    
    // Should return identity quaternion as fallback
    REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Conversion: Non-orthonormal matrix returns identity", "[conversion][matrix-to-quat][validation]") {
    // Scaled rotation matrix (not orthonormal)
    Mat3 m(2.0f, 0.0f, 0.0f,
           0.0f, 2.0f, 0.0f,
           0.0f, 0.0f, 2.0f);
    
    Quat q = toQuaternion(m);
    
    // Should return identity quaternion as fallback
    REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Round-Trip Conversion Tests
// ============================================================================

TEST_CASE("Round-trip: Quat -> Matrix -> Quat preserves rotation", "[conversion][round-trip]") {
    SECTION("Identity") {
        Quat original;
        Mat3 m = toRotationMatrix(original);
        Quat result = toQuaternion(m);
        
        REQUIRE(quaternionsEqual(original, result, 1e-5f));
    }
    
    SECTION("90° around X") {
        Quat original(Vec3(1.0f, 0.0f, 0.0f), static_cast<float>(M_PI / 2.0));
        Mat3 m = toRotationMatrix(original);
        Quat result = toQuaternion(m);
        
        REQUIRE(quaternionsEqual(original, result, 1e-5f));
    }
    
    SECTION("45° around arbitrary axis") {
        Vec3 axis(1.0f, 2.0f, 3.0f);
        axis = axis.normalized();
        Quat original(axis, static_cast<float>(M_PI / 4.0));
        
        Mat3 m = toRotationMatrix(original);
        Quat result = toQuaternion(m);
        
        REQUIRE(quaternionsEqual(original, result, 1e-5f));
    }
    
    SECTION("Large angle rotation") {
        Quat original(Vec3(0.0f, 1.0f, 0.0f), static_cast<float>(2.8));
        Mat3 m = toRotationMatrix(original);
        Quat result = toQuaternion(m);
        
        REQUIRE(quaternionsEqual(original, result, 1e-5f));
    }
}

TEST_CASE("Round-trip: Matrix -> Quat -> Matrix preserves rotation", "[conversion][round-trip]") {
    SECTION("90° around Y") {
        Mat3 original(0.0f,  0.0f,  1.0f,
                      0.0f,  1.0f,  0.0f,
                     -1.0f,  0.0f,  0.0f);
        
        Quat q = toQuaternion(original);
        Mat3 result = toRotationMatrix(q);
        
        REQUIRE(matricesEqual(original, result, 1e-5f));
    }
    
    SECTION("180° around Z") {
        Mat3 original(-1.0f,  0.0f,  0.0f,
                       0.0f, -1.0f,  0.0f,
                       0.0f,  0.0f,  1.0f);
        
        Quat q = toQuaternion(original);
        Mat3 result = toRotationMatrix(q);
        
        REQUIRE(matricesEqual(original, result, 1e-4f));
    }
}

TEST_CASE("Round-trip: Vector rotation consistency", "[conversion][round-trip]") {
    // Create arbitrary rotation
    Vec3 axis(1.0f, -2.0f, 3.0f);
    axis = axis.normalized();
    float angle = 1.234f;
    Quat q(axis, angle);
    
    // Convert to matrix
    Mat3 m = toRotationMatrix(q);
    
    // Rotate several test vectors
    Vec3 testVectors[] = {
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f),
        Vec3(0.0f, 0.0f, 1.0f),
        Vec3(1.0f, 1.0f, 1.0f).normalized(),
        Vec3(2.5f, -3.7f, 1.2f)
    };
    
    for (const auto& v : testVectors) {
        Vec3 quatRotated = q.rotateVector(v);
        Vec3 matRotated = m * v;
        
        REQUIRE_THAT(matRotated.x, WithinAbs(quatRotated.x, 1e-5f));
        REQUIRE_THAT(matRotated.y, WithinAbs(quatRotated.y, 1e-5f));
        REQUIRE_THAT(matRotated.z, WithinAbs(quatRotated.z, 1e-5f));
    }
}

// ============================================================================
// Numerical Stress Tests
// ============================================================================

TEST_CASE("Stress test: Multiple conversions don't accumulate error", "[conversion][stress]") {
    Quat q(Vec3(1.0f, 1.0f, 1.0f).normalized(), static_cast<float>(M_PI / 6.0));
    
    // Convert back and forth multiple times
    for (int i = 0; i < 10; ++i) {
        Mat3 m = toRotationMatrix(q);
        q = toQuaternion(m);
    }
    
    // Quaternion should still be unit
    float magnitude = q.magnitude();
    REQUIRE_THAT(magnitude, WithinAbs(1.0f, 1e-5f));
    
    // Should still produce valid rotation matrix
    Mat3 finalMatrix = toRotationMatrix(q);
    REQUIRE(isValidRotationMatrix(finalMatrix));
}

TEST_CASE("Stress test: Very small angles", "[conversion][stress]") {
    // 0.01 degrees
    float angle = 0.01f * static_cast<float>(M_PI) / 180.0f;
    Quat q(Vec3(1.0f, 0.0f, 0.0f), angle);
    
    Mat3 m = toRotationMatrix(q);
    REQUIRE(isValidRotationMatrix(m));
    
    Quat q2 = toQuaternion(m);
    REQUIRE(quaternionsEqual(q, q2, 1e-5f));
}

TEST_CASE("Stress test: Near-180° rotations", "[conversion][stress]") {
    // 179.9 degrees
    float angle = 179.9f * static_cast<float>(M_PI) / 180.0f;
    
    SECTION("Around X") {
        Quat q(Vec3(1.0f, 0.0f, 0.0f), angle);
        Mat3 m = toRotationMatrix(q);
        REQUIRE(isValidRotationMatrix(m));
        
        Quat q2 = toQuaternion(m);
        REQUIRE(quaternionsEqual(q, q2, 1e-4f));
    }
    
    SECTION("Around Y") {
        Quat q(Vec3(0.0f, 1.0f, 0.0f), angle);
        Mat3 m = toRotationMatrix(q);
        REQUIRE(isValidRotationMatrix(m));
        
        Quat q2 = toQuaternion(m);
        REQUIRE(quaternionsEqual(q, q2, 1e-4f));
    }
    
    SECTION("Around Z") {
        Quat q(Vec3(0.0f, 0.0f, 1.0f), angle);
        Mat3 m = toRotationMatrix(q);
        REQUIRE(isValidRotationMatrix(m));
        
        Quat q2 = toQuaternion(m);
        REQUIRE(quaternionsEqual(q, q2, 1e-4f));
    }
}

TEST_CASE("Stress test: All Shepperd's method branches", "[conversion][stress]") {
    // This test ensures all 4 branches of Shepperd's method are exercised
    
    SECTION("Trace > 0 (common case - w is largest)") {
        // Small rotation - w will be largest
        Quat q(Vec3(1.0f, 0.0f, 0.0f), 0.5f);
        Mat3 m = toRotationMatrix(q);
        Quat result = toQuaternion(m);
        REQUIRE(quaternionsEqual(q, result, 1e-5f));
    }
    
    SECTION("X is largest component") {
        // ~180° around X
        Quat q(Vec3(1.0f, 0.0f, 0.0f), 3.0f);
        Mat3 m = toRotationMatrix(q);
        Quat result = toQuaternion(m);
        REQUIRE(quaternionsEqual(q, result, 1e-5f));
    }
    
    SECTION("Y is largest component") {
        // ~180° around Y
        Quat q(Vec3(0.0f, 1.0f, 0.0f), 3.0f);
        Mat3 m = toRotationMatrix(q);
        Quat result = toQuaternion(m);
        REQUIRE(quaternionsEqual(q, result, 1e-5f));
    }
    
    SECTION("Z is largest component") {
        // ~180° around Z
        Quat q(Vec3(0.0f, 0.0f, 1.0f), 3.0f);
        Mat3 m = toRotationMatrix(q);
        Quat result = toQuaternion(m);
        REQUIRE(quaternionsEqual(q, result, 1e-5f));
    }
}

// ============================================================================
// EULER ANGLES <-> QUATERNION CONVERSION TESTS
// ============================================================================

/// Helper to normalize angle to [-π, π] range
float normalizeAngle(float angle) {
    while (angle > mathf::pi) angle -= mathf::two_pi;
    while (angle < -mathf::pi) angle += mathf::two_pi;
    return angle;
}

/// Compare two Vec3 Euler angles considering angle wrapping
bool approxEuler(const Vec3& a, const Vec3& b, float tolerance = 1e-5f) {
    return std::abs(normalizeAngle(a.x - b.x)) < tolerance &&
           std::abs(normalizeAngle(a.y - b.y)) < tolerance &&
           std::abs(normalizeAngle(a.z - b.z)) < tolerance;
}

TEST_CASE("Conversion: Identity Euler to quaternion", "[conversion][euler-to-quat]") {
    Vec3 euler(0.0f, 0.0f, 0.0f);
    Quat q = toQuaternion(euler);
    
    // Identity should give (1, 0, 0, 0)
    REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Conversion: 90° roll Euler to quaternion", "[conversion][euler-to-quat]") {
    Vec3 euler(mathf::pi / 2.0f, 0.0f, 0.0f);
    Quat q = toQuaternion(euler);
    
    // 90° around X-axis: q = (cos(45°), sin(45°), 0, 0)
    float expected = std::sqrt(2.0f) / 2.0f;
    REQUIRE_THAT(q.w, WithinAbs(expected, 1e-5f));
    REQUIRE_THAT(q.x, WithinAbs(expected, 1e-5f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Conversion: 90° pitch Euler to quaternion", "[conversion][euler-to-quat]") {
    Vec3 euler(0.0f, mathf::pi / 2.0f, 0.0f);
    Quat q = toQuaternion(euler);
    
    // 90° around Y-axis: q = (cos(45°), 0, sin(45°), 0)
    float expected = std::sqrt(2.0f) / 2.0f;
    REQUIRE_THAT(q.w, WithinAbs(expected, 1e-5f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(q.y, WithinAbs(expected, 1e-5f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Conversion: 90° yaw Euler to quaternion", "[conversion][euler-to-quat]") {
    Vec3 euler(0.0f, 0.0f, mathf::pi / 2.0f);
    Quat q = toQuaternion(euler);
    
    // 90° around Z-axis: q = (cos(45°), 0, 0, sin(45°))
    float expected = std::sqrt(2.0f) / 2.0f;
    REQUIRE_THAT(q.w, WithinAbs(expected, 1e-5f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(q.z, WithinAbs(expected, 1e-5f));
}

TEST_CASE("Conversion: Combined Euler angles to quaternion", "[conversion][euler-to-quat]") {
    Vec3 euler(
        30.0f * mathf::deg_to_rad,
        45.0f * mathf::deg_to_rad,
        60.0f * mathf::deg_to_rad
    );
    Quat q = toQuaternion(euler);
    
    // Should be unit quaternion
    float magnitude = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    REQUIRE_THAT(magnitude, WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Round-trip: Euler -> Quat -> Euler (identity)", "[conversion][euler-round-trip]") {
    Vec3 original(0.0f, 0.0f, 0.0f);
    Quat q = toQuaternion(original);
    Vec3 recovered = toEulerAngles(q);
    
    REQUIRE(approxEuler(original, recovered));
}

TEST_CASE("Round-trip: Euler -> Quat -> Euler (single axis rotations)", "[conversion][euler-round-trip]") {
    std::vector<Vec3> testAngles = {
        Vec3(mathf::pi / 4.0f, 0.0f, 0.0f),   // 45° roll
        Vec3(0.0f, mathf::pi / 6.0f, 0.0f),   // 30° pitch
        Vec3(0.0f, 0.0f, mathf::pi / 3.0f),   // 60° yaw
        Vec3(mathf::pi / 2.0f, 0.0f, 0.0f),   // 90° roll
        Vec3(0.0f, 0.0f, mathf::pi / 2.0f),   // 90° yaw
    };
    
    for (const auto& original : testAngles) {
        Quat q = toQuaternion(original);
        Vec3 recovered = toEulerAngles(q);
        REQUIRE(approxEuler(original, recovered));
    }
}

TEST_CASE("Round-trip: Euler -> Quat -> Euler (combined rotations)", "[conversion][euler-round-trip]") {
    std::vector<Vec3> testAngles = {
        Vec3(30.0f * mathf::deg_to_rad, 45.0f * mathf::deg_to_rad, 60.0f * mathf::deg_to_rad),
        Vec3(-30.0f * mathf::deg_to_rad, 20.0f * mathf::deg_to_rad, 15.0f * mathf::deg_to_rad),
        Vec3(15.0f * mathf::deg_to_rad, -45.0f * mathf::deg_to_rad, 120.0f * mathf::deg_to_rad),
        Vec3(80.0f * mathf::deg_to_rad, 70.0f * mathf::deg_to_rad, 50.0f * mathf::deg_to_rad),
        Vec3(-45.0f * mathf::deg_to_rad, -30.0f * mathf::deg_to_rad, -60.0f * mathf::deg_to_rad),
    };
    
    for (const auto& original : testAngles) {
        Quat q = toQuaternion(original);
        Vec3 recovered = toEulerAngles(q);
        REQUIRE(approxEuler(original, recovered));
    }
}

TEST_CASE("Round-trip: Euler -> Quat -> Euler (gimbal lock at +90°)", "[conversion][euler-round-trip][gimbal-lock]") {
    // At gimbal lock, roll and yaw combine - roll becomes 0, yaw captures combined rotation
    Vec3 original(0.0f, mathf::pi / 2.0f, 0.0f);
    Quat q = toQuaternion(original);
    Vec3 recovered = toEulerAngles(q);
    
    // Pitch should be preserved
    REQUIRE_THAT(recovered.y, WithinAbs(mathf::pi / 2.0f, 1e-5f));
    // Roll should be 0 by convention
    REQUIRE_THAT(recovered.x, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Round-trip: Euler -> Quat -> Euler (gimbal lock at -90°)", "[conversion][euler-round-trip][gimbal-lock]") {
    Vec3 original(0.0f, -mathf::pi / 2.0f, 0.0f);
    Quat q = toQuaternion(original);
    Vec3 recovered = toEulerAngles(q);
    
    REQUIRE_THAT(recovered.y, WithinAbs(-mathf::pi / 2.0f, 1e-5f));
    REQUIRE_THAT(recovered.x, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Round-trip: Euler -> Quat -> Euler (gimbal lock with non-zero roll+yaw)", "[conversion][euler-round-trip][gimbal-lock]") {
    // At gimbal lock with non-zero roll and yaw:
    // The Euler representation is ambiguous - roll and yaw combine into a single degree of freedom.
    // We cannot expect to recover the exact same Euler angles, but the ROTATION must be preserved.
    Vec3 original(
        30.0f * mathf::deg_to_rad,      // roll = 30°
        mathf::pi / 2.0f,                // pitch = 90° (gimbal lock)
        45.0f * mathf::deg_to_rad        // yaw = 45°
    );
    
    Quat q = toQuaternion(original);
    Vec3 recovered = toEulerAngles(q);
    Quat q_back = toQuaternion(recovered);

    float rotation_error = std::abs(dot(q, q_back));

    REQUIRE(rotation_error > 1.0f - 1e-5f);
}

TEST_CASE("Round-trip: Quat -> Euler -> Quat (identity)", "[conversion][quat-round-trip]") {
    Quat original(1.0f, 0.0f, 0.0f, 0.0f);
    Vec3 euler = toEulerAngles(original);
    Quat recovered = toQuaternion(euler);
    
    REQUIRE(quaternionsEqual(original, recovered));
}

TEST_CASE("Round-trip: Quat -> Euler -> Quat (simple rotations)", "[conversion][quat-round-trip]") {
    std::vector<Quat> testQuats = {
        Quat(Vec3(1.0f, 0.0f, 0.0f), mathf::pi / 4.0f),  // 45° around X
        Quat(Vec3(0.0f, 1.0f, 0.0f), mathf::pi / 6.0f),  // 30° around Y
        Quat(Vec3(0.0f, 0.0f, 1.0f), mathf::pi / 3.0f),  // 60° around Z
    };
    
    for (const auto& original : testQuats) {
        Vec3 euler = toEulerAngles(original);
        Quat recovered = toQuaternion(euler);
        REQUIRE(quaternionsEqual(original, recovered));
    }
}

TEST_CASE("Round-trip: Quat -> Euler -> Quat (simple normalized quaternions)", "[conversion][quat-round-trip]") {
    // Use simple, well-formed quaternions to avoid gimbal lock issues
    std::vector<Quat> testQuats = {
        Quat(0.9238795f, 0.3826834f, 0.0f, 0.0f),           // ~45° around X
        Quat(0.9238795f, 0.0f, 0.3826834f, 0.0f),           // ~45° around Y
        Quat(0.9238795f, 0.0f, 0.0f, 0.3826834f),           // ~45° around Z
    };
    
    for (auto original : testQuats) {
        Vec3 euler = toEulerAngles(original);
        Quat recovered = toQuaternion(euler);
        
        // Use tolerance to account for numerical precision in round-trip
        REQUIRE(quaternionsEqual(original, recovered, 1e-3f));
    }
}

TEST_CASE("Round-trip: Quat -> Euler -> Quat (near gimbal lock)", "[conversion][quat-round-trip]") {
    // Test quaternions that produce pitch close to ±90°
    Vec3 euler1(10.0f * mathf::deg_to_rad, 89.0f * mathf::deg_to_rad, 20.0f * mathf::deg_to_rad);
    Quat q1 = toQuaternion(euler1);
    Vec3 recovered1 = toEulerAngles(q1);
    Quat q1_back = toQuaternion(recovered1);
    REQUIRE(quaternionsEqual(q1, q1_back, 1e-4f));
    
    Vec3 euler2(10.0f * mathf::deg_to_rad, -89.0f * mathf::deg_to_rad, 20.0f * mathf::deg_to_rad);
    Quat q2 = toQuaternion(euler2);
    Vec3 recovered2 = toEulerAngles(q2);
    Quat q2_back = toQuaternion(recovered2);
    REQUIRE(quaternionsEqual(q2, q2_back, 1e-4f));
}

TEST_CASE("Stress: Random Euler angles round-trip", "[conversion][euler-round-trip][stress]") {
    // Test with random Euler angles (avoiding exact gimbal lock)
    std::srand(12345); // Fixed seed for reproducibility
    
    for (int i = 0; i < 100; ++i) {
        Vec3 original(
            static_cast<float>(std::rand() % 360 - 180) * mathf::deg_to_rad,  // roll: [-180, 180]
            static_cast<float>(std::rand() % 178 - 89) * mathf::deg_to_rad,   // pitch: [-89, 89] (avoid exact gimbal lock)
            static_cast<float>(std::rand() % 360 - 180) * mathf::deg_to_rad   // yaw: [-180, 180]
        );
        
        Quat q = toQuaternion(original);
        Vec3 recovered = toEulerAngles(q);
        
        REQUIRE(approxEuler(original, recovered, 1e-4f));
    }
}

TEST_CASE("Stress: Euler conversion is deterministic", "[conversion][euler-round-trip][stress]") {
    Vec3 euler(30.0f * mathf::deg_to_rad, 45.0f * mathf::deg_to_rad, 60.0f * mathf::deg_to_rad);
    
    // Convert multiple times - should always get same result
    Quat q1 = toQuaternion(euler);
    Quat q2 = toQuaternion(euler);
    Quat q3 = toQuaternion(euler);
    
    REQUIRE(quaternionsEqual(q1, q2));
    REQUIRE(quaternionsEqual(q2, q3));
    
    Vec3 e1 = toEulerAngles(q1);
    Vec3 e2 = toEulerAngles(q1);
    Vec3 e3 = toEulerAngles(q1);
    
    REQUIRE(approxEuler(e1, e2));
    REQUIRE(approxEuler(e2, e3));
}

TEST_CASE("Edge case: Very small Euler angles", "[conversion][euler-round-trip][edge-case]") {
    Vec3 original(0.001f, 0.002f, 0.003f);
    Quat q = toQuaternion(original);
    Vec3 recovered = toEulerAngles(q);
    
    REQUIRE(approxEuler(original, recovered));
}

TEST_CASE("Edge case: Large Euler angles (angle wrapping)", "[conversion][euler-round-trip][edge-case]") {
    // Test angles outside [-π, π] range
    Vec3 original(
        3.0f * mathf::pi,        // 540° (wraps to 180°)
        mathf::pi / 4.0f,        // 45°
        -2.5f * mathf::pi        // -450° (wraps to -90°)
    );
    
    Quat q = toQuaternion(original);
    Vec3 recovered = toEulerAngles(q);
    
    // Recovered should be in normalized range but represent same rotation
    Quat q_recovered = toQuaternion(recovered);
    REQUIRE(quaternionsEqual(q, q_recovered));
}

TEST_CASE("Consistency: Euler -> Quat matches expected quaternion", "[conversion][euler-to-quat][validation]") {
    // Known test case from conversion_formulas.txt
    Vec3 euler(0.0f, 0.0f, 0.0f);
    Quat q = toQuaternion(euler);
    
    // Identity: q = (1, 0, 0, 0)
    REQUIRE_THAT(q.w, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q.z, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Quaternion to Axis-Angle Tests
// ============================================================================

TEST_CASE("Conversion: Identity quaternion to axis-angle", "[conversion][quat-to-axis-angle]") {
    Quat q;  // Identity: (1, 0, 0, 0)
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // Should produce zero angle
    REQUIRE_THAT(angle, WithinAbs(0.0f, 1e-6f));
    
    // Axis is arbitrary but should be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Conversion: 90° rotation around X-axis (quat to axis-angle)", "[conversion][quat-to-axis-angle]") {
    Vec3 expectedAxis(1.0f, 0.0f, 0.0f);
    float expectedAngle = static_cast<float>(M_PI / 2.0);
    
    Quat q(expectedAxis, expectedAngle);
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // Check angle
    REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-5f));
    
    // Check axis
    REQUIRE_THAT(axis.x, WithinAbs(expectedAxis.x, 1e-5f));
    REQUIRE_THAT(axis.y, WithinAbs(expectedAxis.y, 1e-5f));
    REQUIRE_THAT(axis.z, WithinAbs(expectedAxis.z, 1e-5f));
    
    // Axis should be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Conversion: 90° rotation around Y-axis (quat to axis-angle)", "[conversion][quat-to-axis-angle]") {
    Vec3 expectedAxis(0.0f, 1.0f, 0.0f);
    float expectedAngle = static_cast<float>(M_PI / 2.0);
    
    Quat q(expectedAxis, expectedAngle);
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // Check angle
    REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-5f));
    
    // Check axis
    REQUIRE_THAT(axis.x, WithinAbs(expectedAxis.x, 1e-5f));
    REQUIRE_THAT(axis.y, WithinAbs(expectedAxis.y, 1e-5f));
    REQUIRE_THAT(axis.z, WithinAbs(expectedAxis.z, 1e-5f));
    
    // Axis should be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Conversion: 90° rotation around Z-axis (quat to axis-angle)", "[conversion][quat-to-axis-angle]") {
    Vec3 expectedAxis(0.0f, 0.0f, 1.0f);
    float expectedAngle = static_cast<float>(M_PI / 2.0);
    
    Quat q(expectedAxis, expectedAngle);
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // Check angle
    REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-5f));
    
    // Check axis
    REQUIRE_THAT(axis.x, WithinAbs(expectedAxis.x, 1e-5f));
    REQUIRE_THAT(axis.y, WithinAbs(expectedAxis.y, 1e-5f));
    REQUIRE_THAT(axis.z, WithinAbs(expectedAxis.z, 1e-5f));
    
    // Axis should be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Conversion: 180° rotation around X-axis (quat to axis-angle)", "[conversion][quat-to-axis-angle]") {
    Vec3 expectedAxis(1.0f, 0.0f, 0.0f);
    float expectedAngle = static_cast<float>(M_PI);
    
    Quat q(expectedAxis, expectedAngle);
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // Check angle (should be π)
    REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-5f));
    
    // Check axis (may be negated due to quaternion double-cover)
    REQUIRE_THAT(std::abs(axis.x), WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(std::abs(axis.y), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(axis.z), WithinAbs(0.0f, 1e-5f));
    
    // Axis should be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Conversion: 180° rotation around Y-axis (quat to axis-angle)", "[conversion][quat-to-axis-angle]") {
    Vec3 expectedAxis(0.0f, 1.0f, 0.0f);
    float expectedAngle = static_cast<float>(M_PI);
    
    Quat q(expectedAxis, expectedAngle);
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // Check angle (should be π)
    REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-5f));
    
    // Check axis (may be negated due to quaternion double-cover)
    REQUIRE_THAT(std::abs(axis.x), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(axis.y), WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(std::abs(axis.z), WithinAbs(0.0f, 1e-5f));
    
    // Axis should be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Conversion: 180° rotation around Z-axis (quat to axis-angle)", "[conversion][quat-to-axis-angle]") {
    Vec3 expectedAxis(0.0f, 0.0f, 1.0f);
    float expectedAngle = static_cast<float>(M_PI);
    
    Quat q(expectedAxis, expectedAngle);
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // Check angle (should be π)
    REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-5f));
    
    // Check axis (may be negated due to quaternion double-cover)
    REQUIRE_THAT(std::abs(axis.x), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(axis.y), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(std::abs(axis.z), WithinAbs(1.0f, 1e-5f));
    
    // Axis should be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Conversion: Arbitrary rotation (quat to axis-angle)", "[conversion][quat-to-axis-angle]") {
    // 60° around axis (1, 1, 1) normalized
    Vec3 expectedAxis(1.0f, 1.0f, 1.0f);
    expectedAxis = expectedAxis.normalized();
    float expectedAngle = static_cast<float>(M_PI / 3.0);  // 60°
    
    Quat q(expectedAxis, expectedAngle);
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // Check angle
    REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-5f));
    
    // Check axis
    REQUIRE_THAT(axis.x, WithinAbs(expectedAxis.x, 1e-5f));
    REQUIRE_THAT(axis.y, WithinAbs(expectedAxis.y, 1e-5f));
    REQUIRE_THAT(axis.z, WithinAbs(expectedAxis.z, 1e-5f));
    
    // Axis should be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Conversion: Very small rotation (quat to axis-angle)", "[conversion][quat-to-axis-angle]") {
    // Very small rotation: 0.001 radians around X-axis
    Vec3 expectedAxis(1.0f, 0.0f, 0.0f);
    float expectedAngle = 0.001f;
    
    Quat q(expectedAxis, expectedAngle);
    Vec3 axis;
    float angle;
    
    toAxisAngle(q, axis, angle);
    
    // For very small angles, numerical precision may vary but should be close
    REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-4f));

    // Axis should still be unit vector
    REQUIRE_THAT(axis.length(), WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Conversion: Non-unit quaternion handles normalization (axis-angle)", "[conversion][quat-to-axis-angle]") {
    // Create a quaternion and scale it (making it non-unit)
    Vec3 axis(0.0f, 1.0f, 0.0f);
    float angle = static_cast<float>(M_PI / 4.0);  // 45°
    Quat q(axis, angle);
    
    // Scale quaternion (make it non-unit)
    Quat nonUnitQ = q * 2.0f;
    
    Vec3 recoveredAxis;
    float recoveredAngle;
    
    toAxisAngle(nonUnitQ, recoveredAxis, recoveredAngle);
    
    // Should normalize internally and produce correct result
    REQUIRE_THAT(recoveredAngle, WithinAbs(angle, 1e-5f));
    REQUIRE_THAT(recoveredAxis.x, WithinAbs(axis.x, 1e-5f));
    REQUIRE_THAT(recoveredAxis.y, WithinAbs(axis.y, 1e-5f));
    REQUIRE_THAT(recoveredAxis.z, WithinAbs(axis.z, 1e-5f));
}

TEST_CASE("Round-trip: Axis-Angle -> Quat -> Axis-Angle preserves rotation", "[conversion][round-trip][axis-angle]") {
    SECTION("90° around X") {
        Vec3 originalAxis(1.0f, 0.0f, 0.0f);
        float originalAngle = static_cast<float>(M_PI / 2.0);
        
        Quat q(originalAxis, originalAngle);
        
        Vec3 recoveredAxis;
        float recoveredAngle;
        toAxisAngle(q, recoveredAxis, recoveredAngle);
        
        REQUIRE_THAT(recoveredAngle, WithinAbs(originalAngle, 1e-5f));
        REQUIRE_THAT(recoveredAxis.x, WithinAbs(originalAxis.x, 1e-5f));
        REQUIRE_THAT(recoveredAxis.y, WithinAbs(originalAxis.y, 1e-5f));
        REQUIRE_THAT(recoveredAxis.z, WithinAbs(originalAxis.z, 1e-5f));
    }
    
    SECTION("45° around diagonal axis") {
        Vec3 originalAxis(1.0f, 1.0f, 1.0f);
        originalAxis = originalAxis.normalized();
        float originalAngle = static_cast<float>(M_PI / 4.0);
        
        Quat q(originalAxis, originalAngle);
        
        Vec3 recoveredAxis;
        float recoveredAngle;
        toAxisAngle(q, recoveredAxis, recoveredAngle);
        
        REQUIRE_THAT(recoveredAngle, WithinAbs(originalAngle, 1e-5f));
        REQUIRE_THAT(recoveredAxis.x, WithinAbs(originalAxis.x, 1e-5f));
        REQUIRE_THAT(recoveredAxis.y, WithinAbs(originalAxis.y, 1e-5f));
        REQUIRE_THAT(recoveredAxis.z, WithinAbs(originalAxis.z, 1e-5f));
    }
    
    SECTION("180° rotation") {
        Vec3 originalAxis(0.0f, 1.0f, 0.0f);
        float originalAngle = static_cast<float>(M_PI);
        
        Quat q(originalAxis, originalAngle);
        
        Vec3 recoveredAxis;
        float recoveredAngle;
        toAxisAngle(q, recoveredAxis, recoveredAngle);
        
        REQUIRE_THAT(recoveredAngle, WithinAbs(originalAngle, 1e-5f));
        // For 180°, axis may be negated (both represent same rotation)
        float axisDot = recoveredAxis.dot(originalAxis);
        REQUIRE_THAT(std::abs(axisDot), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("Angle range: toAxisAngle always returns angle in [0, π]", "[conversion][quat-to-axis-angle][validation]") {
    Vec3 axis(0.0f, 0.0f, 1.0f);

    // Test various angles, including those outside [0, π]
    for (int i = -720; i <= 720; i += 45) {
        float inputAngle = static_cast<float>(i) * mathf::deg_to_rad;
        Quat q(axis, inputAngle);
        
        Vec3 recoveredAxis;
        float recoveredAngle;
        toAxisAngle(q, recoveredAxis, recoveredAngle);
        
        // Angle should always be in [0, π]
        REQUIRE(recoveredAngle >= 0.0f);
        REQUIRE(recoveredAngle <= mathf::pi + 1e-5f);
    }
}


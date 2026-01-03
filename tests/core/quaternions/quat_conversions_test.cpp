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

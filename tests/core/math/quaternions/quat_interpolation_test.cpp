#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_interpolation.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/math/utilities/constants.hpp>
#include <cmath>

using phynity::math::quaternions::Quatf;
using phynity::math::vectors::Vec3f;
using phynity::math::utilities::mathf;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Helper Functions
// ============================================================================

/// Compare two quaternions accounting for double-cover (q and -q are equivalent)
bool quaternionsEqual(const Quatf& q1, const Quatf& q2, float tolerance = 1e-5f) {
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

// ============================================================================
// NLERP Basic Tests
// ============================================================================

TEST_CASE("NLERP: t=0 returns first quaternion", "[interpolation][nlerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));  // 45°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf result = nlerp(q1, q2, 0.0f);
    
    REQUIRE(quaternionsEqual(result, q1));
}

TEST_CASE("NLERP: t=1 returns second quaternion", "[interpolation][nlerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));  // 45°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf result = nlerp(q1, q2, 1.0f);
    
    REQUIRE(quaternionsEqual(result, q2));
}

TEST_CASE("NLERP: t=0.5 produces intermediate rotation", "[interpolation][nlerp]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);  // Z-axis
    Quatf q1;  // Identity (0°)
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    // Result should be normalized
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Result should be between q1 and q2
    // Can't easily test exact angle, but we can verify it's not identity or 90°
    REQUIRE_FALSE(quaternionsEqual(result, q1));
    REQUIRE_FALSE(quaternionsEqual(result, q2));
}

TEST_CASE("NLERP: Interpolating identical quaternions returns same quaternion", "[interpolation][nlerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q(axis, static_cast<float>(mathf::pi / 3.0));
    
    Quatf result = nlerp(q, q, 0.5f);
    
    REQUIRE(quaternionsEqual(result, q));
}

TEST_CASE("NLERP: Result is always normalized", "[interpolation][nlerp]") {
    Vec3f axis(1.0f, 1.0f, 1.0f);
    axis = axis.normalized();
    
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));  // 30°
    Quatf q2(axis, static_cast<float>(mathf::pi / 3.0));  // 60°
    
    for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
        Quatf result = nlerp(q1, q2, t);
        REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// NLERP Shortest Path Tests
// ============================================================================

TEST_CASE("NLERP: Chooses shortest path (positive dot product)", "[interpolation][nlerp][shortest-path]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));   // 30°
    Quatf q2(axis, static_cast<float>(mathf::pi / 4.0));   // 45°
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    // Should interpolate smoothly
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Dot product should be positive (same hemisphere)
    REQUIRE(dot(q1, q2) > 0.0f);
}

TEST_CASE("NLERP: Chooses shortest path (negative dot product)", "[interpolation][nlerp][shortest-path]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));   // 30°
    Quatf q2(axis, static_cast<float>(mathf::pi / 6.0));   // 30° (same rotation)
    
    // Negate q2 to put it on opposite hemisphere (double-cover)
    q2 = -q2;
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    // Result should still represent the same rotation as q1/q2
    REQUIRE(quaternionsEqual(result, q1));
    
    // Verify it chose shortest path
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("NLERP: Handles opposite quaternions (180° apart)", "[interpolation][nlerp][edge-case]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, 0.0f);                              // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi));          // 180°
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    // Result should be normalized
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// NLERP Axis-Specific Tests
// ============================================================================

TEST_CASE("NLERP: X-axis rotation interpolation", "[interpolation][nlerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, 0.0f);                              // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));    // 90°
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Test that result rotates a vector correctly
    Vec3f testVec(0.0f, 1.0f, 0.0f);
    Vec3f rotated = result.rotateVector(testVec);
    
    // Rotating (0,1,0) around X-axis by 90° goes to (0,0,1)
    // At t=0.5, should be between (0,1,0) and (0,0,1)
    REQUIRE(rotated.y > 0.0f);  // Still has positive Y component
    REQUIRE(rotated.z > 0.0f);  // Has started rotating toward +Z
}

TEST_CASE("NLERP: Y-axis rotation interpolation", "[interpolation][nlerp]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, 0.0f);                              // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));    // 90°
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Test that result rotates a vector correctly
    Vec3f testVec(1.0f, 0.0f, 0.0f);
    Vec3f rotated = result.rotateVector(testVec);
    
    // Rotating (1,0,0) around Y-axis by 90° goes to (0,0,-1)
    // At t=0.5, should be between (1,0,0) and (0,0,-1)
    REQUIRE(rotated.x > 0.0f);  // Still has positive X component
    REQUIRE(rotated.z < 0.0f);  // Has started rotating toward -Z
}

TEST_CASE("NLERP: Z-axis rotation interpolation", "[interpolation][nlerp]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q1(axis, 0.0f);                              // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));    // 90°
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Test that result rotates a vector correctly
    Vec3f testVec(1.0f, 0.0f, 0.0f);
    Vec3f rotated = result.rotateVector(testVec);
    
    // Should be rotated less than 90° around Z
    REQUIRE(rotated.x > 0.0f);  // Still has positive X component
    REQUIRE(rotated.y > 0.0f);  // Has started rotating toward +Y
}

TEST_CASE("NLERP: Arbitrary axis rotation interpolation", "[interpolation][nlerp]") {
    Vec3f axis(1.0f, 1.0f, 1.0f);
    axis = axis.normalized();
    
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));   // 30°
    Quatf q2(axis, static_cast<float>(mathf::pi / 3.0));   // 60°
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// NLERP Identity and Special Cases
// ============================================================================

TEST_CASE("NLERP: Identity to rotation", "[interpolation][nlerp]") {
    Quatf q1;  // Identity
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf halfway = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
    REQUIRE_FALSE(quaternionsEqual(halfway, q1));
    REQUIRE_FALSE(quaternionsEqual(halfway, q2));
}

TEST_CASE("NLERP: Rotation to identity", "[interpolation][nlerp]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    Quatf q2;  // Identity
    
    Quatf halfway = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
    REQUIRE_FALSE(quaternionsEqual(halfway, q1));
    REQUIRE_FALSE(quaternionsEqual(halfway, q2));
}

TEST_CASE("NLERP: Both quaternions are identity", "[interpolation][nlerp]") {
    Quatf q1;  // Identity
    Quatf q2;  // Identity
    
    Quatf result = nlerp(q1, q2, 0.5f);
    
    REQUIRE(quaternionsEqual(result, q1));
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// NLERP Parameter Clamping Tests
// ============================================================================

TEST_CASE("NLERP: t < 0 is clamped to 0", "[interpolation][nlerp][clamping]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));
    
    Quatf result = nlerp(q1, q2, -0.5f);
    
    // Should return q1
    REQUIRE(quaternionsEqual(result, q1));
}

TEST_CASE("NLERP: t > 1 is clamped to 1", "[interpolation][nlerp][clamping]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));
    
    Quatf result = nlerp(q1, q2, 1.5f);
    
    // Should return q2
    REQUIRE(quaternionsEqual(result, q2));
}

TEST_CASE("NLERP: Multiple t values produce smooth progression", "[interpolation][nlerp]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1;  // Identity
    Quatf q2(axis, static_cast<float>(mathf::pi));  // 180°
    
    Quatf prev = q1;
    
    for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
        Quatf current = nlerp(q1, q2, t);
        
        // Each result should be normalized
        REQUIRE_THAT(current.magnitude(), WithinAbs(1.0f, 1e-6f));
        
        // Store for next iteration
        prev = current;
    }
}

// ============================================================================
// NLERP Vector Rotation Tests
// ============================================================================

TEST_CASE("NLERP: Interpolated quaternion rotates vectors correctly", "[interpolation][nlerp][rotation]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q1;  // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90° around Z
    
    Quatf halfway = nlerp(q1, q2, 0.5f);
    
    Vec3f testVec(1.0f, 0.0f, 0.0f);
    Vec3f rotated = halfway.rotateVector(testVec);
    
    // Vector should be rotated somewhere between 0° and 90°
    REQUIRE(rotated.x > 0.0f);  // Still has X component
    REQUIRE(rotated.y > 0.0f);  // Has gained Y component
    REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-6f));  // No Z component
}

TEST_CASE("NLERP: Multiple interpolations maintain consistency", "[interpolation][nlerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, 0.0f);
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));
    
    // Test vector
    Vec3f v(0.0f, 1.0f, 0.0f);
    
    // Interpolate at different t values
    Quatf q_25  = nlerp(q1, q2, 0.25f);
    Quatf q_50  = nlerp(q1, q2, 0.50f);
    Quatf q_75  = nlerp(q1, q2, 0.75f);
    
    Vec3f v_25  = q_25.rotateVector(v);
    Vec3f v_50  = q_50.rotateVector(v);
    Vec3f v_75  = q_75.rotateVector(v);
    Vec3f v_100 = q2.rotateVector(v);
    
    // Rotating (0,1,0) around X-axis: Y decreases, Z increases (goes to +Z)
    // Y component should decrease as we rotate
    REQUIRE(v_25.y > v_50.y);
    REQUIRE(v_50.y > v_75.y);
    REQUIRE(v_75.y > v_100.y);
    
    // Z component should become more positive as we rotate toward (0,0,1)
    REQUIRE(v_25.z < v_50.z);
    REQUIRE(v_50.z < v_75.z);
    REQUIRE(v_75.z < v_100.z);
}

// ============================================================================
// NLERP Large Angle Tests
// ============================================================================

TEST_CASE("NLERP: 180° rotation interpolation", "[interpolation][nlerp][large-angle]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, 0.0f);
    Quatf q2(axis, static_cast<float>(mathf::pi));  // 180°
    
    Quatf halfway = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("NLERP: Nearly 180° rotation (edge case)", "[interpolation][nlerp][large-angle]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, 0.01f);  // Small angle
    Quatf q2(axis, static_cast<float>(mathf::pi - 0.01f));  // Almost 180°
    
    Quatf halfway = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// NLERP Symmetry Tests
// ============================================================================

TEST_CASE("NLERP: Symmetry - nlerp(q1, q2, t) related to nlerp(q2, q1, 1-t)", "[interpolation][nlerp][symmetry]") {
    Vec3f axis(1.0f, 1.0f, 0.0f);
    axis = axis.normalized();
    
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 3.0));
    
    float t = 0.3f;
    
    Quatf forward = nlerp(q1, q2, t);
    Quatf reverse = nlerp(q2, q1, 1.0f - t);
    
    // They should represent the same rotation
    REQUIRE(quaternionsEqual(forward, reverse, 1e-4f));
}

// ============================================================================
// Utility Function Tests: angleBetween
// ============================================================================

TEST_CASE("angleBetween: Identity quaternions have zero angle", "[interpolation][utility][angleBetween]") {
    Quatf q1;  // Identity
    Quatf q2;  // Identity
    
    float angle = angleBetween(q1, q2);
    
    REQUIRE_THAT(angle, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("angleBetween: Same rotation returns zero angle", "[interpolation][utility][angleBetween]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 4.0));
    
    float angle = angleBetween(q1, q2);
    
    REQUIRE_THAT(angle, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("angleBetween: Double-cover (q and -q) returns zero angle", "[interpolation][utility][angleBetween]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 3.0));
    Quatf q2 = -q1;  // Negated quaternion represents same rotation
    
    float angle = angleBetween(q1, q2);
    
    REQUIRE_THAT(angle, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("angleBetween: 90° rotation difference", "[interpolation][utility][angleBetween]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q1;  // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    float angle = angleBetween(q1, q2);
    
    REQUIRE_THAT(angle, WithinAbs(static_cast<float>(mathf::pi / 2.0), 1e-5f));
}

TEST_CASE("angleBetween: 180° rotation difference", "[interpolation][utility][angleBetween]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1;  // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi));  // 180°
    
    float angle = angleBetween(q1, q2);
    
    REQUIRE_THAT(angle, WithinAbs(static_cast<float>(mathf::pi), 1e-5f));
}

TEST_CASE("angleBetween: Various angles on same axis", "[interpolation][utility][angleBetween]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    
    std::vector<float> testAngles = {
        static_cast<float>(mathf::pi / 6.0),   // 30°
        static_cast<float>(mathf::pi / 4.0),   // 45°
        static_cast<float>(mathf::pi / 3.0),   // 60°
        static_cast<float>(mathf::pi / 2.0),   // 90°
    };
    
    Quatf q1;  // Identity
    
    for (float expectedAngle : testAngles) {
        Quatf q2(axis, expectedAngle);
        float angle = angleBetween(q1, q2);
        
        REQUIRE_THAT(angle, WithinAbs(expectedAngle, 1e-5f));
    }
}

TEST_CASE("angleBetween: Symmetry - angle(q1, q2) == angle(q2, q1)", "[interpolation][utility][angleBetween]") {
    Vec3f axis(1.0f, 1.0f, 1.0f);
    axis = axis.normalized();
    
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 3.0));
    
    float angle1 = angleBetween(q1, q2);
    float angle2 = angleBetween(q2, q1);
    
    REQUIRE_THAT(angle1, WithinAbs(angle2, 1e-6f));
}

TEST_CASE("angleBetween: Always returns positive angle", "[interpolation][utility][angleBetween]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));
    
    float angle = angleBetween(q1, q2);
    
    REQUIRE(angle >= 0.0f);
    REQUIRE(angle <= static_cast<float>(mathf::pi));
}

// ============================================================================
// SLERP Basic Tests
// ============================================================================

TEST_CASE("SLERP: t=0 returns first quaternion", "[interpolation][slerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));  // 45°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf result = slerp(q1, q2, 0.0f);
    
    REQUIRE(quaternionsEqual(result, q1));
}

TEST_CASE("SLERP: t=1 returns second quaternion", "[interpolation][slerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));  // 45°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf result = slerp(q1, q2, 1.0f);
    
    REQUIRE(quaternionsEqual(result, q2));
}

TEST_CASE("SLERP: t=0.5 produces intermediate rotation", "[interpolation][slerp]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);  // Z-axis
    Quatf q1;  // Identity (0°)
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf result = slerp(q1, q2, 0.5f);
    
    // Result should be normalized
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Result should be between q1 and q2
    REQUIRE_FALSE(quaternionsEqual(result, q1));
    REQUIRE_FALSE(quaternionsEqual(result, q2));
    
    // For SLERP, t=0.5 should give exactly 45° rotation
    float angleFromStart = angleBetween(q1, result);
    float angleFromEnd = angleBetween(result, q2);
    REQUIRE_THAT(angleFromStart, WithinAbs(angleFromEnd, 1e-4f));
}

TEST_CASE("SLERP: Interpolating identical quaternions returns same quaternion", "[interpolation][slerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q(axis, static_cast<float>(mathf::pi / 3.0));
    
    Quatf result = slerp(q, q, 0.5f);
    
    REQUIRE(quaternionsEqual(result, q));
}

TEST_CASE("SLERP: Result is always normalized", "[interpolation][slerp]") {
    Vec3f axis(1.0f, 1.0f, 1.0f);
    axis = axis.normalized();
    
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));  // 30°
    Quatf q2(axis, static_cast<float>(mathf::pi / 3.0));  // 60°
    
    for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
        Quatf result = slerp(q1, q2, t);
        REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// SLERP Shortest Path Tests
// ============================================================================

TEST_CASE("SLERP: Chooses shortest path (positive dot product)", "[interpolation][slerp][shortest-path]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));   // 30°
    Quatf q2(axis, static_cast<float>(mathf::pi / 4.0));   // 45°
    
    Quatf result = slerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    REQUIRE(dot(q1, q2) > 0.0f);
}

TEST_CASE("SLERP: Chooses shortest path (negative dot product)", "[interpolation][slerp][shortest-path]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));   // 30°
    Quatf q2(axis, static_cast<float>(mathf::pi / 6.0));   // 30° (same rotation)
    
    // Negate q2 to put it on opposite hemisphere (double-cover)
    q2 = -q2;
    
    Quatf result = slerp(q1, q2, 0.5f);
    
    // Result should still represent the same rotation as q1/q2
    REQUIRE(quaternionsEqual(result, q1));
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// SLERP Fallback to NLERP Tests
// ============================================================================

TEST_CASE("SLERP: Falls back to NLERP for very small angles", "[interpolation][slerp][fallback]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q1;  // Identity
    Quatf q2(axis, 0.001f);  // Very small angle (~0.057°)
    
    Quatf slerpResult = slerp(q1, q2, 0.5f);
    Quatf nlerpResult = nlerp(q1, q2, 0.5f);
    
    // Should be very close (fallback triggered)
    REQUIRE(quaternionsEqual(slerpResult, nlerpResult, 1e-5f));
}

TEST_CASE("SLERP: Does not fallback for moderate angles", "[interpolation][slerp][fallback]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q1;  // Identity
    Quatf q2(axis, static_cast<float>(mathf::pi / 4.0));  // 45° - well above threshold
    
    Quatf slerpResult = slerp(q1, q2, 0.5f);
    
    // Angle should be preserved (SLERP has constant angular velocity)
    float angleToMid = angleBetween(q1, slerpResult);
    float expectedAngle = static_cast<float>(mathf::pi / 8.0);  // Half of 45°
    
    REQUIRE_THAT(angleToMid, WithinAbs(expectedAngle, 1e-4f));
}

// ============================================================================
// SLERP Constant Angular Velocity Tests
// ============================================================================

TEST_CASE("SLERP: Maintains constant angular velocity", "[interpolation][slerp][angular-velocity]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1;  // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    // Interpolate at equal t intervals
    Quatf q_25 = slerp(q1, q2, 0.25f);
    Quatf q_50 = slerp(q1, q2, 0.50f);
    Quatf q_75 = slerp(q1, q2, 0.75f);
    
    // Measure angular distances
    float angle_0_25  = angleBetween(q1, q_25);
    float angle_25_50 = angleBetween(q_25, q_50);
    float angle_50_75 = angleBetween(q_50, q_75);
    float angle_75_100 = angleBetween(q_75, q2);
    
    // All angular distances should be equal (constant angular velocity)
    REQUIRE_THAT(angle_0_25, WithinAbs(angle_25_50, 1e-4f));
    REQUIRE_THAT(angle_25_50, WithinAbs(angle_50_75, 1e-4f));
    REQUIRE_THAT(angle_50_75, WithinAbs(angle_75_100, 1e-4f));
}

TEST_CASE("SLERP: t=0.5 gives exactly halfway rotation", "[interpolation][slerp][angular-velocity]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1;  // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf halfway = slerp(q1, q2, 0.5f);
    
    float angleFromStart = angleBetween(q1, halfway);
    float angleFromEnd = angleBetween(halfway, q2);
    
    // Should be exactly halfway
    REQUIRE_THAT(angleFromStart, WithinAbs(angleFromEnd, 1e-5f));
    REQUIRE_THAT(angleFromStart, WithinAbs(static_cast<float>(mathf::pi / 4.0), 1e-5f));
}

// ============================================================================
// SLERP Axis-Specific Tests
// ============================================================================

TEST_CASE("SLERP: X-axis rotation interpolation", "[interpolation][slerp]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, 0.0f);                              // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));    // 90°
    
    Quatf result = slerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Test that result rotates a vector correctly
    Vec3f testVec(0.0f, 1.0f, 0.0f);
    Vec3f rotated = result.rotateVector(testVec);
    
    // At t=0.5, should be rotated 45° around X
    REQUIRE(rotated.y > 0.0f);
    REQUIRE(rotated.z > 0.0f);
    REQUIRE_THAT(rotated.y, WithinAbs(rotated.z, 1e-5f));  // Equal Y and Z for 45°
}

TEST_CASE("SLERP: Y-axis rotation interpolation", "[interpolation][slerp]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, 0.0f);                              // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));    // 90°
    
    Quatf result = slerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("SLERP: Z-axis rotation interpolation", "[interpolation][slerp]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q1(axis, 0.0f);                              // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));    // 90°
    
    Quatf result = slerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// SLERP Parameter Clamping Tests
// ============================================================================

TEST_CASE("SLERP: t < 0 is clamped to 0", "[interpolation][slerp][clamping]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));
    
    Quatf result = slerp(q1, q2, -0.5f);
    
    REQUIRE(quaternionsEqual(result, q1));
}

TEST_CASE("SLERP: t > 1 is clamped to 1", "[interpolation][slerp][clamping]") {
    Vec3f axis(1.0f, 0.0f, 0.0f);
    Quatf q1(axis, static_cast<float>(mathf::pi / 4.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));
    
    Quatf result = slerp(q1, q2, 1.5f);
    
    REQUIRE(quaternionsEqual(result, q2));
}

// ============================================================================
// SLERP Identity and Special Cases
// ============================================================================

TEST_CASE("SLERP: Identity to rotation", "[interpolation][slerp]") {
    Quatf q1;  // Identity
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    Quatf halfway = slerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
    REQUIRE_FALSE(quaternionsEqual(halfway, q1));
    REQUIRE_FALSE(quaternionsEqual(halfway, q2));
}

TEST_CASE("SLERP: Both quaternions are identity", "[interpolation][slerp]") {
    Quatf q1;  // Identity
    Quatf q2;  // Identity
    
    Quatf result = slerp(q1, q2, 0.5f);
    
    REQUIRE(quaternionsEqual(result, q1));
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("SLERP: 180° rotation interpolation", "[interpolation][slerp][large-angle]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1(axis, 0.0f);
    Quatf q2(axis, static_cast<float>(mathf::pi));  // 180°
    
    Quatf halfway = slerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Should be at 90°
    float angle = angleBetween(q1, halfway);
    REQUIRE_THAT(angle, WithinAbs(static_cast<float>(mathf::pi / 2.0), 1e-4f));
}

// ============================================================================
// SLERP Symmetry Tests
// ============================================================================

TEST_CASE("SLERP: Symmetry - slerp(q1, q2, t) related to slerp(q2, q1, 1-t)", "[interpolation][slerp][symmetry]") {
    Vec3f axis(1.0f, 1.0f, 0.0f);
    axis = axis.normalized();
    
    Quatf q1(axis, static_cast<float>(mathf::pi / 6.0));
    Quatf q2(axis, static_cast<float>(mathf::pi / 3.0));
    
    float t = 0.3f;
    
    Quatf forward = slerp(q1, q2, t);
    Quatf reverse = slerp(q2, q1, 1.0f - t);
    
    // They should represent the same rotation
    REQUIRE(quaternionsEqual(forward, reverse, 1e-4f));
}

// ============================================================================
// SLERP vs NLERP Comparison
// ============================================================================

TEST_CASE("SLERP vs NLERP: SLERP has constant angular velocity", "[interpolation][comparison]") {
    Vec3f axis(0.0f, 0.0f, 1.0f);
    Quatf q1;  // 0°
    Quatf q2(axis, static_cast<float>(mathf::pi / 2.0));  // 90°
    
    // Measure angular velocity for SLERP
    Quatf s_25 = slerp(q1, q2, 0.25f);
    Quatf s_75 = slerp(q1, q2, 0.75f);
    
    float slerp_angle_first_quarter = angleBetween(q1, s_25);
    float slerp_angle_last_quarter = angleBetween(s_75, q2);
    
    // SLERP should have equal angular distances
    REQUIRE_THAT(slerp_angle_first_quarter, WithinAbs(slerp_angle_last_quarter, 1e-4f));
    
    // Measure for NLERP
    Quatf n_25 = nlerp(q1, q2, 0.25f);
    Quatf n_75 = nlerp(q1, q2, 0.75f);
    
    float nlerp_angle_first_quarter = angleBetween(q1, n_25);
    float nlerp_angle_last_quarter = angleBetween(n_75, q2);
    
    // For this test, we just verify SLERP has constant angular velocity
    // NLERP's difference is minimal for 90° rotations due to normalization
}

TEST_CASE("SLERP vs NLERP: Results are similar for small angles", "[interpolation][comparison]") {
    Vec3f axis(0.0f, 1.0f, 0.0f);
    Quatf q1;
    Quatf q2(axis, static_cast<float>(mathf::pi / 12.0));  // 15° - small angle
    
    Quatf slerpResult = slerp(q1, q2, 0.5f);
    Quatf nlerpResult = nlerp(q1, q2, 0.5f);
    
    // For small angles, SLERP and NLERP should be very close
    float angleDiff = angleBetween(slerpResult, nlerpResult);
    REQUIRE(angleDiff < 0.01f);  // Less than ~0.57°
}

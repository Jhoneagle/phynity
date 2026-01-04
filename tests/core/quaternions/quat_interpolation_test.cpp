#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_interpolation.hpp>
#include <core/math/vectors/vec3.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace phynity::math::quaternions;
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

// ============================================================================
// NLERP Basic Tests
// ============================================================================

TEST_CASE("NLERP: t=0 returns first quaternion", "[interpolation][nlerp]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q1(axis, static_cast<float>(M_PI / 4.0));  // 45°
    Quat q2(axis, static_cast<float>(M_PI / 2.0));  // 90°
    
    Quat result = nlerp(q1, q2, 0.0f);
    
    REQUIRE(quaternionsEqual(result, q1));
}

TEST_CASE("NLERP: t=1 returns second quaternion", "[interpolation][nlerp]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q1(axis, static_cast<float>(M_PI / 4.0));  // 45°
    Quat q2(axis, static_cast<float>(M_PI / 2.0));  // 90°
    
    Quat result = nlerp(q1, q2, 1.0f);
    
    REQUIRE(quaternionsEqual(result, q2));
}

TEST_CASE("NLERP: t=0.5 produces intermediate rotation", "[interpolation][nlerp]") {
    Vec3 axis(0.0f, 0.0f, 1.0f);  // Z-axis
    Quat q1;  // Identity (0°)
    Quat q2(axis, static_cast<float>(M_PI / 2.0));  // 90°
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    // Result should be normalized
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Result should be between q1 and q2
    // Can't easily test exact angle, but we can verify it's not identity or 90°
    REQUIRE_FALSE(quaternionsEqual(result, q1));
    REQUIRE_FALSE(quaternionsEqual(result, q2));
}

TEST_CASE("NLERP: Interpolating identical quaternions returns same quaternion", "[interpolation][nlerp]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q(axis, static_cast<float>(M_PI / 3.0));
    
    Quat result = nlerp(q, q, 0.5f);
    
    REQUIRE(quaternionsEqual(result, q));
}

TEST_CASE("NLERP: Result is always normalized", "[interpolation][nlerp]") {
    Vec3 axis(1.0f, 1.0f, 1.0f);
    axis = axis.normalized();
    
    Quat q1(axis, static_cast<float>(M_PI / 6.0));  // 30°
    Quat q2(axis, static_cast<float>(M_PI / 3.0));  // 60°
    
    for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
        Quat result = nlerp(q1, q2, t);
        REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// NLERP Shortest Path Tests
// ============================================================================

TEST_CASE("NLERP: Chooses shortest path (positive dot product)", "[interpolation][nlerp][shortest-path]") {
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q1(axis, static_cast<float>(M_PI / 6.0));   // 30°
    Quat q2(axis, static_cast<float>(M_PI / 4.0));   // 45°
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    // Should interpolate smoothly
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Dot product should be positive (same hemisphere)
    REQUIRE(dot(q1, q2) > 0.0f);
}

TEST_CASE("NLERP: Chooses shortest path (negative dot product)", "[interpolation][nlerp][shortest-path]") {
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q1(axis, static_cast<float>(M_PI / 6.0));   // 30°
    Quat q2(axis, static_cast<float>(M_PI / 6.0));   // 30° (same rotation)
    
    // Negate q2 to put it on opposite hemisphere (double-cover)
    q2 = -q2;
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    // Result should still represent the same rotation as q1/q2
    REQUIRE(quaternionsEqual(result, q1));
    
    // Verify it chose shortest path
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("NLERP: Handles opposite quaternions (180° apart)", "[interpolation][nlerp][edge-case]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q1(axis, 0.0f);                              // 0°
    Quat q2(axis, static_cast<float>(M_PI));          // 180°
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    // Result should be normalized
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// NLERP Axis-Specific Tests
// ============================================================================

TEST_CASE("NLERP: X-axis rotation interpolation", "[interpolation][nlerp]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q1(axis, 0.0f);                              // 0°
    Quat q2(axis, static_cast<float>(M_PI / 2.0));    // 90°
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Test that result rotates a vector correctly
    Vec3 testVec(0.0f, 1.0f, 0.0f);
    Vec3 rotated = result.rotateVector(testVec);
    
    // Rotating (0,1,0) around X-axis by 90° goes to (0,0,1)
    // At t=0.5, should be between (0,1,0) and (0,0,1)
    REQUIRE(rotated.y > 0.0f);  // Still has positive Y component
    REQUIRE(rotated.z > 0.0f);  // Has started rotating toward +Z
}

TEST_CASE("NLERP: Y-axis rotation interpolation", "[interpolation][nlerp]") {
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q1(axis, 0.0f);                              // 0°
    Quat q2(axis, static_cast<float>(M_PI / 2.0));    // 90°
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Test that result rotates a vector correctly
    Vec3 testVec(1.0f, 0.0f, 0.0f);
    Vec3 rotated = result.rotateVector(testVec);
    
    // Rotating (1,0,0) around Y-axis by 90° goes to (0,0,-1)
    // At t=0.5, should be between (1,0,0) and (0,0,-1)
    REQUIRE(rotated.x > 0.0f);  // Still has positive X component
    REQUIRE(rotated.z < 0.0f);  // Has started rotating toward -Z
}

TEST_CASE("NLERP: Z-axis rotation interpolation", "[interpolation][nlerp]") {
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q1(axis, 0.0f);                              // 0°
    Quat q2(axis, static_cast<float>(M_PI / 2.0));    // 90°
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
    
    // Test that result rotates a vector correctly
    Vec3 testVec(1.0f, 0.0f, 0.0f);
    Vec3 rotated = result.rotateVector(testVec);
    
    // Should be rotated less than 90° around Z
    REQUIRE(rotated.x > 0.0f);  // Still has positive X component
    REQUIRE(rotated.y > 0.0f);  // Has started rotating toward +Y
}

TEST_CASE("NLERP: Arbitrary axis rotation interpolation", "[interpolation][nlerp]") {
    Vec3 axis(1.0f, 1.0f, 1.0f);
    axis = axis.normalized();
    
    Quat q1(axis, static_cast<float>(M_PI / 6.0));   // 30°
    Quat q2(axis, static_cast<float>(M_PI / 3.0));   // 60°
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// NLERP Identity and Special Cases
// ============================================================================

TEST_CASE("NLERP: Identity to rotation", "[interpolation][nlerp]") {
    Quat q1;  // Identity
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q2(axis, static_cast<float>(M_PI / 2.0));  // 90°
    
    Quat halfway = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
    REQUIRE_FALSE(quaternionsEqual(halfway, q1));
    REQUIRE_FALSE(quaternionsEqual(halfway, q2));
}

TEST_CASE("NLERP: Rotation to identity", "[interpolation][nlerp]") {
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q1(axis, static_cast<float>(M_PI / 2.0));  // 90°
    Quat q2;  // Identity
    
    Quat halfway = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
    REQUIRE_FALSE(quaternionsEqual(halfway, q1));
    REQUIRE_FALSE(quaternionsEqual(halfway, q2));
}

TEST_CASE("NLERP: Both quaternions are identity", "[interpolation][nlerp]") {
    Quat q1;  // Identity
    Quat q2;  // Identity
    
    Quat result = nlerp(q1, q2, 0.5f);
    
    REQUIRE(quaternionsEqual(result, q1));
    REQUIRE_THAT(result.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// NLERP Parameter Clamping Tests
// ============================================================================

TEST_CASE("NLERP: t < 0 is clamped to 0", "[interpolation][nlerp][clamping]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q1(axis, static_cast<float>(M_PI / 4.0));
    Quat q2(axis, static_cast<float>(M_PI / 2.0));
    
    Quat result = nlerp(q1, q2, -0.5f);
    
    // Should return q1
    REQUIRE(quaternionsEqual(result, q1));
}

TEST_CASE("NLERP: t > 1 is clamped to 1", "[interpolation][nlerp][clamping]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q1(axis, static_cast<float>(M_PI / 4.0));
    Quat q2(axis, static_cast<float>(M_PI / 2.0));
    
    Quat result = nlerp(q1, q2, 1.5f);
    
    // Should return q2
    REQUIRE(quaternionsEqual(result, q2));
}

TEST_CASE("NLERP: Multiple t values produce smooth progression", "[interpolation][nlerp]") {
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q1;  // Identity
    Quat q2(axis, static_cast<float>(M_PI));  // 180°
    
    Quat prev = q1;
    
    for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
        Quat current = nlerp(q1, q2, t);
        
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
    Vec3 axis(0.0f, 0.0f, 1.0f);
    Quat q1;  // 0°
    Quat q2(axis, static_cast<float>(M_PI / 2.0));  // 90° around Z
    
    Quat halfway = nlerp(q1, q2, 0.5f);
    
    Vec3 testVec(1.0f, 0.0f, 0.0f);
    Vec3 rotated = halfway.rotateVector(testVec);
    
    // Vector should be rotated somewhere between 0° and 90°
    REQUIRE(rotated.x > 0.0f);  // Still has X component
    REQUIRE(rotated.y > 0.0f);  // Has gained Y component
    REQUIRE_THAT(rotated.z, WithinAbs(0.0f, 1e-6f));  // No Z component
}

TEST_CASE("NLERP: Multiple interpolations maintain consistency", "[interpolation][nlerp]") {
    Vec3 axis(1.0f, 0.0f, 0.0f);
    Quat q1(axis, 0.0f);
    Quat q2(axis, static_cast<float>(M_PI / 2.0));
    
    // Test vector
    Vec3 v(0.0f, 1.0f, 0.0f);
    
    // Interpolate at different t values
    Quat q_25  = nlerp(q1, q2, 0.25f);
    Quat q_50  = nlerp(q1, q2, 0.50f);
    Quat q_75  = nlerp(q1, q2, 0.75f);
    
    Vec3 v_25  = q_25.rotateVector(v);
    Vec3 v_50  = q_50.rotateVector(v);
    Vec3 v_75  = q_75.rotateVector(v);
    Vec3 v_100 = q2.rotateVector(v);
    
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
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q1(axis, 0.0f);
    Quat q2(axis, static_cast<float>(M_PI));  // 180°
    
    Quat halfway = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("NLERP: Nearly 180° rotation (edge case)", "[interpolation][nlerp][large-angle]") {
    Vec3 axis(0.0f, 1.0f, 0.0f);
    Quat q1(axis, 0.01f);  // Small angle
    Quat q2(axis, static_cast<float>(M_PI - 0.01f));  // Almost 180°
    
    Quat halfway = nlerp(q1, q2, 0.5f);
    
    REQUIRE_THAT(halfway.magnitude(), WithinAbs(1.0f, 1e-6f));
}

// ============================================================================
// NLERP Symmetry Tests
// ============================================================================

TEST_CASE("NLERP: Symmetry - nlerp(q1, q2, t) related to nlerp(q2, q1, 1-t)", "[interpolation][nlerp][symmetry]") {
    Vec3 axis(1.0f, 1.0f, 0.0f);
    axis = axis.normalized();
    
    Quat q1(axis, static_cast<float>(M_PI / 6.0));
    Quat q2(axis, static_cast<float>(M_PI / 3.0));
    
    float t = 0.3f;
    
    Quat forward = nlerp(q1, q2, t);
    Quat reverse = nlerp(q2, q1, 1.0f - t);
    
    // They should represent the same rotation
    REQUIRE(quaternionsEqual(forward, reverse, 1e-4f));
}

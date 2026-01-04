#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <core/math/utilities/constants.hpp>
#include <core/math/utilities/conversions.hpp>
#include <core/math/utilities/trigonometry.hpp>
#include <core/math/utilities/numeric.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/math/utilities/geometry.hpp>

using namespace phynity::math::utilities;
using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

// ================================================================
// Constants Tests
// ================================================================

TEST_CASE("Constants: Mathematical constants", "[utilities][constants]") {
    REQUIRE_THAT(mathf::pi, WithinRel(3.14159265f, 1e-6f));
    REQUIRE_THAT(mathf::two_pi, WithinRel(6.28318530f, 1e-6f));
    REQUIRE_THAT(mathf::half_pi, WithinRel(1.57079632f, 1e-6f));
    REQUIRE_THAT(mathf::e, WithinRel(2.71828182f, 1e-6f));
}

TEST_CASE("Constants: Conversion factors", "[utilities][constants]") {
    REQUIRE_THAT(mathf::deg_to_rad * 180.0f, WithinRel(mathf::pi, 1e-5f));
    REQUIRE_THAT(mathf::rad_to_deg * mathf::pi, WithinRel(180.0f, 1e-5f));
}

// ================================================================
// Conversions Tests
// ================================================================

TEST_CASE("Conversions: Degrees to radians", "[utilities][conversions]") {
    REQUIRE_THAT(degrees_to_radians(180.0f), WithinAbs(mathf::pi, 1e-6f));
    REQUIRE_THAT(degrees_to_radians(90.0f), WithinAbs(mathf::half_pi, 1e-6f));
    REQUIRE_THAT(degrees_to_radians(0.0f), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(degrees_to_radians(-90.0f), WithinAbs(-mathf::half_pi, 1e-6f));
}

TEST_CASE("Conversions: Radians to degrees", "[utilities][conversions]") {
    REQUIRE_THAT(radians_to_degrees(mathf::pi), WithinAbs(180.0f, 1e-4f));
    REQUIRE_THAT(radians_to_degrees(mathf::half_pi), WithinAbs(90.0f, 1e-4f));
    REQUIRE_THAT(radians_to_degrees(0.0f), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Conversions: Aliases deg2rad and rad2deg", "[utilities][conversions]") {
    REQUIRE_THAT(deg2rad(45.0f), WithinAbs(mathf::pi / 4.0f, 1e-6f));
    REQUIRE_THAT(rad2deg(mathf::pi / 4.0f), WithinAbs(45.0f, 1e-4f));
}

// ================================================================
// Trigonometry Tests
// ================================================================

TEST_CASE("Trigonometry: Angle normalization", "[utilities][trigonometry]") {
    REQUIRE_THAT(normalize_angle(mathf::pi), WithinAbs(mathf::pi, 1e-6f));
    REQUIRE_THAT(normalize_angle(mathf::two_pi), WithinAbs(0.0f, 1e-6f));
    // 3*pi normalized should be close to pi or -pi (equivalent due to floating point)
    float normalized = normalize_angle(3 * mathf::pi);
    bool is_pi = std::abs(normalized - mathf::pi) < 1e-4f;
    bool is_neg_pi = std::abs(normalized + mathf::pi) < 1e-4f;
    REQUIRE((is_pi || is_neg_pi));
}

TEST_CASE("Trigonometry: sin/cos/tan normalized", "[utilities][trigonometry]") {
    REQUIRE_THAT(sin_normalized(0.0f), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(cos_normalized(0.0f), WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(sin_normalized(mathf::half_pi), WithinAbs(1.0f, 1e-5f));
    REQUIRE_THAT(cos_normalized(mathf::half_pi), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Trigonometry: Small angle approximations", "[utilities][trigonometry]") {
    const float small_angle = 0.05f;  // 2.86 degrees
    
    REQUIRE(is_small_angle(small_angle));
    REQUIRE_FALSE(is_small_angle(0.2f));
    
    // sin(θ) ≈ θ for small θ
    REQUIRE_THAT(sin_small_angle(small_angle), WithinRel(small_angle, 0.01f));
    
    // cos(θ) ≈ 1 - θ²/2 for small θ
    float expected_cos = 1.0f - small_angle * small_angle / 2.0f;
    REQUIRE_THAT(cos_small_angle(small_angle), WithinRel(expected_cos, 0.01f));
}

// ================================================================
// Numeric Helpers Tests
// ================================================================

TEST_CASE("Numeric: Clamping", "[utilities][numeric]") {
    REQUIRE_THAT(clamp(5.0f, 0.0f, 10.0f), WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(clamp(-5.0f, 0.0f, 10.0f), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(clamp(15.0f, 0.0f, 10.0f), WithinAbs(10.0f, 1e-6f));
}

TEST_CASE("Numeric: Linear interpolation", "[utilities][numeric]") {
    REQUIRE_THAT(lerp(0.0f, 10.0f, 0.0f), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(lerp(0.0f, 10.0f, 1.0f), WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(lerp(0.0f, 10.0f, 0.5f), WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(lerp(0.0f, 10.0f, 0.25f), WithinAbs(2.5f, 1e-6f));
}

TEST_CASE("Numeric: Inverse linear interpolation", "[utilities][numeric]") {
    REQUIRE_THAT(inverse_lerp(0.0f, 10.0f, 0.0f), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(inverse_lerp(0.0f, 10.0f, 10.0f), WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(inverse_lerp(0.0f, 10.0f, 5.0f), WithinAbs(0.5f, 1e-6f));
}

TEST_CASE("Numeric: Smoothstep", "[utilities][numeric]") {
    REQUIRE_THAT(smoothstep(0.0f, 1.0f, -1.0f), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(smoothstep(0.0f, 1.0f, 2.0f), WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(smoothstep(0.0f, 1.0f, 0.0f), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(smoothstep(0.0f, 1.0f, 1.0f), WithinAbs(1.0f, 1e-6f));
    
    // At 0.5, smoothstep should be exactly 0.5
    REQUIRE_THAT(smoothstep(0.0f, 1.0f, 0.5f), WithinAbs(0.5f, 1e-6f));
}

TEST_CASE("Numeric: Smootherstep", "[utilities][numeric]") {
    REQUIRE_THAT(smootherstep(0.0f, 1.0f, -1.0f), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(smootherstep(0.0f, 1.0f, 2.0f), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Numeric: Absolute value and sign", "[utilities][numeric]") {
    REQUIRE_THAT(abs(-5.0f), WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(abs(5.0f), WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(sign(5.0f), WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(sign(-5.0f), WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(sign(0.0f), WithinAbs(0.0f, 1e-6f));
}

// ================================================================
// Float Comparison Tests
// ================================================================

TEST_CASE("Float Comparison: Validity checks", "[utilities][float_comparison]") {
    REQUIRE(is_finite(1.0f));
    REQUIRE_FALSE(is_nan(1.0f));
    REQUIRE_FALSE(is_inf(1.0f));
    
    float nan_val = std::nanf("");
    REQUIRE(is_nan(nan_val));
    REQUIRE_FALSE(is_finite(nan_val));
    
    float inf_val = infinity<float>();
    REQUIRE(is_inf(inf_val));
    REQUIRE_FALSE(is_finite(inf_val));
}

TEST_CASE("Float Comparison: Absolute equality", "[utilities][float_comparison]") {
    REQUIRE(equals_absolute(1.0f, 1.0f + 1e-7f, 1e-6f));
    REQUIRE_FALSE(equals_absolute(1.0f, 1.0f + 1e-5f, 1e-6f));
}

TEST_CASE("Float Comparison: Relative equality", "[utilities][float_comparison]") {
    // Relative equality: diff <= tolerance * max_magnitude
    // Example: 1000.01 is 1e-5 away from 1000, which is 1e-5 relative
    //          1e-5 <= 1e-4 * 1000 = 0.1? YES ✓
    REQUIRE(equals_relative(1000.0f, 1000.01f, 1e-4f));
    // Example: 1.0001 is 1e-4 away from 1.0, which is 1e-4 relative
    //          1e-4 <= 1e-3 * 1.0 = 1e-3? YES ✓
    REQUIRE(equals_relative(1.0f, 1.0001f, 1e-3f));
}

TEST_CASE("Float Comparison: Combined equality", "[utilities][float_comparison]") {
    REQUIRE(equals(1.0f, 1.0f));
    REQUIRE(equals(0.0f, 1e-7f, 1e-6f));
    REQUIRE(equals(1e6f, 1e6f + 1.0f, 1e-7f, 1e-5f));
}

TEST_CASE("Float Comparison: Zero check", "[utilities][float_comparison]") {
    REQUIRE(is_zero(0.0f));
    REQUIRE(is_zero(1e-7f, 1e-6f));
    REQUIRE_FALSE(is_zero(1e-5f, 1e-6f));
}

// ================================================================
// Geometry Tests
// ================================================================

TEST_CASE("Geometry: Sphere point containment", "[utilities][geometry]") {
    Sphere<float> sphere(0.0f, 0.0f, 0.0f, 1.0f);
    
    REQUIRE(point_in_sphere(Vec3<float>(0.0f, 0.0f, 0.0f), sphere));
    REQUIRE(point_in_sphere(Vec3<float>(0.5f, 0.0f, 0.0f), sphere));
    REQUIRE(point_in_sphere(Vec3<float>(1.0f, 0.0f, 0.0f), sphere));
    REQUIRE_FALSE(point_in_sphere(Vec3<float>(1.5f, 0.0f, 0.0f), sphere));
}

TEST_CASE("Geometry: Sphere-sphere intersection", "[utilities][geometry]") {
    Sphere<float> s1(0.0f, 0.0f, 0.0f, 1.0f);
    Sphere<float> s2(1.5f, 0.0f, 0.0f, 1.0f);
    Sphere<float> s3(3.0f, 0.0f, 0.0f, 1.0f);
    
    REQUIRE(sphere_sphere_intersection(s1, s2));  // Intersect
    REQUIRE_FALSE(sphere_sphere_intersection(s1, s3));  // Too far
}

TEST_CASE("Geometry: AABB point containment", "[utilities][geometry]") {
    AABB<float> aabb(Vec3<float>(0.0f, 0.0f, 0.0f), Vec3<float>(1.0f, 1.0f, 1.0f));
    
    REQUIRE(point_in_aabb(Vec3<float>(0.5f, 0.5f, 0.5f), aabb));
    REQUIRE(point_in_aabb(Vec3<float>(0.0f, 0.0f, 0.0f), aabb));
    REQUIRE_FALSE(point_in_aabb(Vec3<float>(1.5f, 0.5f, 0.5f), aabb));
}

TEST_CASE("Geometry: AABB-AABB intersection", "[utilities][geometry]") {
    AABB<float> aabb1(Vec3<float>(0.0f, 0.0f, 0.0f), Vec3<float>(1.0f, 1.0f, 1.0f));
    AABB<float> aabb2(Vec3<float>(0.5f, 0.5f, 0.5f), Vec3<float>(1.5f, 1.5f, 1.5f));
    AABB<float> aabb3(Vec3<float>(2.0f, 2.0f, 2.0f), Vec3<float>(3.0f, 3.0f, 3.0f));
    
    REQUIRE(aabb_aabb_intersection(aabb1, aabb2));
    REQUIRE_FALSE(aabb_aabb_intersection(aabb1, aabb3));
}

TEST_CASE("Geometry: Ray-plane intersection", "[utilities][geometry]") {
    // Plane at z=1, facing +z
    Plane<float> plane(0.0f, 0.0f, 1.0f, -1.0f);
    
    // Ray from origin going +z
    Vec3<float> ray_origin(0.0f, 0.0f, 0.0f);
    Vec3<float> ray_direction(0.0f, 0.0f, 1.0f);
    float t = 0.0f;
    
    REQUIRE(ray_plane_intersection(ray_origin, ray_direction, plane, t));
    REQUIRE_THAT(t, WithinAbs(1.0f, 1e-6f));
}

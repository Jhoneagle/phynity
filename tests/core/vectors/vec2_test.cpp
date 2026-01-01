#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec2.hpp>
#include <cmath>

using phynity::math::vectors::Vec2;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Constructors and Basic Properties
// ============================================================================

TEST_CASE("Vec2: Constructors", "[Vec2][constructor]") {
    SECTION("Default constructor initializes to zero") {
        Vec2 v;
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Scalar constructor") {
        Vec2 v(5.0f);
        REQUIRE_THAT(v.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Component constructor") {
        Vec2 v(3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(4.0f, 1e-6f));
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Vec2: Addition", "[Vec2][arithmetic]") {
    Vec2 a(1.0f, 2.0f);
    Vec2 b(3.0f, 4.0f);
    Vec2 c = a + b;

    REQUIRE_THAT(c.x, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.y, WithinAbs(6.0f, 1e-6f));
}

TEST_CASE("Vec2: Subtraction", "[Vec2][arithmetic]") {
    Vec2 a(5.0f, 7.0f);
    Vec2 b(2.0f, 3.0f);
    Vec2 c = a - b;

    REQUIRE_THAT(c.x, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(c.y, WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Vec2: Scalar multiplication", "[Vec2][arithmetic]") {
    Vec2 v(2.0f, 3.0f);
    
    SECTION("Vector * scalar") {
        Vec2 result = v * 2.0f;
        REQUIRE_THAT(result.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Scalar * vector") {
        Vec2 result = 2.0f * v;
        REQUIRE_THAT(result.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(6.0f, 1e-6f));
    }
}

TEST_CASE("Vec2: Scalar division", "[Vec2][arithmetic]") {
    Vec2 v(4.0f, 6.0f);
    Vec2 result = v / 2.0f;

    REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("Vec2: Component-wise multiplication", "[Vec2][arithmetic]") {
    Vec2 a(2.0f, 3.0f);
    Vec2 b(4.0f, 5.0f);
    Vec2 result = a * b;

    REQUIRE_THAT(result.x, WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(15.0f, 1e-6f));
}

TEST_CASE("Vec2: Component-wise division", "[Vec2][arithmetic]") {
    Vec2 a(8.0f, 15.0f);
    Vec2 b(2.0f, 3.0f);
    Vec2 result = a / b;

    REQUIRE_THAT(result.x, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("Vec2: Negation", "[Vec2][arithmetic]") {
    Vec2 v(3.0f, -4.0f);
    Vec2 result = -v;

    REQUIRE_THAT(result.x, WithinAbs(-3.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Vec2: Assignment operators", "[Vec2][assignment]") {
    Vec2 v(1.0f, 2.0f);

    SECTION("Addition assignment") {
        v += Vec2(3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Subtraction assignment") {
        v -= Vec2(1.0f, 1.0f);
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Scalar multiplication assignment") {
        v *= 2.0f;
        REQUIRE_THAT(v.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Scalar division assignment") {
        v /= 2.0f;
        REQUIRE_THAT(v.x, WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Vec2: Comparison operators", "[Vec2][comparison]") {
    Vec2 a(1.0f, 2.0f);
    Vec2 b(1.0f, 2.0f);
    Vec2 c(2.0f, 3.0f);

    REQUIRE(a == b);
    REQUIRE(a != c);
    REQUIRE(!(a == c));
    REQUIRE(!(a != b));
}

// ============================================================================
// Index Access
// ============================================================================

TEST_CASE("Vec2: Index access", "[Vec2][access]") {
    Vec2 v(5.0f, 10.0f);

    SECTION("Read access") {
        REQUIRE_THAT(v[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v[1], WithinAbs(10.0f, 1e-6f));
    }

    SECTION("Write access") {
        v[0] = 20.0f;
        v[1] = 30.0f;
        REQUIRE_THAT(v.x, WithinAbs(20.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(30.0f, 1e-6f));
    }
}

// ============================================================================
// Vector Operations: Magnitude and Normalization
// ============================================================================

TEST_CASE("Vec2: Length operations", "[Vec2][magnitude]") {
    SECTION("3-4-5 triangle") {
        Vec2 v(3.0f, 4.0f);
        REQUIRE_THAT(v.squaredLength(), WithinAbs(25.0f, 1e-6f));
        REQUIRE_THAT(v.length(), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Zero vector") {
        Vec2 v(0.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.squaredLength(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Unit vector") {
        Vec2 v(1.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-6f));
    }
}

TEST_CASE("Vec2: Normalization", "[Vec2][normalization]") {
    SECTION("Normal vector becomes unit") {
        Vec2 v(3.0f, 4.0f);
        Vec2 n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Direction is preserved") {
        Vec2 v(2.0f, 0.0f);
        Vec2 n = v.normalized();
        REQUIRE_THAT(n.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(n.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Zero vector normalization") {
        Vec2 v(0.0f, 0.0f);
        Vec2 n = v.normalized();
        REQUIRE_THAT(n.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(n.y, WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Dot Product
// ============================================================================

TEST_CASE("Vec2: Dot product", "[Vec2][dot]") {
    SECTION("Orthogonal vectors") {
        Vec2 a(1.0f, 0.0f);
        Vec2 b(0.0f, 1.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Parallel vectors") {
        Vec2 a(2.0f, 3.0f);
        Vec2 b(4.0f, 6.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(26.0f, 1e-6f));
    }

    SECTION("Same vector (squared length)") {
        Vec2 v(3.0f, 4.0f);
        REQUIRE_THAT(v.dot(v), WithinAbs(25.0f, 1e-6f));
    }
}

// ============================================================================
// Distance
// ============================================================================

TEST_CASE("Vec2: Distance", "[Vec2][distance]") {
    SECTION("3-4-5 triangle") {
        Vec2 a(0.0f, 0.0f);
        Vec2 b(3.0f, 4.0f);
        REQUIRE_THAT(a.distance(b), WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(b.distance(a), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Same point") {
        Vec2 a(5.0f, 5.0f);
        REQUIRE_THAT(a.distance(a), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Squared distance") {
        Vec2 a(0.0f, 0.0f);
        Vec2 b(3.0f, 4.0f);
        REQUIRE_THAT(a.squaredDistance(b), WithinAbs(25.0f, 1e-6f));
    }
}

// ============================================================================
// Angle Between Vectors
// ============================================================================

TEST_CASE("Vec2: Angle between vectors", "[Vec2][angle]") {
    SECTION("Same direction (angle = 0)") {
        Vec2 a(1.0f, 0.0f);
        Vec2 b(2.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Perpendicular (angle = π/2)") {
        Vec2 a(1.0f, 0.0f);
        Vec2 b(0.0f, 1.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f / 2.0f, 1e-4f));
    }

    SECTION("Opposite direction (angle = π)") {
        Vec2 a(1.0f, 0.0f);
        Vec2 b(-1.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f, 1e-4f));
    }

    SECTION("Zero vector handling") {
        Vec2 a(0.0f, 0.0f);
        Vec2 b(1.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Clamping
// ============================================================================

TEST_CASE("Vec2: Clamping", "[Vec2][clamp]") {
    SECTION("Vector within limit") {
        Vec2 v(2.0f, 3.0f);
        Vec2 clamped = v.clamped(10.0f);
        REQUIRE_THAT(clamped.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(clamped.y, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Vector exceeds limit") {
        Vec2 v(3.0f, 4.0f);  // length = 5
        Vec2 clamped = v.clamped(2.0f);
        REQUIRE_THAT(clamped.length(), WithinAbs(2.0f, 1e-5f));
        // Direction should be preserved
        REQUIRE_THAT(clamped[0] / v[0], WithinAbs(clamped[1] / v[1], 1e-5f));
    }

    SECTION("In-place clamping") {
        Vec2 v(3.0f, 4.0f);
        v.clamp(2.0f);
        REQUIRE_THAT(v.length(), WithinAbs(2.0f, 1e-5f));
    }
}

// ============================================================================
// Linear Interpolation
// ============================================================================

TEST_CASE("Vec2: Linear interpolation (lerp)", "[Vec2][lerp]") {
    Vec2 a(0.0f, 0.0f);
    Vec2 b(10.0f, 20.0f);

    SECTION("t = 0") {
        Vec2 result = a.lerp(b, 0.0f);
        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("t = 1") {
        Vec2 result = a.lerp(b, 1.0f);
        REQUIRE_THAT(result.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("t = 0.5 (midpoint)") {
        Vec2 result = a.lerp(b, 0.5f);
        REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(10.0f, 1e-6f));
    }
}

// ============================================================================
// Projection
// ============================================================================

TEST_CASE("Vec2: Projection", "[Vec2][project]") {
    SECTION("Project onto parallel vector") {
        Vec2 v(4.0f, 0.0f);
        Vec2 onto(2.0f, 0.0f);
        Vec2 proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(proj.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Project onto perpendicular vector") {
        Vec2 v(1.0f, 0.0f);
        Vec2 onto(0.0f, 2.0f);
        Vec2 proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(proj.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Project onto zero vector") {
        Vec2 v(1.0f, 2.0f);
        Vec2 onto(0.0f, 0.0f);
        Vec2 proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(proj.y, WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Reflection
// ============================================================================

TEST_CASE("Vec2: Reflection", "[Vec2][reflect]") {
    SECTION("Reflect off horizontal") {
        Vec2 v(1.0f, 1.0f);
        Vec2 normal(0.0f, 1.0f);  // Pointing up
        Vec2 reflected = v.reflect(normal);
        REQUIRE_THAT(reflected.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(reflected.y, WithinAbs(-1.0f, 1e-6f));
    }

    SECTION("Reflect off vertical") {
        Vec2 v(1.0f, 1.0f);
        Vec2 normal(1.0f, 0.0f);  // Pointing right
        Vec2 reflected = v.reflect(normal);
        REQUIRE_THAT(reflected.x, WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(reflected.y, WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Perpendicular (2D rotation)
// ============================================================================

TEST_CASE("Vec2: Perpendicular", "[Vec2][perpendicular]") {
    SECTION("Right vector becomes up") {
        Vec2 v(1.0f, 0.0f);
        Vec2 perp = v.perpendicular();
        REQUIRE_THAT(perp.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(perp.y, WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Up vector becomes left") {
        Vec2 v(0.0f, 1.0f);
        Vec2 perp = v.perpendicular();
        REQUIRE_THAT(perp.x, WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(perp.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Perpendicular is perpendicular (dot = 0)") {
        Vec2 v(3.0f, 4.0f);
        Vec2 perp = v.perpendicular();
        REQUIRE_THAT(v.dot(perp), WithinAbs(0.0f, 1e-4f));
    }
}

// ============================================================================
// Min/Max
// ============================================================================

TEST_CASE("Vec2: Min/Max component-wise", "[Vec2][minmax]") {
    Vec2 a(2.0f, 8.0f);
    Vec2 b(5.0f, 3.0f);

    SECTION("Min") {
        Vec2 result = a.min(b);
        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Max") {
        Vec2 result = a.max(b);
        REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(8.0f, 1e-6f));
    }
}

// ============================================================================
// Queries
// ============================================================================

TEST_CASE("Vec2: Query functions", "[Vec2][query]") {
    SECTION("isZero") {
        REQUIRE(Vec2(0.0f, 0.0f).isZero());
        REQUIRE(!Vec2(0.001f, 0.0f).isZero());
    }

    SECTION("isNormalized") {
        REQUIRE(Vec2(1.0f, 0.0f).isNormalized());
        REQUIRE(Vec2(0.0f, 1.0f).isNormalized());
        REQUIRE(!Vec2(2.0f, 0.0f).isNormalized());
    }
}

// ============================================================================
// Static Utility Vectors
// ============================================================================

TEST_CASE("Vec2: Static utility vectors", "[Vec2][static]") {
    REQUIRE(Vec2::zero() == Vec2(0.0f, 0.0f));
    REQUIRE(Vec2::one() == Vec2(1.0f, 1.0f));
    REQUIRE(Vec2::up() == Vec2(0.0f, 1.0f));
    REQUIRE(Vec2::down() == Vec2(0.0f, -1.0f));
    REQUIRE(Vec2::right() == Vec2(1.0f, 0.0f));
    REQUIRE(Vec2::left() == Vec2(-1.0f, 0.0f));
}

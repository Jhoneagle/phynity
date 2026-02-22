#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec2.hpp>
#include <core/math/utilities/constants.hpp>
#include <cmath>
#include <sstream>

using phynity::math::vectors::Vec2f;
using phynity::math::utilities::mathf;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Constructors and Basic Properties
// ============================================================================

TEST_CASE("Vec2f: Constructors", "[Vec2f][constructor]") {
    SECTION("Default constructor initializes to zero") {
        Vec2f v;
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Scalar constructor") {
        Vec2f v(5.0f);
        REQUIRE_THAT(v.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Component constructor") {
        Vec2f v(3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(4.0f, 1e-6f));
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Vec2f: Addition", "[Vec2f][arithmetic]") {
    Vec2f a(1.0f, 2.0f);
    Vec2f b(3.0f, 4.0f);
    Vec2f c = a + b;

    REQUIRE_THAT(c.x, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.y, WithinAbs(6.0f, 1e-6f));
}

TEST_CASE("Vec2f: Subtraction", "[Vec2f][arithmetic]") {
    Vec2f a(5.0f, 7.0f);
    Vec2f b(2.0f, 3.0f);
    Vec2f c = a - b;

    REQUIRE_THAT(c.x, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(c.y, WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("Vec2f: Scalar multiplication", "[Vec2f][arithmetic]") {
    Vec2f v(2.0f, 3.0f);
    
    SECTION("Vector * scalar") {
        Vec2f result = v * 2.0f;
        REQUIRE_THAT(result.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Scalar * vector") {
        Vec2f result = 2.0f * v;
        REQUIRE_THAT(result.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(6.0f, 1e-6f));
    }
}

TEST_CASE("Vec2f: Scalar division", "[Vec2f][arithmetic]") {
    Vec2f v(4.0f, 6.0f);
    Vec2f result = v / 2.0f;

    REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("Vec2f: Component-wise multiplication", "[Vec2f][arithmetic]") {
    Vec2f a(2.0f, 3.0f);
    Vec2f b(4.0f, 5.0f);
    Vec2f result = a * b;

    REQUIRE_THAT(result.x, WithinAbs(8.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(15.0f, 1e-6f));
}

TEST_CASE("Vec2f: Component-wise division", "[Vec2f][arithmetic]") {
    Vec2f a(8.0f, 15.0f);
    Vec2f b(2.0f, 3.0f);
    Vec2f result = a / b;

    REQUIRE_THAT(result.x, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("Vec2f: Negation", "[Vec2f][arithmetic]") {
    Vec2f v(3.0f, -4.0f);
    Vec2f result = -v;

    REQUIRE_THAT(result.x, WithinAbs(-3.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Vec2f: Assignment operators", "[Vec2f][assignment]") {
    Vec2f v(1.0f, 2.0f);

    SECTION("Addition assignment") {
        v += Vec2f(3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Subtraction assignment") {
        v -= Vec2f(1.0f, 1.0f);
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

    SECTION("Component-wise multiplication assignment") {
        v *= Vec2f(2.0f, 3.0f);
        REQUIRE_THAT(v.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Component-wise division assignment") {
        v /= Vec2f(2.0f, 2.0f);
        REQUIRE_THAT(v.x, WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Vec2f: Comparison operators", "[Vec2f][comparison]") {
    Vec2f a(1.0f, 2.0f);
    Vec2f b(1.0f, 2.0f);
    Vec2f c(2.0f, 3.0f);

    REQUIRE(a == b);
    REQUIRE(a != c);
    REQUIRE(!(a == c));
    REQUIRE(!(a != b));
}

// ============================================================================
// Index Access
// ============================================================================

TEST_CASE("Vec2f: Index access", "[Vec2f][access]") {
    Vec2f v(5.0f, 10.0f);

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

TEST_CASE("Vec2f: Length operations", "[Vec2f][magnitude]") {
    SECTION("3-4-5 triangle") {
        Vec2f v(3.0f, 4.0f);
        REQUIRE_THAT(v.squaredLength(), WithinAbs(25.0f, 1e-6f));
        REQUIRE_THAT(v.length(), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Zero vector") {
        Vec2f v(0.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.squaredLength(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Unit vector") {
        Vec2f v(1.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-6f));
    }
}

TEST_CASE("Vec2f: Normalization", "[Vec2f][normalization]") {
    SECTION("Normal vector becomes unit") {
        Vec2f v(3.0f, 4.0f);
        Vec2f n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Direction is preserved") {
        Vec2f v(2.0f, 0.0f);
        Vec2f n = v.normalized();
        REQUIRE_THAT(n.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(n.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Zero vector normalization") {
        Vec2f v(0.0f, 0.0f);
        Vec2f n = v.normalized();
        REQUIRE_THAT(n.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(n.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("In-place normalize") {
        Vec2f v(3.0f, 4.0f);
        v.normalize();
        REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-5f));
    }
}

// ============================================================================
// Dot Product
// ============================================================================

TEST_CASE("Vec2f: Dot product", "[Vec2f][dot]") {
    SECTION("Orthogonal vectors") {
        Vec2f a(1.0f, 0.0f);
        Vec2f b(0.0f, 1.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Parallel vectors") {
        Vec2f a(2.0f, 3.0f);
        Vec2f b(4.0f, 6.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(26.0f, 1e-6f));
    }

    SECTION("Same vector (squared length)") {
        Vec2f v(3.0f, 4.0f);
        REQUIRE_THAT(v.dot(v), WithinAbs(25.0f, 1e-6f));
    }
}

// ============================================================================
// Distance
// ============================================================================

TEST_CASE("Vec2f: Distance", "[Vec2f][distance]") {
    SECTION("3-4-5 triangle") {
        Vec2f a(0.0f, 0.0f);
        Vec2f b(3.0f, 4.0f);
        REQUIRE_THAT(a.distance(b), WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(b.distance(a), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Same point") {
        Vec2f a(5.0f, 5.0f);
        REQUIRE_THAT(a.distance(a), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Squared distance") {
        Vec2f a(0.0f, 0.0f);
        Vec2f b(3.0f, 4.0f);
        REQUIRE_THAT(a.squaredDistance(b), WithinAbs(25.0f, 1e-6f));
    }
}

// ============================================================================
// Angle Between Vectors
// ============================================================================

TEST_CASE("Vec2f: Angle between vectors", "[Vec2f][angle]") {
    SECTION("Same direction (angle = 0)") {
        Vec2f a(1.0f, 0.0f);
        Vec2f b(2.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Perpendicular (angle = π/2)") {
        Vec2f a(1.0f, 0.0f);
        Vec2f b(0.0f, 1.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(mathf::half_pi, 1e-4f));
    }

    SECTION("Opposite direction (angle = π)") {
        Vec2f a(1.0f, 0.0f);
        Vec2f b(-1.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(mathf::pi, 1e-4f));
    }

    SECTION("Zero vector handling") {
        Vec2f a(0.0f, 0.0f);
        Vec2f b(1.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Clamping
// ============================================================================

TEST_CASE("Vec2f: Clamping", "[Vec2f][clamp]") {
    SECTION("Vector within limit") {
        Vec2f v(2.0f, 3.0f);
        Vec2f clamped = v.clamped(10.0f);
        REQUIRE_THAT(clamped.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(clamped.y, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Vector exceeds limit") {
        Vec2f v(3.0f, 4.0f);  // length = 5
        Vec2f clamped = v.clamped(2.0f);
        REQUIRE_THAT(clamped.length(), WithinAbs(2.0f, 1e-5f));
        // Direction should be preserved
        REQUIRE_THAT(clamped[0] / v[0], WithinAbs(clamped[1] / v[1], 1e-5f));
    }

    SECTION("In-place clamping") {
        Vec2f v(3.0f, 4.0f);
        v.clamp(2.0f);
        REQUIRE_THAT(v.length(), WithinAbs(2.0f, 1e-5f));
    }
}

// ============================================================================
// Linear Interpolation
// ============================================================================

TEST_CASE("Vec2f: Linear interpolation (lerp)", "[Vec2f][lerp]") {
    Vec2f a(0.0f, 0.0f);
    Vec2f b(10.0f, 20.0f);

    SECTION("t = 0") {
        Vec2f result = a.lerp(b, 0.0f);
        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("t = 1") {
        Vec2f result = a.lerp(b, 1.0f);
        REQUIRE_THAT(result.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("t = 0.5 (midpoint)") {
        Vec2f result = a.lerp(b, 0.5f);
        REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(10.0f, 1e-6f));
    }
}

// ============================================================================
// Projection
// ============================================================================

TEST_CASE("Vec2f: Projection", "[Vec2f][project]") {
    SECTION("Project onto parallel vector") {
        Vec2f v(4.0f, 0.0f);
        Vec2f onto(2.0f, 0.0f);
        Vec2f proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(proj.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Project onto perpendicular vector") {
        Vec2f v(1.0f, 0.0f);
        Vec2f onto(0.0f, 2.0f);
        Vec2f proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(proj.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Project onto zero vector") {
        Vec2f v(1.0f, 2.0f);
        Vec2f onto(0.0f, 0.0f);
        Vec2f proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(proj.y, WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Reflection
// ============================================================================

TEST_CASE("Vec2f: Reflection", "[Vec2f][reflect]") {
    SECTION("Reflect off horizontal") {
        Vec2f v(1.0f, 1.0f);
        Vec2f normal(0.0f, 1.0f);  // Pointing up
        Vec2f reflected = v.reflect(normal);
        REQUIRE_THAT(reflected.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(reflected.y, WithinAbs(-1.0f, 1e-6f));
    }

    SECTION("Reflect off vertical") {
        Vec2f v(1.0f, 1.0f);
        Vec2f normal(1.0f, 0.0f);  // Pointing right
        Vec2f reflected = v.reflect(normal);
        REQUIRE_THAT(reflected.x, WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(reflected.y, WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Perpendicular (2D rotation)
// ============================================================================

TEST_CASE("Vec2f: Perpendicular", "[Vec2f][perpendicular]") {
    SECTION("Right vector becomes up") {
        Vec2f v(1.0f, 0.0f);
        Vec2f perp = v.perpendicular();
        REQUIRE_THAT(perp.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(perp.y, WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Up vector becomes left") {
        Vec2f v(0.0f, 1.0f);
        Vec2f perp = v.perpendicular();
        REQUIRE_THAT(perp.x, WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(perp.y, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Perpendicular is perpendicular (dot = 0)") {
        Vec2f v(3.0f, 4.0f);
        Vec2f perp = v.perpendicular();
        REQUIRE_THAT(v.dot(perp), WithinAbs(0.0f, 1e-4f));
    }
}

// ============================================================================
// Min/Max
// ============================================================================

TEST_CASE("Vec2f: Min/Max component-wise", "[Vec2f][minmax]") {
    Vec2f a(2.0f, 8.0f);
    Vec2f b(5.0f, 3.0f);

    SECTION("Min") {
        Vec2f result = a.min(b);
        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Max") {
        Vec2f result = a.max(b);
        REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(8.0f, 1e-6f));
    }
}

// ============================================================================
// Queries
// ============================================================================

TEST_CASE("Vec2f: Query functions", "[Vec2f][query]") {
    SECTION("isZero") {
        REQUIRE(Vec2f(0.0f, 0.0f).isZero());
        REQUIRE(!Vec2f(0.001f, 0.0f).isZero());
    }

    SECTION("isNormalized") {
        REQUIRE(Vec2f(1.0f, 0.0f).isNormalized());
        REQUIRE(Vec2f(0.0f, 1.0f).isNormalized());
        REQUIRE(!Vec2f(2.0f, 0.0f).isNormalized());
    }

    SECTION("approxEqual") {
        Vec2f a(1.0f, 2.0f);
        Vec2f b(1.00001f, 2.00001f);
        Vec2f c(1.1f, 2.0f);
        REQUIRE(a.approxEqual(b, 1e-4f));
        REQUIRE(!a.approxEqual(c, 1e-4f));
    }

    SECTION("abs") {
        Vec2f v(-3.0f, 4.0f);
        Vec2f result = v.abs();
        REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
    }
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("Vec2f: Edge cases", "[Vec2f][edge]") {
    SECTION("Division by zero scalar") {
        Vec2f v(1.0f, 2.0f);
        Vec2f result = v / 0.0f;
        // Result should be infinity or NaN, just verify it doesn't crash
        REQUIRE((std::isinf(result.x) || std::isnan(result.x)));
    }

    SECTION("Normalization of very small vector") {
        Vec2f v(1e-20f, 1e-20f);
        Vec2f n = v.normalized();
        // Normalized vector should still have length 1 (or handle underflow gracefully)
        // With very small inputs, normalized() still produces a unit vector
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Operations on very large numbers") {
        Vec2f v(1e20f, 1e20f);
        float len = v.length();
        REQUIRE(len > 1e20f);
    }

    SECTION("Component-wise division by zero vector") {
        Vec2f a(1.0f, 2.0f);
        Vec2f b(0.0f, 0.0f);
        Vec2f result = a / b;
        REQUIRE((std::isinf(result.x) || std::isnan(result.x)));
    }
}

// ============================================================================
// Static Utility Vectors
// ============================================================================

TEST_CASE("Vec2f: Static utility vectors", "[Vec2f][static]") {
    REQUIRE(Vec2f::zero() == Vec2f(0.0f, 0.0f));
    REQUIRE(Vec2f::one() == Vec2f(1.0f, 1.0f));
    REQUIRE(Vec2f::up() == Vec2f(0.0f, 1.0f));
    REQUIRE(Vec2f::down() == Vec2f(0.0f, -1.0f));
    REQUIRE(Vec2f::right() == Vec2f(1.0f, 0.0f));
    REQUIRE(Vec2f::left() == Vec2f(-1.0f, 0.0f));
}

TEST_CASE("Vec2f: Stream output", "[Vec2f][stream]") {
    Vec2f v(1.5f, 2.5f);
    std::ostringstream oss;
    oss << v;
    REQUIRE(oss.str() == "(1.5, 2.5)");
}

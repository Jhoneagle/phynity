#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/math/utilities/constants.hpp>
#include <cmath>
#include <sstream>

using phynity::math::vectors::Vec3f;
using phynity::math::utilities::mathf;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Constructors and Basic Properties
// ============================================================================

TEST_CASE("Vec3f: Constructors", "[Vec3f][constructor]") {
    SECTION("Default constructor initializes to zero") {
        Vec3f v;
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Scalar constructor") {
        Vec3f v(5.0f);
        REQUIRE_THAT(v.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Component constructor") {
        Vec3f v(1.0f, 2.0f, 3.0f);
        REQUIRE_THAT(v.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(3.0f, 1e-6f));
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Vec3f: Addition", "[Vec3f][arithmetic]") {
    Vec3f a(1.0f, 2.0f, 3.0f);
    Vec3f b(4.0f, 5.0f, 6.0f);
    Vec3f c = a + b;

    REQUIRE_THAT(c.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(c.y, WithinAbs(7.0f, 1e-6f));
    REQUIRE_THAT(c.z, WithinAbs(9.0f, 1e-6f));
}

TEST_CASE("Vec3f: Subtraction", "[Vec3f][arithmetic]") {
    Vec3f a(5.0f, 7.0f, 9.0f);
    Vec3f b(2.0f, 3.0f, 4.0f);
    Vec3f c = a - b;

    REQUIRE_THAT(c.x, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(c.y, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.z, WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("Vec3f: Scalar multiplication", "[Vec3f][arithmetic]") {
    Vec3f v(1.0f, 2.0f, 3.0f);
    
    SECTION("Vector * scalar") {
        Vec3f result = v * 2.0f;
        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Scalar * vector") {
        Vec3f result = 2.0f * v;
        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(6.0f, 1e-6f));
    }
}

TEST_CASE("Vec3f: Scalar division", "[Vec3f][arithmetic]") {
    Vec3f v(2.0f, 4.0f, 6.0f);
    Vec3f result = v / 2.0f;

    REQUIRE_THAT(result.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("Vec3f: Component-wise multiplication", "[Vec3f][arithmetic]") {
    Vec3f a(2.0f, 3.0f, 4.0f);
    Vec3f b(5.0f, 6.0f, 7.0f);
    Vec3f result = a * b;

    REQUIRE_THAT(result.x, WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(18.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(28.0f, 1e-6f));
}

TEST_CASE("Vec3f: Component-wise division", "[Vec3f][arithmetic]") {
    Vec3f a(10.0f, 18.0f, 28.0f);
    Vec3f b(2.0f, 3.0f, 4.0f);
    Vec3f result = a / b;

    REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(7.0f, 1e-6f));
}

TEST_CASE("Vec3f: Negation", "[Vec3f][arithmetic]") {
    Vec3f v(1.0f, -2.0f, 3.0f);
    Vec3f result = -v;

    REQUIRE_THAT(result.x, WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(-3.0f, 1e-6f));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Vec3f: Assignment operators", "[Vec3f][assignment]") {
    Vec3f v(1.0f, 2.0f, 3.0f);

    SECTION("Addition assignment") {
        v += Vec3f(4.0f, 5.0f, 6.0f);
        REQUIRE_THAT(v.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(7.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Subtraction assignment") {
        v -= Vec3f(1.0f, 1.0f, 1.0f);
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Scalar multiplication assignment") {
        v *= 2.0f;
        REQUIRE_THAT(v.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Scalar division assignment") {
        v /= 2.0f;
        REQUIRE_THAT(v.x, WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(1.5f, 1e-6f));
    }

    SECTION("Component-wise multiplication assignment") {
        v *= Vec3f(2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(12.0f, 1e-6f));
    }

    SECTION("Component-wise division assignment") {
        v /= Vec3f(2.0f, 2.0f, 3.0f);
        REQUIRE_THAT(v.x, WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(1.0f, 1e-6f));
    }
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Vec3f: Comparison operators", "[Vec3f][comparison]") {
    Vec3f a(1.0f, 2.0f, 3.0f);
    Vec3f b(1.0f, 2.0f, 3.0f);
    Vec3f c(2.0f, 3.0f, 4.0f);

    REQUIRE(a == b);
    REQUIRE(a != c);
    REQUIRE(!(a == c));
    REQUIRE(!(a != b));
}

// ============================================================================
// Index Access
// ============================================================================

TEST_CASE("Vec3f: Index access", "[Vec3f][access]") {
    Vec3f v(5.0f, 10.0f, 15.0f);

    SECTION("Read access") {
        REQUIRE_THAT(v[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v[1], WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(v[2], WithinAbs(15.0f, 1e-6f));
    }

    SECTION("Write access") {
        v[0] = 20.0f;
        v[1] = 30.0f;
        v[2] = 40.0f;
        REQUIRE_THAT(v.x, WithinAbs(20.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(30.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(40.0f, 1e-6f));
    }
}

// ============================================================================
// Vector Operations: Magnitude and Normalization
// ============================================================================

TEST_CASE("Vec3f: Length operations", "[Vec3f][magnitude]") {
    SECTION("1-2-2 triangle") {
        Vec3f v(2.0f, 3.0f, 6.0f);
        REQUIRE_THAT(v.squaredLength(), WithinAbs(49.0f, 1e-6f));
        REQUIRE_THAT(v.length(), WithinAbs(7.0f, 1e-6f));
    }

    SECTION("Zero vector") {
        Vec3f v(0.0f, 0.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.squaredLength(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Unit vector") {
        Vec3f v(1.0f, 0.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-6f));
    }
}

TEST_CASE("Vec3f: Normalization", "[Vec3f][normalization]") {
    SECTION("Normal vector becomes unit") {
        Vec3f v(2.0f, 3.0f, 6.0f);
        Vec3f n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Direction is preserved") {
        Vec3f v(1.0f, 0.0f, 0.0f);
        Vec3f n = v.normalized();
        REQUIRE_THAT(n.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(n.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(n.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Zero vector normalization") {
        Vec3f v(0.0f, 0.0f, 0.0f);
        Vec3f n = v.normalized();
        REQUIRE_THAT(n.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(n.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(n.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("In-place normalize") {
        Vec3f v(2.0f, 3.0f, 6.0f);
        v.normalize();
        REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-5f));
    }
}

// ============================================================================
// Dot Product
// ============================================================================

TEST_CASE("Vec3f: Dot product", "[Vec3f][dot]") {
    SECTION("Orthogonal vectors") {
        Vec3f a(1.0f, 0.0f, 0.0f);
        Vec3f b(0.0f, 1.0f, 0.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Parallel vectors") {
        Vec3f a(1.0f, 2.0f, 3.0f);
        Vec3f b(2.0f, 4.0f, 6.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(28.0f, 1e-6f));
    }

    SECTION("Same vector (squared length)") {
        Vec3f v(1.0f, 2.0f, 3.0f);
        REQUIRE_THAT(v.dot(v), WithinAbs(14.0f, 1e-6f));
    }
}

// ============================================================================
// Cross Product
// ============================================================================

TEST_CASE("Vec3f: Cross product", "[Vec3f][cross]") {
    SECTION("Right hand rule: X × Y = Z") {
        Vec3f x(1.0f, 0.0f, 0.0f);
        Vec3f y(0.0f, 1.0f, 0.0f);
        Vec3f z = x.cross(y);
        REQUIRE_THAT(z.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(z.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(z.z, WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Anti-commutative: A × B = -(B × A)") {
        Vec3f a(1.0f, 2.0f, 3.0f);
        Vec3f b(4.0f, 5.0f, 6.0f);
        Vec3f cross_ab = a.cross(b);
        Vec3f cross_ba = b.cross(a);
        REQUIRE_THAT(cross_ab.x, WithinAbs(-cross_ba.x, 1e-6f));
        REQUIRE_THAT(cross_ab.y, WithinAbs(-cross_ba.y, 1e-6f));
        REQUIRE_THAT(cross_ab.z, WithinAbs(-cross_ba.z, 1e-6f));
    }

    SECTION("Cross product is perpendicular") {
        Vec3f a(1.0f, 0.0f, 0.0f);
        Vec3f b(0.0f, 1.0f, 0.0f);
        Vec3f cross = a.cross(b);
        REQUIRE_THAT(a.dot(cross), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(b.dot(cross), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Parallel vectors have zero cross product") {
        Vec3f a(1.0f, 2.0f, 3.0f);
        Vec3f b(2.0f, 4.0f, 6.0f);
        Vec3f cross = a.cross(b);
        REQUIRE_THAT(cross.length(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Distance
// ============================================================================

TEST_CASE("Vec3f: Distance", "[Vec3f][distance]") {
    SECTION("Simple distance") {
        Vec3f a(0.0f, 0.0f, 0.0f);
        Vec3f b(3.0f, 4.0f, 0.0f);
        REQUIRE_THAT(a.distance(b), WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(b.distance(a), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Same point") {
        Vec3f a(5.0f, 5.0f, 5.0f);
        REQUIRE_THAT(a.distance(a), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Squared distance") {
        Vec3f a(0.0f, 0.0f, 0.0f);
        Vec3f b(2.0f, 3.0f, 6.0f);
        REQUIRE_THAT(a.squaredDistance(b), WithinAbs(49.0f, 1e-6f));
    }
}

// ============================================================================
// Angle Between Vectors
// ============================================================================

TEST_CASE("Vec3f: Angle between vectors", "[Vec3f][angle]") {
    SECTION("Same direction (angle = 0)") {
        Vec3f a(1.0f, 0.0f, 0.0f);
        Vec3f b(2.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Perpendicular (angle = π/2)") {
        Vec3f a(1.0f, 0.0f, 0.0f);
        Vec3f b(0.0f, 1.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(mathf::half_pi, 1e-4f));
    }

    SECTION("Opposite direction (angle = π)") {
        Vec3f a(1.0f, 0.0f, 0.0f);
        Vec3f b(-1.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(mathf::pi, 1e-4f));
    }
}

// ============================================================================
// Clamping
// ============================================================================

TEST_CASE("Vec3f: Clamping", "[Vec3f][clamp]") {
    SECTION("Vector within limit") {
        Vec3f v(1.0f, 2.0f, 3.0f);
        Vec3f clamped = v.clamped(10.0f);
        REQUIRE_THAT(clamped.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(clamped.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(clamped.z, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Vector exceeds limit") {
        Vec3f v(2.0f, 3.0f, 6.0f);  // length = 7
        Vec3f clamped = v.clamped(3.5f);
        REQUIRE_THAT(clamped.length(), WithinAbs(3.5f, 1e-5f));
    }
}

// ============================================================================
// Linear Interpolation
// ============================================================================

TEST_CASE("Vec3f: Linear interpolation (lerp)", "[Vec3f][lerp]") {
    Vec3f a(0.0f, 0.0f, 0.0f);
    Vec3f b(10.0f, 20.0f, 30.0f);

    SECTION("t = 0") {
        Vec3f result = a.lerp(b, 0.0f);
        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("t = 1") {
        Vec3f result = a.lerp(b, 1.0f);
        REQUIRE_THAT(result.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(20.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(30.0f, 1e-6f));
    }

    SECTION("t = 0.5 (midpoint)") {
        Vec3f result = a.lerp(b, 0.5f);
        REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(15.0f, 1e-6f));
    }
}

// ============================================================================
// Projection
// ============================================================================

TEST_CASE("Vec3f: Projection", "[Vec3f][project]") {
    SECTION("Project onto parallel vector") {
        Vec3f v(4.0f, 0.0f, 0.0f);
        Vec3f onto(2.0f, 0.0f, 0.0f);
        Vec3f proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(proj.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(proj.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Project onto perpendicular vector") {
        Vec3f v(1.0f, 0.0f, 0.0f);
        Vec3f onto(0.0f, 2.0f, 0.0f);
        Vec3f proj = v.project(onto);
        REQUIRE_THAT(proj.length(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Perpendicular
// ============================================================================

TEST_CASE("Vec3f: Perpendicular vector generation", "[Vec3f][perpendicular]") {
    SECTION("Perpendicular is orthogonal") {
        Vec3f v(1.0f, 2.0f, 3.0f);
        Vec3f perp = v.perpendicular();
        REQUIRE_THAT(v.dot(perp), WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Perpendicular is normalized") {
        Vec3f v(3.0f, 4.0f, 5.0f);
        Vec3f perp = v.perpendicular();
        REQUIRE_THAT(perp.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Perpendicular to X-axis") {
        Vec3f x(1.0f, 0.0f, 0.0f);
        Vec3f perp = x.perpendicular();
        REQUIRE_THAT(x.dot(perp), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Perpendicular to Y-axis") {
        Vec3f y(0.0f, 1.0f, 0.0f);
        Vec3f perp = y.perpendicular();
        REQUIRE_THAT(y.dot(perp), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Reflection
// ============================================================================

TEST_CASE("Vec3f: Reflection", "[Vec3f][reflect]") {
    SECTION("Reflect off plane") {
        Vec3f v(1.0f, 1.0f, 0.0f);
        Vec3f normal(0.0f, 1.0f, 0.0f);
        Vec3f reflected = v.reflect(normal);
        REQUIRE_THAT(reflected.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(reflected.y, WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(reflected.z, WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Min/Max
// ============================================================================

TEST_CASE("Vec3f: Min/Max component-wise", "[Vec3f][minmax]") {
    Vec3f a(2.0f, 8.0f, 5.0f);
    Vec3f b(5.0f, 3.0f, 7.0f);

    SECTION("Min") {
        Vec3f result = a.min(b);
        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Max") {
        Vec3f result = a.max(b);
        REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(7.0f, 1e-6f));
    }
}

// ============================================================================
// Queries
// ============================================================================

TEST_CASE("Vec3f: Query functions", "[Vec3f][query]") {
    SECTION("isZero") {
        REQUIRE(Vec3f(0.0f, 0.0f, 0.0f).isZero());
        REQUIRE(!Vec3f(0.001f, 0.0f, 0.0f).isZero());
    }

    SECTION("isNormalized") {
        REQUIRE(Vec3f(1.0f, 0.0f, 0.0f).isNormalized());
        REQUIRE(!Vec3f(2.0f, 0.0f, 0.0f).isNormalized());
    }

    SECTION("approxEqual") {
        Vec3f a(1.0f, 2.0f, 3.0f);
        Vec3f b(1.00001f, 2.00001f, 3.00001f);
        Vec3f c(1.1f, 2.0f, 3.0f);
        REQUIRE(a.approxEqual(b, 1e-4f));
        REQUIRE(!a.approxEqual(c, 1e-4f));
    }

    SECTION("abs") {
        Vec3f v(-3.0f, 4.0f, -5.0f);
        Vec3f result = v.abs();
        REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-6f));
    }
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_CASE("Vec3f: Edge cases", "[Vec3f][edge]") {
    SECTION("Division by zero scalar") {
        Vec3f v(1.0f, 2.0f, 3.0f);
        Vec3f result = v / 0.0f;
        REQUIRE((std::isinf(result.x) || std::isnan(result.x)));
    }

    SECTION("Normalization of very small vector") {
        Vec3f v(1e-20f, 1e-20f, 1e-20f);
        Vec3f n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Cross product of parallel vectors") {
        Vec3f a(1.0f, 2.0f, 3.0f);
        Vec3f b(2.0f, 4.0f, 6.0f);
        Vec3f cross = a.cross(b);
        REQUIRE_THAT(cross.length(), WithinAbs(0.0f, 1e-4f));
    }

    SECTION("Angle between zero vectors") {
        Vec3f a(0.0f, 0.0f, 0.0f);
        Vec3f b(1.0f, 0.0f, 0.0f);
        float angle = a.angle(b);
        REQUIRE_THAT(angle, WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Static Utility Vectors
// ============================================================================

TEST_CASE("Vec3f: Static utility vectors", "[Vec3f][static]") {
    REQUIRE(Vec3f::zero() == Vec3f(0.0f, 0.0f, 0.0f));
    REQUIRE(Vec3f::one() == Vec3f(1.0f, 1.0f, 1.0f));
    REQUIRE(Vec3f::up() == Vec3f(0.0f, 1.0f, 0.0f));
    REQUIRE(Vec3f::down() == Vec3f(0.0f, -1.0f, 0.0f));
    REQUIRE(Vec3f::right() == Vec3f(1.0f, 0.0f, 0.0f));
    REQUIRE(Vec3f::left() == Vec3f(-1.0f, 0.0f, 0.0f));
    REQUIRE(Vec3f::forward() == Vec3f(0.0f, 0.0f, 1.0f));
    REQUIRE(Vec3f::back() == Vec3f(0.0f, 0.0f, -1.0f));
}

TEST_CASE("Vec3f: Stream output", "[Vec3f][stream]") {
    Vec3f v(1.5f, 2.5f, 3.5f);
    std::ostringstream oss;
    oss << v;
    REQUIRE(oss.str() == "(1.5, 2.5, 3.5)");
}

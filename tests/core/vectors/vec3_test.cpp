#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec3.hpp>
#include <cmath>

using phynity::math::Vec3;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Constructors and Basic Properties
// ============================================================================

TEST_CASE("Vec3: Constructors", "[Vec3][constructor]") {
    SECTION("Default constructor initializes to zero") {
        Vec3 v;
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Scalar constructor") {
        Vec3 v(5.0f);
        REQUIRE_THAT(v.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Component constructor") {
        Vec3 v(1.0f, 2.0f, 3.0f);
        REQUIRE_THAT(v.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(3.0f, 1e-6f));
    }
}

// ============================================================================
// Arithmetic Operators
// ============================================================================

TEST_CASE("Vec3: Addition", "[Vec3][arithmetic]") {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b;

    REQUIRE_THAT(c.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(c.y, WithinAbs(7.0f, 1e-6f));
    REQUIRE_THAT(c.z, WithinAbs(9.0f, 1e-6f));
}

TEST_CASE("Vec3: Subtraction", "[Vec3][arithmetic]") {
    Vec3 a(5.0f, 7.0f, 9.0f);
    Vec3 b(2.0f, 3.0f, 4.0f);
    Vec3 c = a - b;

    REQUIRE_THAT(c.x, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(c.y, WithinAbs(4.0f, 1e-6f));
    REQUIRE_THAT(c.z, WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("Vec3: Scalar multiplication", "[Vec3][arithmetic]") {
    Vec3 v(1.0f, 2.0f, 3.0f);
    
    SECTION("Vector * scalar") {
        Vec3 result = v * 2.0f;
        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Scalar * vector") {
        Vec3 result = 2.0f * v;
        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(6.0f, 1e-6f));
    }
}

TEST_CASE("Vec3: Scalar division", "[Vec3][arithmetic]") {
    Vec3 v(2.0f, 4.0f, 6.0f);
    Vec3 result = v / 2.0f;

    REQUIRE_THAT(result.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("Vec3: Component-wise multiplication", "[Vec3][arithmetic]") {
    Vec3 a(2.0f, 3.0f, 4.0f);
    Vec3 b(5.0f, 6.0f, 7.0f);
    Vec3 result = a * b;

    REQUIRE_THAT(result.x, WithinAbs(10.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(18.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(28.0f, 1e-6f));
}

TEST_CASE("Vec3: Component-wise division", "[Vec3][arithmetic]") {
    Vec3 a(10.0f, 18.0f, 28.0f);
    Vec3 b(2.0f, 3.0f, 4.0f);
    Vec3 result = a / b;

    REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(7.0f, 1e-6f));
}

TEST_CASE("Vec3: Negation", "[Vec3][arithmetic]") {
    Vec3 v(1.0f, -2.0f, 3.0f);
    Vec3 result = -v;

    REQUIRE_THAT(result.x, WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(result.y, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(result.z, WithinAbs(-3.0f, 1e-6f));
}

// ============================================================================
// Assignment Operators
// ============================================================================

TEST_CASE("Vec3: Assignment operators", "[Vec3][assignment]") {
    Vec3 v(1.0f, 2.0f, 3.0f);

    SECTION("Addition assignment") {
        v += Vec3(4.0f, 5.0f, 6.0f);
        REQUIRE_THAT(v.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(7.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Subtraction assignment") {
        v -= Vec3(1.0f, 1.0f, 1.0f);
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
}

// ============================================================================
// Comparison Operators
// ============================================================================

TEST_CASE("Vec3: Comparison operators", "[Vec3][comparison]") {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(1.0f, 2.0f, 3.0f);
    Vec3 c(2.0f, 3.0f, 4.0f);

    REQUIRE(a == b);
    REQUIRE(a != c);
    REQUIRE(!(a == c));
    REQUIRE(!(a != b));
}

// ============================================================================
// Index Access
// ============================================================================

TEST_CASE("Vec3: Index access", "[Vec3][access]") {
    Vec3 v(5.0f, 10.0f, 15.0f);

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

TEST_CASE("Vec3: Length operations", "[Vec3][magnitude]") {
    SECTION("1-2-2 triangle") {
        Vec3 v(2.0f, 3.0f, 6.0f);
        REQUIRE_THAT(v.squaredLength(), WithinAbs(49.0f, 1e-6f));
        REQUIRE_THAT(v.length(), WithinAbs(7.0f, 1e-6f));
    }

    SECTION("Zero vector") {
        Vec3 v(0.0f, 0.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.squaredLength(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Unit vector") {
        Vec3 v(1.0f, 0.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-6f));
    }
}

TEST_CASE("Vec3: Normalization", "[Vec3][normalization]") {
    SECTION("Normal vector becomes unit") {
        Vec3 v(2.0f, 3.0f, 6.0f);
        Vec3 n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Direction is preserved") {
        Vec3 v(1.0f, 0.0f, 0.0f);
        Vec3 n = v.normalized();
        REQUIRE_THAT(n.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(n.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(n.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Zero vector normalization") {
        Vec3 v(0.0f, 0.0f, 0.0f);
        Vec3 n = v.normalized();
        REQUIRE_THAT(n.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(n.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(n.z, WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Dot Product
// ============================================================================

TEST_CASE("Vec3: Dot product", "[Vec3][dot]") {
    SECTION("Orthogonal vectors") {
        Vec3 a(1.0f, 0.0f, 0.0f);
        Vec3 b(0.0f, 1.0f, 0.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Parallel vectors") {
        Vec3 a(1.0f, 2.0f, 3.0f);
        Vec3 b(2.0f, 4.0f, 6.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(28.0f, 1e-6f));
    }

    SECTION("Same vector (squared length)") {
        Vec3 v(1.0f, 2.0f, 3.0f);
        REQUIRE_THAT(v.dot(v), WithinAbs(14.0f, 1e-6f));
    }
}

// ============================================================================
// Cross Product
// ============================================================================

TEST_CASE("Vec3: Cross product", "[Vec3][cross]") {
    SECTION("Right hand rule: X × Y = Z") {
        Vec3 x(1.0f, 0.0f, 0.0f);
        Vec3 y(0.0f, 1.0f, 0.0f);
        Vec3 z = x.cross(y);
        REQUIRE_THAT(z.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(z.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(z.z, WithinAbs(1.0f, 1e-6f));
    }

    SECTION("Anti-commutative: A × B = -(B × A)") {
        Vec3 a(1.0f, 2.0f, 3.0f);
        Vec3 b(4.0f, 5.0f, 6.0f);
        Vec3 cross_ab = a.cross(b);
        Vec3 cross_ba = b.cross(a);
        REQUIRE_THAT(cross_ab.x, WithinAbs(-cross_ba.x, 1e-6f));
        REQUIRE_THAT(cross_ab.y, WithinAbs(-cross_ba.y, 1e-6f));
        REQUIRE_THAT(cross_ab.z, WithinAbs(-cross_ba.z, 1e-6f));
    }

    SECTION("Cross product is perpendicular") {
        Vec3 a(1.0f, 0.0f, 0.0f);
        Vec3 b(0.0f, 1.0f, 0.0f);
        Vec3 cross = a.cross(b);
        REQUIRE_THAT(a.dot(cross), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(b.dot(cross), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Parallel vectors have zero cross product") {
        Vec3 a(1.0f, 2.0f, 3.0f);
        Vec3 b(2.0f, 4.0f, 6.0f);
        Vec3 cross = a.cross(b);
        REQUIRE_THAT(cross.length(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Distance
// ============================================================================

TEST_CASE("Vec3: Distance", "[Vec3][distance]") {
    SECTION("Simple distance") {
        Vec3 a(0.0f, 0.0f, 0.0f);
        Vec3 b(3.0f, 4.0f, 0.0f);
        REQUIRE_THAT(a.distance(b), WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(b.distance(a), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Same point") {
        Vec3 a(5.0f, 5.0f, 5.0f);
        REQUIRE_THAT(a.distance(a), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Squared distance") {
        Vec3 a(0.0f, 0.0f, 0.0f);
        Vec3 b(2.0f, 3.0f, 6.0f);
        REQUIRE_THAT(a.squaredDistance(b), WithinAbs(49.0f, 1e-6f));
    }
}

// ============================================================================
// Angle Between Vectors
// ============================================================================

TEST_CASE("Vec3: Angle between vectors", "[Vec3][angle]") {
    SECTION("Same direction (angle = 0)") {
        Vec3 a(1.0f, 0.0f, 0.0f);
        Vec3 b(2.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Perpendicular (angle = π/2)") {
        Vec3 a(1.0f, 0.0f, 0.0f);
        Vec3 b(0.0f, 1.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f / 2.0f, 1e-4f));
    }

    SECTION("Opposite direction (angle = π)") {
        Vec3 a(1.0f, 0.0f, 0.0f);
        Vec3 b(-1.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f, 1e-4f));
    }
}

// ============================================================================
// Clamping
// ============================================================================

TEST_CASE("Vec3: Clamping", "[Vec3][clamp]") {
    SECTION("Vector within limit") {
        Vec3 v(1.0f, 2.0f, 3.0f);
        Vec3 clamped = v.clamped(10.0f);
        REQUIRE_THAT(clamped.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(clamped.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(clamped.z, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Vector exceeds limit") {
        Vec3 v(2.0f, 3.0f, 6.0f);  // length = 7
        Vec3 clamped = v.clamped(3.5f);
        REQUIRE_THAT(clamped.length(), WithinAbs(3.5f, 1e-5f));
    }
}

// ============================================================================
// Linear Interpolation
// ============================================================================

TEST_CASE("Vec3: Linear interpolation (lerp)", "[Vec3][lerp]") {
    Vec3 a(0.0f, 0.0f, 0.0f);
    Vec3 b(10.0f, 20.0f, 30.0f);

    SECTION("t = 0") {
        Vec3 result = a.lerp(b, 0.0f);
        REQUIRE_THAT(result.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("t = 1") {
        Vec3 result = a.lerp(b, 1.0f);
        REQUIRE_THAT(result.x, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(20.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(30.0f, 1e-6f));
    }

    SECTION("t = 0.5 (midpoint)") {
        Vec3 result = a.lerp(b, 0.5f);
        REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(15.0f, 1e-6f));
    }
}

// ============================================================================
// Projection
// ============================================================================

TEST_CASE("Vec3: Projection", "[Vec3][project]") {
    SECTION("Project onto parallel vector") {
        Vec3 v(4.0f, 0.0f, 0.0f);
        Vec3 onto(2.0f, 0.0f, 0.0f);
        Vec3 proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(proj.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(proj.z, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Project onto perpendicular vector") {
        Vec3 v(1.0f, 0.0f, 0.0f);
        Vec3 onto(0.0f, 2.0f, 0.0f);
        Vec3 proj = v.project(onto);
        REQUIRE_THAT(proj.length(), WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Reflection
// ============================================================================

TEST_CASE("Vec3: Reflection", "[Vec3][reflect]") {
    SECTION("Reflect off plane") {
        Vec3 v(1.0f, 1.0f, 0.0f);
        Vec3 normal(0.0f, 1.0f, 0.0f);
        Vec3 reflected = v.reflect(normal);
        REQUIRE_THAT(reflected.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(reflected.y, WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(reflected.z, WithinAbs(0.0f, 1e-6f));
    }
}

// ============================================================================
// Min/Max
// ============================================================================

TEST_CASE("Vec3: Min/Max component-wise", "[Vec3][minmax]") {
    Vec3 a(2.0f, 8.0f, 5.0f);
    Vec3 b(5.0f, 3.0f, 7.0f);

    SECTION("Min") {
        Vec3 result = a.min(b);
        REQUIRE_THAT(result.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Max") {
        Vec3 result = a.max(b);
        REQUIRE_THAT(result.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(8.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(7.0f, 1e-6f));
    }
}

// ============================================================================
// Queries
// ============================================================================

TEST_CASE("Vec3: Query functions", "[Vec3][query]") {
    SECTION("isZero") {
        REQUIRE(Vec3(0.0f, 0.0f, 0.0f).isZero());
        REQUIRE(!Vec3(0.001f, 0.0f, 0.0f).isZero());
    }

    SECTION("isNormalized") {
        REQUIRE(Vec3(1.0f, 0.0f, 0.0f).isNormalized());
        REQUIRE(!Vec3(2.0f, 0.0f, 0.0f).isNormalized());
    }
}

// ============================================================================
// Static Utility Vectors
// ============================================================================

TEST_CASE("Vec3: Static utility vectors", "[Vec3][static]") {
    REQUIRE(Vec3::zero() == Vec3(0.0f, 0.0f, 0.0f));
    REQUIRE(Vec3::one() == Vec3(1.0f, 1.0f, 1.0f));
    REQUIRE(Vec3::up() == Vec3(0.0f, 1.0f, 0.0f));
    REQUIRE(Vec3::down() == Vec3(0.0f, -1.0f, 0.0f));
    REQUIRE(Vec3::right() == Vec3(1.0f, 0.0f, 0.0f));
    REQUIRE(Vec3::left() == Vec3(-1.0f, 0.0f, 0.0f));
    REQUIRE(Vec3::forward() == Vec3(0.0f, 0.0f, 1.0f));
    REQUIRE(Vec3::back() == Vec3(0.0f, 0.0f, -1.0f));
}

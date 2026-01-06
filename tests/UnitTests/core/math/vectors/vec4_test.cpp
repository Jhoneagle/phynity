#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec4.hpp>
#include <cmath>
#include <sstream>

using phynity::math::vectors::Vec4f;
using Catch::Matchers::WithinAbs;

TEST_CASE("Vec4f: Constructors", "[Vec4f][constructor]") {
    SECTION("Default constructor") {
        Vec4f v;
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Component constructor") {
        Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.z, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(4.0f, 1e-6f));
    }
}

TEST_CASE("Vec4f: Arithmetic operations", "[Vec4f][arithmetic]") {
    Vec4f a(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4f b(5.0f, 6.0f, 7.0f, 8.0f);

    SECTION("Negation") {
        Vec4f v(1.0f, -2.0f, 3.0f, -4.0f);
        Vec4f neg = -v;
        REQUIRE_THAT(neg.x, WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(neg.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(neg.z, WithinAbs(-3.0f, 1e-6f));
        REQUIRE_THAT(neg.w, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Addition") {
        Vec4f c = a + b;
        REQUIRE_THAT(c.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(c.w, WithinAbs(12.0f, 1e-6f));
    }

    SECTION("Scalar multiplication") {
        Vec4f c = a * 2.0f;
        REQUIRE_THAT(c.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(c.w, WithinAbs(8.0f, 1e-6f));
    }
}

TEST_CASE("Vec4f: Vector operations", "[Vec4f][operations]") {
    SECTION("Length") {
        Vec4f v(1.0f, 2.0f, 2.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Dot product") {
        Vec4f a(1.0f, 0.0f, 0.0f, 0.0f);
        Vec4f b(0.0f, 1.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Normalization") {
        Vec4f v(3.0f, 4.0f, 0.0f, 0.0f);
        Vec4f n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("Vec4f: Static utilities", "[Vec4f][static]") {
    REQUIRE(Vec4f::zero() == Vec4f(0.0f, 0.0f, 0.0f, 0.0f));
    REQUIRE(Vec4f::one() == Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
    REQUIRE(Vec4f::unitX() == Vec4f(1.0f, 0.0f, 0.0f, 0.0f));
    REQUIRE(Vec4f::unitY() == Vec4f(0.0f, 1.0f, 0.0f, 0.0f));
    REQUIRE(Vec4f::unitZ() == Vec4f(0.0f, 0.0f, 1.0f, 0.0f));
    REQUIRE(Vec4f::unitW() == Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
}

TEST_CASE("Vec4f: Assignment operators", "[Vec4f][assignment]") {
    Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);

    SECTION("Addition assignment") {
        v += Vec4f(5.0f, 6.0f, 7.0f, 8.0f);
        REQUIRE_THAT(v.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(12.0f, 1e-6f));
    }

    SECTION("Subtraction assignment") {
        v -= Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Scalar multiplication assignment") {
        v *= 2.0f;
        REQUIRE_THAT(v.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(8.0f, 1e-6f));
    }

    SECTION("Scalar division assignment") {
        v /= 2.0f;
        REQUIRE_THAT(v.x, WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Component-wise multiplication assignment") {
        v *= Vec4f(2.0f, 3.0f, 4.0f, 5.0f);
        REQUIRE_THAT(v.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Component-wise division assignment") {
        v /= Vec4f(2.0f, 2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(1.0f, 1e-6f));
    }
}

TEST_CASE("Vec4f: Comparison operators", "[Vec4f][comparison]") {
    Vec4f a(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4f b(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4f c(2.0f, 3.0f, 4.0f, 5.0f);

    REQUIRE(a == b);
    REQUIRE(a != c);
    REQUIRE(!(a == c));
    REQUIRE(!(a != b));
}

TEST_CASE("Vec4f: Index access", "[Vec4f][access]") {
    Vec4f v(5.0f, 10.0f, 15.0f, 20.0f);

    SECTION("Read access") {
        REQUIRE_THAT(v[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v[1], WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(v[2], WithinAbs(15.0f, 1e-6f));
        REQUIRE_THAT(v[3], WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Write access") {
        v[0] = 25.0f;
        v[3] = 30.0f;
        REQUIRE_THAT(v.x, WithinAbs(25.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(30.0f, 1e-6f));
    }
}

TEST_CASE("Vec4f: Component-wise operations", "[Vec4f][arithmetic]") {
    Vec4f a(2.0f, 3.0f, 4.0f, 5.0f);
    Vec4f b(6.0f, 7.0f, 8.0f, 9.0f);

    SECTION("Component-wise multiplication") {
        Vec4f c = a * b;
        REQUIRE_THAT(c.x, WithinAbs(12.0f, 1e-6f));
        REQUIRE_THAT(c.w, WithinAbs(45.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        Vec4f c = b / a;
        REQUIRE_THAT(c.x, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(c.w, WithinAbs(1.8f, 1e-6f));
    }
}

TEST_CASE("Vec4f: Distance and angle", "[Vec4f][operations]") {
    SECTION("Distance") {
        Vec4f a(0.0f, 0.0f, 0.0f, 0.0f);
        Vec4f b(1.0f, 2.0f, 2.0f, 0.0f);
        REQUIRE_THAT(a.distance(b), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Squared distance") {
        Vec4f a(0.0f, 0.0f, 0.0f, 0.0f);
        Vec4f b(2.0f, 0.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.squaredDistance(b), WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Angle") {
        Vec4f a(1.0f, 0.0f, 0.0f, 0.0f);
        Vec4f b(0.0f, 1.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f / 2.0f, 1e-4f));
    }
}

TEST_CASE("Vec4f: Utility operations", "[Vec4f][utility]") {
    SECTION("Clamping") {
        Vec4f v(3.0f, 4.0f, 0.0f, 0.0f);
        Vec4f clamped = v.clamped(2.0f);
        REQUIRE_THAT(clamped.length(), WithinAbs(2.0f, 1e-5f));
    }

    SECTION("In-place clamp") {
        Vec4f v(3.0f, 4.0f, 0.0f, 0.0f);
        v.clamp(2.0f);
        REQUIRE_THAT(v.length(), WithinAbs(2.0f, 1e-5f));
    }

    SECTION("Lerp") {
        Vec4f a(0.0f, 0.0f, 0.0f, 0.0f);
        Vec4f b(10.0f, 20.0f, 30.0f, 40.0f);
        Vec4f mid = a.lerp(b, 0.5f);
        REQUIRE_THAT(mid.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(mid.w, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Project") {
        Vec4f v(4.0f, 0.0f, 0.0f, 0.0f);
        Vec4f onto(2.0f, 0.0f, 0.0f, 0.0f);
        Vec4f proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Reflect") {
        Vec4f v(1.0f, 1.0f, 0.0f, 0.0f);
        Vec4f normal(0.0f, 1.0f, 0.0f, 0.0f);
        Vec4f reflected = v.reflect(normal);
        REQUIRE_THAT(reflected.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(reflected.y, WithinAbs(-1.0f, 1e-6f));
    }

    SECTION("Min/Max") {
        Vec4f a(2.0f, 8.0f, 5.0f, 1.0f);
        Vec4f b(5.0f, 3.0f, 7.0f, 2.0f);
        Vec4f minV = a.min(b);
        Vec4f maxV = a.max(b);
        REQUIRE_THAT(minV.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(maxV.x, WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("Vec4f: Query functions", "[Vec4f][query]") {
    SECTION("isZero") {
        REQUIRE(Vec4f(0.0f, 0.0f, 0.0f, 0.0f).isZero());
        REQUIRE(!Vec4f(0.001f, 0.0f, 0.0f, 0.0f).isZero());
    }

    SECTION("isNormalized") {
        REQUIRE(Vec4f(1.0f, 0.0f, 0.0f, 0.0f).isNormalized());
        REQUIRE(!Vec4f(2.0f, 0.0f, 0.0f, 0.0f).isNormalized());
    }

    SECTION("approxEqual") {
        Vec4f a(1.0f, 2.0f, 3.0f, 4.0f);
        Vec4f b(1.00001f, 2.00001f, 3.00001f, 4.00001f);
        Vec4f c(1.1f, 2.0f, 3.0f, 4.0f);
        REQUIRE(a.approxEqual(b, 1e-4f));
        REQUIRE(!a.approxEqual(c, 1e-4f));
    }

    SECTION("abs") {
        Vec4f v(-3.0f, 4.0f, -5.0f, 6.0f);
        Vec4f result = v.abs();
        REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.w, WithinAbs(6.0f, 1e-6f));
    }
}

TEST_CASE("Vec4f: In-place normalize", "[Vec4f][normalization]") {
    Vec4f v(3.0f, 4.0f, 0.0f, 0.0f);
    v.normalize();
    REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Vec4f: Edge cases", "[Vec4f][edge]") {
    SECTION("Division by zero scalar") {
        Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
        Vec4f result = v / 0.0f;
        REQUIRE((std::isinf(result.x) || std::isnan(result.x)));
    }

    SECTION("Normalization of very small vector") {
        Vec4f v(1e-20f, 1e-20f, 1e-20f, 1e-20f);
        Vec4f n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Zero vector normalize in-place") {
        Vec4f v(0.0f, 0.0f, 0.0f, 0.0f);
        v.normalize();
        REQUIRE_THAT(v.length(), WithinAbs(0.0f, 1e-6f));
    }
}

TEST_CASE("Vec4f: Static utility vectors", "[Vec4f][static]") {
    REQUIRE(Vec4f::zero() == Vec4f(0.0f, 0.0f, 0.0f, 0.0f));
    REQUIRE(Vec4f::one() == Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
    REQUIRE(Vec4f::unitX() == Vec4f(1.0f, 0.0f, 0.0f, 0.0f));
    REQUIRE(Vec4f::unitY() == Vec4f(0.0f, 1.0f, 0.0f, 0.0f));
    REQUIRE(Vec4f::unitZ() == Vec4f(0.0f, 0.0f, 1.0f, 0.0f));
    REQUIRE(Vec4f::unitW() == Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
}

TEST_CASE("Vec4f: Stream output", "[Vec4f][stream]") {
    Vec4f v(1.5f, 2.5f, 3.5f, 4.5f);
    std::ostringstream oss;
    oss << v;
    REQUIRE(oss.str() == "(1.5, 2.5, 3.5, 4.5)");
}

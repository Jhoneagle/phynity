#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec4.hpp>
#include <cmath>
#include <sstream>

using phynity::math::vectors::Vec4;
using Catch::Matchers::WithinAbs;

TEST_CASE("Vec4: Constructors", "[Vec4][constructor]") {
    SECTION("Default constructor") {
        Vec4 v;
        REQUIRE_THAT(v.x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Component constructor") {
        Vec4 v(1.0f, 2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(4.0f, 1e-6f));
    }
}

TEST_CASE("Vec4: Arithmetic operations", "[Vec4][arithmetic]") {
    Vec4 a(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4 b(5.0f, 6.0f, 7.0f, 8.0f);

    SECTION("Negation") {
        Vec4 v(1.0f, -2.0f, 3.0f, -4.0f);
        Vec4 neg = -v;
        REQUIRE_THAT(neg.x, WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(neg.y, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(neg.z, WithinAbs(-3.0f, 1e-6f));
        REQUIRE_THAT(neg.w, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Addition") {
        Vec4 c = a + b;
        REQUIRE_THAT(c.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(c.w, WithinAbs(12.0f, 1e-6f));
    }

    SECTION("Scalar multiplication") {
        Vec4 c = a * 2.0f;
        REQUIRE_THAT(c.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(c.w, WithinAbs(8.0f, 1e-6f));
    }
}

TEST_CASE("Vec4: Vector operations", "[Vec4][operations]") {
    SECTION("Length") {
        Vec4 v(1.0f, 2.0f, 2.0f, 0.0f);
        REQUIRE_THAT(v.length(), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Dot product") {
        Vec4 a(1.0f, 0.0f, 0.0f, 0.0f);
        Vec4 b(0.0f, 1.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.dot(b), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Normalization") {
        Vec4 v(3.0f, 4.0f, 0.0f, 0.0f);
        Vec4 n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("Vec4: Static utilities", "[Vec4][static]") {
    REQUIRE(Vec4::zero() == Vec4(0.0f, 0.0f, 0.0f, 0.0f));
    REQUIRE(Vec4::one() == Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    REQUIRE(Vec4::unitX() == Vec4(1.0f, 0.0f, 0.0f, 0.0f));
    REQUIRE(Vec4::unitY() == Vec4(0.0f, 1.0f, 0.0f, 0.0f));
    REQUIRE(Vec4::unitZ() == Vec4(0.0f, 0.0f, 1.0f, 0.0f));
    REQUIRE(Vec4::unitW() == Vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

TEST_CASE("Vec4: Assignment operators", "[Vec4][assignment]") {
    Vec4 v(1.0f, 2.0f, 3.0f, 4.0f);

    SECTION("Addition assignment") {
        v += Vec4(5.0f, 6.0f, 7.0f, 8.0f);
        REQUIRE_THAT(v.x, WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(12.0f, 1e-6f));
    }

    SECTION("Subtraction assignment") {
        v -= Vec4(1.0f, 1.0f, 1.0f, 1.0f);
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
        v *= Vec4(2.0f, 3.0f, 4.0f, 5.0f);
        REQUIRE_THAT(v.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Component-wise division assignment") {
        v /= Vec4(2.0f, 2.0f, 3.0f, 4.0f);
        REQUIRE_THAT(v.x, WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(v.w, WithinAbs(1.0f, 1e-6f));
    }
}

TEST_CASE("Vec4: Comparison operators", "[Vec4][comparison]") {
    Vec4 a(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4 b(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4 c(2.0f, 3.0f, 4.0f, 5.0f);

    REQUIRE(a == b);
    REQUIRE(a != c);
    REQUIRE(!(a == c));
    REQUIRE(!(a != b));
}

TEST_CASE("Vec4: Index access", "[Vec4][access]") {
    Vec4 v(5.0f, 10.0f, 15.0f, 20.0f);

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

TEST_CASE("Vec4: Component-wise operations", "[Vec4][arithmetic]") {
    Vec4 a(2.0f, 3.0f, 4.0f, 5.0f);
    Vec4 b(6.0f, 7.0f, 8.0f, 9.0f);

    SECTION("Component-wise multiplication") {
        Vec4 c = a * b;
        REQUIRE_THAT(c.x, WithinAbs(12.0f, 1e-6f));
        REQUIRE_THAT(c.w, WithinAbs(45.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        Vec4 c = b / a;
        REQUIRE_THAT(c.x, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(c.w, WithinAbs(1.8f, 1e-6f));
    }
}

TEST_CASE("Vec4: Distance and angle", "[Vec4][operations]") {
    SECTION("Distance") {
        Vec4 a(0.0f, 0.0f, 0.0f, 0.0f);
        Vec4 b(1.0f, 2.0f, 2.0f, 0.0f);
        REQUIRE_THAT(a.distance(b), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Squared distance") {
        Vec4 a(0.0f, 0.0f, 0.0f, 0.0f);
        Vec4 b(2.0f, 0.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.squaredDistance(b), WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Angle") {
        Vec4 a(1.0f, 0.0f, 0.0f, 0.0f);
        Vec4 b(0.0f, 1.0f, 0.0f, 0.0f);
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f / 2.0f, 1e-4f));
    }
}

TEST_CASE("Vec4: Utility operations", "[Vec4][utility]") {
    SECTION("Clamping") {
        Vec4 v(3.0f, 4.0f, 0.0f, 0.0f);
        Vec4 clamped = v.clamped(2.0f);
        REQUIRE_THAT(clamped.length(), WithinAbs(2.0f, 1e-5f));
    }

    SECTION("In-place clamp") {
        Vec4 v(3.0f, 4.0f, 0.0f, 0.0f);
        v.clamp(2.0f);
        REQUIRE_THAT(v.length(), WithinAbs(2.0f, 1e-5f));
    }

    SECTION("Lerp") {
        Vec4 a(0.0f, 0.0f, 0.0f, 0.0f);
        Vec4 b(10.0f, 20.0f, 30.0f, 40.0f);
        Vec4 mid = a.lerp(b, 0.5f);
        REQUIRE_THAT(mid.x, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(mid.w, WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Project") {
        Vec4 v(4.0f, 0.0f, 0.0f, 0.0f);
        Vec4 onto(2.0f, 0.0f, 0.0f, 0.0f);
        Vec4 proj = v.project(onto);
        REQUIRE_THAT(proj.x, WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Reflect") {
        Vec4 v(1.0f, 1.0f, 0.0f, 0.0f);
        Vec4 normal(0.0f, 1.0f, 0.0f, 0.0f);
        Vec4 reflected = v.reflect(normal);
        REQUIRE_THAT(reflected.x, WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(reflected.y, WithinAbs(-1.0f, 1e-6f));
    }

    SECTION("Min/Max") {
        Vec4 a(2.0f, 8.0f, 5.0f, 1.0f);
        Vec4 b(5.0f, 3.0f, 7.0f, 2.0f);
        Vec4 minV = a.min(b);
        Vec4 maxV = a.max(b);
        REQUIRE_THAT(minV.x, WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(maxV.x, WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("Vec4: Query functions", "[Vec4][query]") {
    SECTION("isZero") {
        REQUIRE(Vec4(0.0f, 0.0f, 0.0f, 0.0f).isZero());
        REQUIRE(!Vec4(0.001f, 0.0f, 0.0f, 0.0f).isZero());
    }

    SECTION("isNormalized") {
        REQUIRE(Vec4(1.0f, 0.0f, 0.0f, 0.0f).isNormalized());
        REQUIRE(!Vec4(2.0f, 0.0f, 0.0f, 0.0f).isNormalized());
    }

    SECTION("approxEqual") {
        Vec4 a(1.0f, 2.0f, 3.0f, 4.0f);
        Vec4 b(1.00001f, 2.00001f, 3.00001f, 4.00001f);
        Vec4 c(1.1f, 2.0f, 3.0f, 4.0f);
        REQUIRE(a.approxEqual(b, 1e-4f));
        REQUIRE(!a.approxEqual(c, 1e-4f));
    }

    SECTION("abs") {
        Vec4 v(-3.0f, 4.0f, -5.0f, 6.0f);
        Vec4 result = v.abs();
        REQUIRE_THAT(result.x, WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result.y, WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result.z, WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(result.w, WithinAbs(6.0f, 1e-6f));
    }
}

TEST_CASE("Vec4: In-place normalize", "[Vec4][normalization]") {
    Vec4 v(3.0f, 4.0f, 0.0f, 0.0f);
    v.normalize();
    REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("Vec4: Edge cases", "[Vec4][edge]") {
    SECTION("Division by zero scalar") {
        Vec4 v(1.0f, 2.0f, 3.0f, 4.0f);
        Vec4 result = v / 0.0f;
        REQUIRE((std::isinf(result.x) || std::isnan(result.x)));
    }

    SECTION("Normalization of very small vector") {
        Vec4 v(1e-20f, 1e-20f, 1e-20f, 1e-20f);
        Vec4 n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Zero vector normalize in-place") {
        Vec4 v(0.0f, 0.0f, 0.0f, 0.0f);
        v.normalize();
        REQUIRE_THAT(v.length(), WithinAbs(0.0f, 1e-6f));
    }
}

TEST_CASE("Vec4: Static utility vectors", "[Vec4][static]") {
    REQUIRE(Vec4::zero() == Vec4(0.0f, 0.0f, 0.0f, 0.0f));
    REQUIRE(Vec4::one() == Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    REQUIRE(Vec4::unitX() == Vec4(1.0f, 0.0f, 0.0f, 0.0f));
    REQUIRE(Vec4::unitY() == Vec4(0.0f, 1.0f, 0.0f, 0.0f));
    REQUIRE(Vec4::unitZ() == Vec4(0.0f, 0.0f, 1.0f, 0.0f));
    REQUIRE(Vec4::unitW() == Vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

TEST_CASE("Vec4: Stream output", "[Vec4][stream]") {
    Vec4 v(1.5f, 2.5f, 3.5f, 4.5f);
    std::ostringstream oss;
    oss << v;
    REQUIRE(oss.str() == "(1.5, 2.5, 3.5, 4.5)");
}

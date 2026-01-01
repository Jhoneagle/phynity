#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec4.hpp>
#include <cmath>

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
}

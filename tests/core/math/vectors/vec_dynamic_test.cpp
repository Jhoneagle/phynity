#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec_dynamic.hpp>
#include <sstream>
#include <cmath>

using phynity::math::vectors::VecDynamic;
using Catch::Matchers::WithinAbs;

TEST_CASE("VecDynamic: Constructors", "[VecDynamic][constructor]") {
    SECTION("Size constructor") {
        VecDynamic v(5);
        REQUIRE(v.size() == 5);
        for (std::size_t i = 0; i < 5; ++i) {
            REQUIRE_THAT(v[i], WithinAbs(0.0f, 1e-6f));
        }
    }

    SECTION("Size and value constructor") {
        VecDynamic v(3, 5.0f);
        REQUIRE(v.size() == 3);
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE_THAT(v[i], WithinAbs(5.0f, 1e-6f));
        }
    }
}

TEST_CASE("VecDynamic: Access methods", "[VecDynamic][access]") {
    VecDynamic v(4, 0.0f);
    v[0] = 1.0f;
    v[1] = 2.0f;
    v[2] = 3.0f;
    v[3] = 4.0f;

    SECTION("Operator[]") {
        REQUIRE_THAT(v[0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v[3], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("at() method") {
        REQUIRE_THAT(v.at(0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v.at(3), WithinAbs(4.0f, 1e-6f));
    }

    SECTION("at() bounds checking") {
        REQUIRE_THROWS(v.at(100));
    }
}

TEST_CASE("VecDynamic: Arithmetic operations", "[VecDynamic][arithmetic]") {
    VecDynamic a(3, 0.0f);
    a[0] = 1.0f; a[1] = 2.0f; a[2] = 3.0f;
    
    VecDynamic b(3, 0.0f);
    b[0] = 4.0f; b[1] = 5.0f; b[2] = 6.0f;

    SECTION("Negation") {
        VecDynamic v(3, 0.0f);
        v[0] = 1.0f; v[1] = -2.0f; v[2] = 3.0f;
        VecDynamic neg = -v;
        REQUIRE_THAT(neg[0], WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(neg[1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(neg[2], WithinAbs(-3.0f, 1e-6f));
    }

    SECTION("Addition") {
        VecDynamic c = a + b;
        REQUIRE(c.size() == 3);
        REQUIRE_THAT(c[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(c[2], WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Subtraction") {
        VecDynamic c = b - a;
        REQUIRE_THAT(c[0], WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Scalar multiplication") {
        VecDynamic c = a * 2.0f;
        REQUIRE_THAT(c[0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(c[2], WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Size mismatch addition throws") {
        VecDynamic c(2, 0.0f);
        REQUIRE_THROWS(a + c);
    }
}

TEST_CASE("VecDynamic: Vector operations", "[VecDynamic][operations]") {
    VecDynamic v(5);
    v[0] = 1.0f; v[1] = 2.0f; v[2] = 2.0f; v[3] = 0.0f; v[4] = 0.0f;

    SECTION("Length") {
        REQUIRE_THAT(v.length(), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Squared length") {
        REQUIRE_THAT(v.squaredLength(), WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Dot product") {
        VecDynamic a(3, 0.0f);
        a[0] = 1.0f;
        
        VecDynamic b(3, 0.0f);
        b[0] = 0.0f; b[1] = 1.0f;
        
        REQUIRE_THAT(a.dot(b), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Normalization") {
        VecDynamic n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("VecDynamic: Distance operations", "[VecDynamic][distance]") {
    VecDynamic a(3, 0.0f);
    a[0] = 0.0f; a[1] = 0.0f; a[2] = 0.0f;
    
    VecDynamic b(3, 0.0f);
    b[0] = 3.0f; b[1] = 4.0f; b[2] = 0.0f;

    SECTION("Distance") {
        REQUIRE_THAT(a.distance(b), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Squared distance") {
        REQUIRE_THAT(a.squaredDistance(b), WithinAbs(25.0f, 1e-6f));
    }
}

TEST_CASE("VecDynamic: Resize", "[VecDynamic][resize]") {
    VecDynamic v(3, 5.0f);
    
    SECTION("Resize to larger") {
        v.resize(5);
        REQUIRE(v.size() == 5);
        REQUIRE_THAT(v[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(v[3], WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Resize with value") {
        v.resize(4, 10.0f);
        REQUIRE(v.size() == 4);
        REQUIRE_THAT(v[3], WithinAbs(10.0f, 1e-6f));
    }
}

TEST_CASE("VecDynamic: Query functions", "[VecDynamic][query]") {
    SECTION("isZero") {
        VecDynamic v(3, 0.0f);
        REQUIRE(v.isZero());
        
        v[0] = 0.001f;
        REQUIRE(!v.isZero());
    }

    SECTION("empty()") {
        VecDynamic v;
        REQUIRE(v.empty());
        
        VecDynamic v2(5);
        REQUIRE(!v2.empty());
    }

    SECTION("isNormalized") {
        VecDynamic v(3, 0.0f);
        v[0] = 1.0f;
        REQUIRE(v.isNormalized());
        v[0] = 2.0f;
        REQUIRE(!v.isNormalized());
    }

    SECTION("approxEqual") {
        VecDynamic a(3, 0.0f);
        a[0] = 1.0f; a[1] = 2.0f; a[2] = 3.0f;
        VecDynamic b(3, 0.0f);
        b[0] = 1.00001f; b[1] = 2.00001f; b[2] = 3.00001f;
        VecDynamic c(3, 0.0f);
        c[0] = 1.1f; c[1] = 2.0f; c[2] = 3.0f;
        REQUIRE(a.approxEqual(b, 1e-4f));
        REQUIRE(!a.approxEqual(c, 1e-4f));
    }

    SECTION("abs") {
        VecDynamic v(3, 0.0f);
        v[0] = -3.0f; v[1] = 4.0f; v[2] = -5.0f;
        VecDynamic result = v.abs();
        REQUIRE_THAT(result[0], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result[1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result[2], WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("VecDynamic: Component-wise assignment", "[VecDynamic][assignment]") {
    VecDynamic v(3, 0.0f);
    v[0] = 2.0f; v[1] = 4.0f; v[2] = 6.0f;

    SECTION("Component-wise multiplication assignment") {
        VecDynamic other(3, 0.0f);
        other[0] = 2.0f; other[1] = 3.0f; other[2] = 4.0f;
        v *= other;
        REQUIRE_THAT(v[0], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(v[2], WithinAbs(24.0f, 1e-6f));
    }

    SECTION("Component-wise division assignment") {
        VecDynamic other(3, 0.0f);
        other[0] = 2.0f; other[1] = 2.0f; other[2] = 3.0f;
        v /= other;
        REQUIRE_THAT(v[0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v[2], WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Size mismatch throws") {
        VecDynamic other(2, 1.0f);
        REQUIRE_THROWS(v *= other);
        REQUIRE_THROWS(v /= other);
    }
}

TEST_CASE("VecDynamic: In-place normalize", "[VecDynamic][normalization]") {
    VecDynamic v(3, 0.0f);
    v[0] = 2.0f; v[1] = 3.0f; v[2] = 6.0f;
    v.normalize();
    REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("VecDynamic: Utility operations", "[VecDynamic][utility]") {
    SECTION("Lerp") {
        VecDynamic a(3, 0.0f);
        VecDynamic b(3, 0.0f);
        b[0] = 10.0f; b[1] = 20.0f; b[2] = 30.0f;
        VecDynamic mid = a.lerp(b, 0.5f);
        REQUIRE_THAT(mid[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(mid[2], WithinAbs(15.0f, 1e-6f));
    }

    SECTION("Project") {
        VecDynamic v(3, 0.0f);
        v[0] = 4.0f;
        VecDynamic onto(3, 0.0f);
        onto[0] = 2.0f;
        VecDynamic proj = v.project(onto);
        REQUIRE_THAT(proj[0], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Reflect") {
        VecDynamic v(3, 0.0f);
        v[0] = 1.0f; v[1] = 1.0f;
        VecDynamic normal(3, 0.0f);
        normal[1] = 1.0f;
        VecDynamic reflected = v.reflect(normal);
        REQUIRE_THAT(reflected[0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(reflected[1], WithinAbs(-1.0f, 1e-6f));
    }

    SECTION("Clamp") {
        VecDynamic v(3, 0.0f);
        v[0] = 3.0f; v[1] = 4.0f;
        VecDynamic clamped = v.clamped(2.0f);
        REQUIRE_THAT(clamped.length(), WithinAbs(2.0f, 1e-5f));
    }

    SECTION("Min/Max") {
        VecDynamic a(3, 0.0f);
        a[0] = 2.0f; a[1] = 8.0f; a[2] = 5.0f;
        VecDynamic b(3, 0.0f);
        b[0] = 5.0f; b[1] = 3.0f; b[2] = 7.0f;
        VecDynamic minV = a.min(b);
        VecDynamic maxV = a.max(b);
        REQUIRE_THAT(minV[0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(maxV[0], WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Angle") {
        VecDynamic a(3, 0.0f);
        a[0] = 1.0f;
        VecDynamic b(3, 0.0f);
        b[1] = 1.0f;
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f / 2.0f, 1e-4f));
    }
}

TEST_CASE("VecDynamic: Edge cases", "[VecDynamic][edge]") {
    SECTION("Division by zero scalar") {
        VecDynamic v(3, 0.0f);
        v[0] = 1.0f; v[1] = 2.0f; v[2] = 3.0f;
        VecDynamic result = v / 0.0f;
        REQUIRE((std::isinf(result[0]) || std::isnan(result[0])));
    }

    SECTION("Normalization of very small vector") {
        VecDynamic v(3, 1e-20f);
        VecDynamic n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Empty vector operations") {
        VecDynamic v;
        REQUIRE(v.empty());
        REQUIRE(v.size() == 0);
        REQUIRE_THAT(v.length(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Resize operations") {
        VecDynamic v(3, 5.0f);
        v.resize(1);
        REQUIRE(v.size() == 1);
        REQUIRE_THAT(v[0], WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("VecDynamic: Stream output", "[VecDynamic][stream]") {
    VecDynamic v(3, 0.0f);
    v[0] = 1.5f; v[1] = 2.5f; v[2] = 3.5f;
    std::ostringstream oss;
    oss << v;
    REQUIRE(oss.str() == "(1.5, 2.5, 3.5)");
}

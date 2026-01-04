#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec_dynamic.hpp>
#include <sstream>
#include <cmath>

using phynity::math::vectors::VecDynamicf;
using Catch::Matchers::WithinAbs;

TEST_CASE("VecDynamicf: Constructors", "[VecDynamicf][constructor]") {
    SECTION("Size constructor") {
        VecDynamicf v(5);
        REQUIRE(v.size() == 5);
        for (std::size_t i = 0; i < 5; ++i) {
            REQUIRE_THAT(v[i], WithinAbs(0.0f, 1e-6f));
        }
    }

    SECTION("Size and value constructor") {
        VecDynamicf v(3, 5.0f);
        REQUIRE(v.size() == 3);
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE_THAT(v[i], WithinAbs(5.0f, 1e-6f));
        }
    }
}

TEST_CASE("VecDynamicf: Access methods", "[VecDynamicf][access]") {
    VecDynamicf v(4, 0.0f);
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

TEST_CASE("VecDynamicf: Arithmetic operations", "[VecDynamicf][arithmetic]") {
    VecDynamicf a(3, 0.0f);
    a[0] = 1.0f; a[1] = 2.0f; a[2] = 3.0f;
    
    VecDynamicf b(3, 0.0f);
    b[0] = 4.0f; b[1] = 5.0f; b[2] = 6.0f;

    SECTION("Negation") {
        VecDynamicf v(3, 0.0f);
        v[0] = 1.0f; v[1] = -2.0f; v[2] = 3.0f;
        VecDynamicf neg = -v;
        REQUIRE_THAT(neg[0], WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(neg[1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(neg[2], WithinAbs(-3.0f, 1e-6f));
    }

    SECTION("Addition") {
        VecDynamicf c = a + b;
        REQUIRE(c.size() == 3);
        REQUIRE_THAT(c[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(c[2], WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Subtraction") {
        VecDynamicf c = b - a;
        REQUIRE_THAT(c[0], WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Scalar multiplication") {
        VecDynamicf c = a * 2.0f;
        REQUIRE_THAT(c[0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(c[2], WithinAbs(6.0f, 1e-6f));
    }

    SECTION("Size mismatch addition throws") {
        VecDynamicf c(2, 0.0f);
        REQUIRE_THROWS(a + c);
    }
}

TEST_CASE("VecDynamicf: Vector operations", "[VecDynamicf][operations]") {
    VecDynamicf v(5);
    v[0] = 1.0f; v[1] = 2.0f; v[2] = 2.0f; v[3] = 0.0f; v[4] = 0.0f;

    SECTION("Length") {
        REQUIRE_THAT(v.length(), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Squared length") {
        REQUIRE_THAT(v.squaredLength(), WithinAbs(9.0f, 1e-6f));
    }

    SECTION("Dot product") {
        VecDynamicf a(3, 0.0f);
        a[0] = 1.0f;
        
        VecDynamicf b(3, 0.0f);
        b[0] = 0.0f; b[1] = 1.0f;
        
        REQUIRE_THAT(a.dot(b), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Normalization") {
        VecDynamicf n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("VecDynamicf: Distance operations", "[VecDynamicf][distance]") {
    VecDynamicf a(3, 0.0f);
    a[0] = 0.0f; a[1] = 0.0f; a[2] = 0.0f;
    
    VecDynamicf b(3, 0.0f);
    b[0] = 3.0f; b[1] = 4.0f; b[2] = 0.0f;

    SECTION("Distance") {
        REQUIRE_THAT(a.distance(b), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Squared distance") {
        REQUIRE_THAT(a.squaredDistance(b), WithinAbs(25.0f, 1e-6f));
    }
}

TEST_CASE("VecDynamicf: Resize", "[VecDynamicf][resize]") {
    VecDynamicf v(3, 5.0f);
    
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

TEST_CASE("VecDynamicf: Query functions", "[VecDynamicf][query]") {
    SECTION("isZero") {
        VecDynamicf v(3, 0.0f);
        REQUIRE(v.isZero());
        
        v[0] = 0.001f;
        REQUIRE(!v.isZero());
    }

    SECTION("empty()") {
        VecDynamicf v;
        REQUIRE(v.empty());
        
        VecDynamicf v2(5);
        REQUIRE(!v2.empty());
    }

    SECTION("isNormalized") {
        VecDynamicf v(3, 0.0f);
        v[0] = 1.0f;
        REQUIRE(v.isNormalized());
        v[0] = 2.0f;
        REQUIRE(!v.isNormalized());
    }

    SECTION("approxEqual") {
        VecDynamicf a(3, 0.0f);
        a[0] = 1.0f; a[1] = 2.0f; a[2] = 3.0f;
        VecDynamicf b(3, 0.0f);
        b[0] = 1.00001f; b[1] = 2.00001f; b[2] = 3.00001f;
        VecDynamicf c(3, 0.0f);
        c[0] = 1.1f; c[1] = 2.0f; c[2] = 3.0f;
        REQUIRE(a.approxEqual(b, 1e-4f));
        REQUIRE(!a.approxEqual(c, 1e-4f));
    }

    SECTION("abs") {
        VecDynamicf v(3, 0.0f);
        v[0] = -3.0f; v[1] = 4.0f; v[2] = -5.0f;
        VecDynamicf result = v.abs();
        REQUIRE_THAT(result[0], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result[1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result[2], WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("VecDynamicf: Component-wise assignment", "[VecDynamicf][assignment]") {
    VecDynamicf v(3, 0.0f);
    v[0] = 2.0f; v[1] = 4.0f; v[2] = 6.0f;

    SECTION("Component-wise multiplication assignment") {
        VecDynamicf other(3, 0.0f);
        other[0] = 2.0f; other[1] = 3.0f; other[2] = 4.0f;
        v *= other;
        REQUIRE_THAT(v[0], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(v[2], WithinAbs(24.0f, 1e-6f));
    }

    SECTION("Component-wise division assignment") {
        VecDynamicf other(3, 0.0f);
        other[0] = 2.0f; other[1] = 2.0f; other[2] = 3.0f;
        v /= other;
        REQUIRE_THAT(v[0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(v[2], WithinAbs(2.0f, 1e-6f));
    }

    SECTION("Size mismatch throws") {
        VecDynamicf other(2, 1.0f);
        REQUIRE_THROWS(v *= other);
        REQUIRE_THROWS(v /= other);
    }
}

TEST_CASE("VecDynamicf: In-place normalize", "[VecDynamicf][normalization]") {
    VecDynamicf v(3, 0.0f);
    v[0] = 2.0f; v[1] = 3.0f; v[2] = 6.0f;
    v.normalize();
    REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("VecDynamicf: Utility operations", "[VecDynamicf][utility]") {
    SECTION("Lerp") {
        VecDynamicf a(3, 0.0f);
        VecDynamicf b(3, 0.0f);
        b[0] = 10.0f; b[1] = 20.0f; b[2] = 30.0f;
        VecDynamicf mid = a.lerp(b, 0.5f);
        REQUIRE_THAT(mid[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(mid[2], WithinAbs(15.0f, 1e-6f));
    }

    SECTION("Project") {
        VecDynamicf v(3, 0.0f);
        v[0] = 4.0f;
        VecDynamicf onto(3, 0.0f);
        onto[0] = 2.0f;
        VecDynamicf proj = v.project(onto);
        REQUIRE_THAT(proj[0], WithinAbs(4.0f, 1e-6f));
    }

    SECTION("Reflect") {
        VecDynamicf v(3, 0.0f);
        v[0] = 1.0f; v[1] = 1.0f;
        VecDynamicf normal(3, 0.0f);
        normal[1] = 1.0f;
        VecDynamicf reflected = v.reflect(normal);
        REQUIRE_THAT(reflected[0], WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(reflected[1], WithinAbs(-1.0f, 1e-6f));
    }

    SECTION("Clamp") {
        VecDynamicf v(3, 0.0f);
        v[0] = 3.0f; v[1] = 4.0f;
        VecDynamicf clamped = v.clamped(2.0f);
        REQUIRE_THAT(clamped.length(), WithinAbs(2.0f, 1e-5f));
    }

    SECTION("Min/Max") {
        VecDynamicf a(3, 0.0f);
        a[0] = 2.0f; a[1] = 8.0f; a[2] = 5.0f;
        VecDynamicf b(3, 0.0f);
        b[0] = 5.0f; b[1] = 3.0f; b[2] = 7.0f;
        VecDynamicf minV = a.min(b);
        VecDynamicf maxV = a.max(b);
        REQUIRE_THAT(minV[0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(maxV[0], WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Angle") {
        VecDynamicf a(3, 0.0f);
        a[0] = 1.0f;
        VecDynamicf b(3, 0.0f);
        b[1] = 1.0f;
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f / 2.0f, 1e-4f));
    }
}

TEST_CASE("VecDynamicf: Edge cases", "[VecDynamicf][edge]") {
    SECTION("Division by zero scalar") {
        VecDynamicf v(3, 0.0f);
        v[0] = 1.0f; v[1] = 2.0f; v[2] = 3.0f;
        VecDynamicf result = v / 0.0f;
        REQUIRE((std::isinf(result[0]) || std::isnan(result[0])));
    }

    SECTION("Normalization of very small vector") {
        VecDynamicf v(3, 1e-20f);
        VecDynamicf n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Empty vector operations") {
        VecDynamicf v;
        REQUIRE(v.empty());
        REQUIRE(v.size() == 0);
        REQUIRE_THAT(v.length(), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("Resize operations") {
        VecDynamicf v(3, 5.0f);
        v.resize(1);
        REQUIRE(v.size() == 1);
        REQUIRE_THAT(v[0], WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("VecDynamicf: Stream output", "[VecDynamicf][stream]") {
    VecDynamicf v(3, 0.0f);
    v[0] = 1.5f; v[1] = 2.5f; v[2] = 3.5f;
    std::ostringstream oss;
    oss << v;
    REQUIRE(oss.str() == "(1.5, 2.5, 3.5)");
}

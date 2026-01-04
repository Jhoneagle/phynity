#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec_n.hpp>
#include <sstream>
#include <cmath>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::VecN;

TEST_CASE("VecN: Constructors", "[VecN][constructor]") {
    SECTION("Default constructor") {
        VecN<4> v;
        for (std::size_t i = 0; i < 4; ++i) {
            REQUIRE_THAT(v[i], WithinAbs(0.0f, 1e-6f));
        }
    }

    SECTION("Scalar constructor") {
        VecN<3> v(5.0f);
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE_THAT(v[i], WithinAbs(5.0f, 1e-6f));
        }
    }
}

TEST_CASE("VecN: Arithmetic operations", "[VecN][arithmetic]") {
    VecN<4> a;
    a[0] = 1.0f; a[1] = 2.0f; a[2] = 3.0f; a[3] = 4.0f;
    
    VecN<4> b;
    b[0] = 5.0f; b[1] = 6.0f; b[2] = 7.0f; b[3] = 8.0f;

    SECTION("Negation") {
        VecN<3> v;
        v[0] = 1.0f; v[1] = -2.0f; v[2] = 3.0f;
        VecN<3> neg = -v;
        REQUIRE_THAT(neg[0], WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(neg[1], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(neg[2], WithinAbs(-3.0f, 1e-6f));
    }

    SECTION("Addition") {
        VecN<4> c = a + b;
        REQUIRE_THAT(c[0], WithinAbs(6.0f, 1e-6f));
        REQUIRE_THAT(c[3], WithinAbs(12.0f, 1e-6f));
    }

    SECTION("Scalar multiplication") {
        VecN<4> c = a * 2.0f;
        REQUIRE_THAT(c[0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(c[3], WithinAbs(8.0f, 1e-6f));
    }
}

TEST_CASE("VecN: Vector operations", "[VecN][operations]") {
    VecN<5> v;
    v[0] = 1.0f; v[1] = 2.0f; v[2] = 2.0f; v[3] = 0.0f; v[4] = 0.0f;

    SECTION("Length") {
        REQUIRE_THAT(v.length(), WithinAbs(3.0f, 1e-6f));
    }

    SECTION("Normalization") {
        VecN<5> n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("VecN: 6D vector (robotics use case)", "[VecN][6D]") {
    VecN<6> state;  // 3 linear + 3 angular
    for (std::size_t i = 0; i < 6; ++i) state[i] = static_cast<float>(i + 1);

    SECTION("Initialization") {
        for (std::size_t i = 0; i < 6; ++i) {
            REQUIRE_THAT(state[i], WithinAbs(static_cast<float>(i + 1), 1e-6f));
        }
    }

    SECTION("Operations preserve size") {
        VecN<6> scaled = state * 2.0f;
        REQUIRE(scaled.size() == 6);
    }

    SECTION("Static size method") {
        REQUIRE(VecN<6>::size() == 6);
        REQUIRE(VecN<3>::size() == 3);
        REQUIRE(VecN<16>::size() == 16);
    }
}

TEST_CASE("VecN: Assignment operators", "[VecN][assignment]") {
    VecN<4> v;
    v[0] = 1.0f; v[1] = 2.0f; v[2] = 3.0f; v[3] = 4.0f;

    SECTION("Scalar multiplication assignment") {
        v *= 2.0f;
        REQUIRE_THAT(v[0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v[3], WithinAbs(8.0f, 1e-6f));
    }

    SECTION("Component-wise multiplication assignment") {
        VecN<4> other;
        other[0] = 2.0f; other[1] = 3.0f; other[2] = 4.0f; other[3] = 5.0f;
        v *= other;
        REQUIRE_THAT(v[0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(v[3], WithinAbs(20.0f, 1e-6f));
    }

    SECTION("Component-wise division assignment") {
        VecN<4> other;
        other[0] = 2.0f; other[1] = 2.0f; other[2] = 3.0f; other[3] = 4.0f;
        v /= other;
        REQUIRE_THAT(v[0], WithinAbs(0.5f, 1e-6f));
        REQUIRE_THAT(v[3], WithinAbs(1.0f, 1e-6f));
    }
}

TEST_CASE("VecN: Component-wise operations", "[VecN][arithmetic]") {
    VecN<3> a;
    a[0] = 2.0f; a[1] = 3.0f; a[2] = 4.0f;
    VecN<3> b;
    b[0] = 5.0f; b[1] = 6.0f; b[2] = 7.0f;

    SECTION("Component-wise multiplication") {
        VecN<3> c = a * b;
        REQUIRE_THAT(c[0], WithinAbs(10.0f, 1e-6f));
        REQUIRE_THAT(c[2], WithinAbs(28.0f, 1e-6f));
    }

    SECTION("Component-wise division") {
        VecN<3> c = b / a;
        REQUIRE_THAT(c[0], WithinAbs(2.5f, 1e-6f));
        REQUIRE_THAT(c[1], WithinAbs(2.0f, 1e-6f));
    }
}

TEST_CASE("VecN: In-place normalize", "[VecN][normalization]") {
    VecN<3> v;
    v[0] = 2.0f; v[1] = 3.0f; v[2] = 6.0f;
    v.normalize();
    REQUIRE_THAT(v.length(), WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("VecN: Query functions", "[VecN][query]") {
    SECTION("isZero") {
        VecN<3> zero;
        REQUIRE(zero.isZero());
        zero[0] = 0.001f;
        REQUIRE(!zero.isZero());
    }

    SECTION("isNormalized") {
        VecN<3> unit;
        unit[0] = 1.0f;
        REQUIRE(unit.isNormalized());
        unit[0] = 2.0f;
        REQUIRE(!unit.isNormalized());
    }

    SECTION("approxEqual") {
        VecN<3> a;
        a[0] = 1.0f; a[1] = 2.0f; a[2] = 3.0f;
        VecN<3> b;
        b[0] = 1.00001f; b[1] = 2.00001f; b[2] = 3.00001f;
        VecN<3> c;
        c[0] = 1.1f; c[1] = 2.0f; c[2] = 3.0f;
        REQUIRE(a.approxEqual(b, 1e-4f));
        REQUIRE(!a.approxEqual(c, 1e-4f));
    }

    SECTION("abs") {
        VecN<3> v;
        v[0] = -3.0f; v[1] = 4.0f; v[2] = -5.0f;
        VecN<3> result = v.abs();
        REQUIRE_THAT(result[0], WithinAbs(3.0f, 1e-6f));
        REQUIRE_THAT(result[1], WithinAbs(4.0f, 1e-6f));
        REQUIRE_THAT(result[2], WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("VecN: Utility operations", "[VecN][utility]") {
    VecN<3> v;
    v[0] = 3.0f; v[1] = 4.0f; v[2] = 0.0f;

    SECTION("Distance") {
        VecN<3> other;
        REQUIRE_THAT(v.distance(other), WithinAbs(5.0f, 1e-6f));
    }

    SECTION("Angle") {
        VecN<3> a;
        a[0] = 1.0f;
        VecN<3> b;
        b[1] = 1.0f;
        REQUIRE_THAT(a.angle(b), WithinAbs(3.14159f / 2.0f, 1e-4f));
    }

    SECTION("Lerp") {
        VecN<3> a;
        VecN<3> b;
        b[0] = 10.0f; b[1] = 20.0f; b[2] = 30.0f;
        VecN<3> mid = a.lerp(b, 0.5f);
        REQUIRE_THAT(mid[0], WithinAbs(5.0f, 1e-6f));
        REQUIRE_THAT(mid[2], WithinAbs(15.0f, 1e-6f));
    }

    SECTION("Min/Max") {
        VecN<3> a;
        a[0] = 2.0f; a[1] = 8.0f; a[2] = 5.0f;
        VecN<3> b;
        b[0] = 5.0f; b[1] = 3.0f; b[2] = 7.0f;
        VecN<3> minV = a.min(b);
        VecN<3> maxV = a.max(b);
        REQUIRE_THAT(minV[0], WithinAbs(2.0f, 1e-6f));
        REQUIRE_THAT(maxV[0], WithinAbs(5.0f, 1e-6f));
    }
}

TEST_CASE("VecN: Edge cases", "[VecN][edge]") {
    SECTION("Division by zero scalar") {
        VecN<3> v;
        v[0] = 1.0f; v[1] = 2.0f; v[2] = 3.0f;
        VecN<3> result = v / 0.0f;
        REQUIRE((std::isinf(result[0]) || std::isnan(result[0])));
    }

    SECTION("Normalization of very small vector") {
        VecN<5> v;
        for (std::size_t i = 0; i < 5; ++i) v[i] = 1e-20f;
        VecN<5> n = v.normalized();
        REQUIRE_THAT(n.length(), WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Large dimension vector") {
        VecN<100> v(1.0f);
        REQUIRE(v.size() == 100);
        REQUIRE_THAT(v.length(), WithinAbs(10.0f, 1e-5f)); // sqrt(100)
    }
}

TEST_CASE("VecN: Stream output", "[VecN][stream]") {
    VecN<3> v;
    v[0] = 1.5f; v[1] = 2.5f; v[2] = 3.5f;
    std::ostringstream oss;
    oss << v;
    REQUIRE(oss.str() == "(1.5, 2.5, 3.5)");
}

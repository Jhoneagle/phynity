#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec_n.hpp>
#include <cmath>

using phynity::math::VecN;
using Catch::Matchers::WithinAbs;

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
}

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec_dynamic.hpp>
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
}

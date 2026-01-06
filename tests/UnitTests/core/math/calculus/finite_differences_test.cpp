#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/calculus/finite_differences.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <cmath>
#include <array>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using phynity::math::calculus::forward_difference_first;
using phynity::math::calculus::central_difference_first;
using phynity::math::calculus::forward_difference_second;
using phynity::math::calculus::central_difference_second;
using phynity::math::calculus::central_difference_vector;
using phynity::math::calculus::optimal_step_size_first_derivative;
using phynity::math::calculus::optimal_step_size_second_derivative;
using phynity::math::calculus::validate_derivative;
using phynity::math::calculus::numerical_jacobian;
using phynity::math::vectors::VecN;

TEST_CASE("Finite Differences: First derivative - linear function", "[calculus][finite_differences]") {
    // f(x) = 2x + 3, f'(x) = 2
    auto f = [](float x) { return 2.0f * x + 3.0f; };
    [[maybe_unused]] auto f_prime = []([[maybe_unused]] float x) { return 2.0f; };
    
    SECTION("Forward difference") {
        float deriv = forward_difference_first(f, 5.0f, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(2.0f, 5e-3f));
    }
    
    SECTION("Central difference") {
        float deriv = central_difference_first(f, 5.0f, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(2.0f, 5e-3f));
    }
    
    SECTION("Central is more accurate") {
        float forward = forward_difference_first(f, 5.0f, 1e-4f);
        float central = central_difference_first(f, 5.0f, 1e-4f);
        
        float forward_error = std::abs(forward - 2.0f);
        float central_error = std::abs(central - 2.0f);
        
        // Central difference should be at least as accurate as forward
        REQUIRE(central_error <= forward_error + 1e-6f);
    }
}

TEST_CASE("Finite Differences: First derivative - quadratic function", "[calculus][finite_differences]") {
    // f(x) = x² + 3x + 1, f'(x) = 2x + 3
    auto f = [](float x) { return x * x + 3.0f * x + 1.0f; };
    auto f_prime = [](float x) { return 2.0f * x + 3.0f; };
    
    float x = 2.0f;
    float expected = f_prime(x);  // 2*2 + 3 = 7
    
    SECTION("Forward difference") {
        float deriv = forward_difference_first(f, x, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(expected, 1e-2f));
    }
    
    SECTION("Central difference") {
        float deriv = central_difference_first(f, x, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(expected, 1e-2f));
    }
}

TEST_CASE("Finite Differences: First derivative - sine function", "[calculus][finite_differences]") {
    // f(x) = sin(x), f'(x) = cos(x)
    auto f = [](float x) { return std::sin(x); };
    auto f_prime = [](float x) { return std::cos(x); };
    
    float x = 1.0f;
    float expected = f_prime(x);
    
    SECTION("Forward difference at x=1") {
        float deriv = forward_difference_first(f, x, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(expected, 1e-2f));
    }
    
    SECTION("Central difference at x=1") {
        float deriv = central_difference_first(f, x, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(expected, 1e-2f));
    }
    
    SECTION("Central difference at multiple points") {
        for (float x_test = 0.0f; x_test <= 3.14f; x_test += 0.5f) {
            float deriv = central_difference_first(f, x_test, 1e-4f);
            float expected_val = f_prime(x_test);
            REQUIRE_THAT(deriv, WithinAbs(expected_val, 1e-2f));
        }
    }
}

TEST_CASE("Finite Differences: Second derivative - quadratic", "[calculus][finite_differences]") {
    // f(x) = x² - 3x + 2, f''(x) = 2
    auto f = [](float x) { return x * x - 3.0f * x + 2.0f; };
    
    SECTION("Forward difference second derivative") {
        float deriv2 = forward_difference_second(f, 5.0f, 1e-3f);
        REQUIRE_THAT(deriv2, WithinAbs(2.0f, 1.0f));
    }
    
    SECTION("Central difference second derivative") {
        float deriv2 = central_difference_second(f, 5.0f, 1e-3f);
        REQUIRE_THAT(deriv2, WithinAbs(2.0f, 1.0f));
    }
}

TEST_CASE("Finite Differences: Second derivative - sine function", "[calculus][finite_differences]") {
    // f(x) = sin(x), f''(x) = -sin(x)
    auto f = [](float x) { return std::sin(x); };
    
    float x = 1.0f;
    float expected = -std::sin(x);
    
    SECTION("Central difference second derivative") {
        float deriv2 = central_difference_second(f, x, 1e-3f);
        REQUIRE_THAT(deriv2, WithinAbs(expected, 1.0f));
    }
}

TEST_CASE("Finite Differences: Cubic polynomial", "[calculus][finite_differences]") {
    // f(x) = x³ + 2x² - 5x + 1
    // f'(x) = 3x² + 4x - 5
    // f''(x) = 6x + 4
    auto f = [](float x) { return x*x*x + 2*x*x - 5*x + 1; };
    auto f_prime = [](float x) { return 3*x*x + 4*x - 5; };
    auto f_prime2 = [](float x) { return 6*x + 4; };
    
    float x = 2.0f;
    
    SECTION("First derivative") {
        float deriv = central_difference_first(f, x, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(f_prime(x), 1e-1f));
    }
    
    SECTION("Second derivative") {
        float deriv2 = central_difference_second(f, x, 1e-3f);
        REQUIRE_THAT(deriv2, WithinAbs(f_prime2(x), 2.0f));
    }
}

TEST_CASE("Finite Differences: Vector-valued function", "[calculus][finite_differences]") {
    // f: R → R² where f(x) = [sin(x), cos(x)]
    // f'(x) = [cos(x), -sin(x)]
    auto f = [](float x) -> VecN<2, float> {
        return VecN<2, float>(std::array<float, 2>{std::sin(x), std::cos(x)});
    };
    
    float x = 1.0f;
    VecN<2, float> expected(std::array<float, 2>{std::cos(x), -std::sin(x)});
    
    VecN<2, float> deriv = central_difference_vector<2>(f, x, 1e-4f);
    
    REQUIRE_THAT(deriv[0], WithinAbs(expected[0], 1e-3f));
    REQUIRE_THAT(deriv[1], WithinAbs(expected[1], 1e-3f));
}

TEST_CASE("Finite Differences: Optimal step size", "[calculus][finite_differences]") {
    SECTION("First derivative step size at x=1") {
        float h = optimal_step_size_first_derivative(1.0f);
        REQUIRE(h > 0.0f);
        REQUIRE(h < 1.0f);  // Should be small
    }
    
    SECTION("Second derivative step size at x=1") {
        float h = optimal_step_size_second_derivative(1.0f);
        REQUIRE(h > 0.0f);
        REQUIRE(h < 1.0f);
    }
    
    SECTION("Step size scales with |x|") {
        float h1 = optimal_step_size_first_derivative(1.0f);
        float h10 = optimal_step_size_first_derivative(10.0f);
        
        // h should scale roughly linearly with |x|
        REQUIRE(h10 > h1);
    }
}

TEST_CASE("Finite Differences: Derivative validation", "[calculus][finite_differences]") {
    auto f = [](float x) { return x * x; };
    auto f_prime = [](float x) { return 2.0f * x; };
    
    SECTION("Valid derivative at x=1") {
        bool valid = validate_derivative(f, f_prime, 1.0f, 1e-4f, 1e-1f);
        REQUIRE(valid);
    }
    
    SECTION("Valid derivative at x=5") {
        bool valid = validate_derivative(f, f_prime, 5.0f, 1e-4f, 1e-1f);
        REQUIRE(valid);
    }
    
    SECTION("Invalid derivative (wrong function)") {
        auto f_prime_wrong = [](float x) { return 3.0f * x; };  // Wrong!
        bool valid = validate_derivative(f, f_prime_wrong, 1.0f, 1e-4f, 1e-1f);
        REQUIRE_FALSE(valid);
    }
}

TEST_CASE("Finite Differences: Jacobian for simple function", "[calculus][finite_differences]") {
    // f: R² → R² where f(x, y) = [x² + y, 2xy]
    // J = [[2x, 1], [2y, 2x]]
    auto f = [](const VecN<2, float>& v) {
        float x = v[0];
        float y = v[1];
        return VecN<2, float>(std::array<float, 2>{x*x + y, 2*x*y});
    };
    
    VecN<2, float> point(std::array<float, 2>{2.0f, 3.0f});
    auto jac = numerical_jacobian<2, 2>(f, point, 1e-4f);
    
    SECTION("Jacobian dimensions") {
        // MatN<2, 2> has compile-time dimensions, so this is implicit
        // Just verify we can access elements
        REQUIRE_THAT(jac(0, 0), WithinAbs(4.0f, 1e-1f));  // Check top-left element
    }
    
    SECTION("Jacobian values") {
        // At (2, 3): J = [[4, 1], [6, 4]]
        REQUIRE_THAT(jac(0, 0), WithinAbs(4.0f, 1e-1f));  // ∂f₀/∂x = 2x = 4
        REQUIRE_THAT(jac(0, 1), WithinAbs(1.0f, 1e-1f));  // ∂f₀/∂y = 1
        REQUIRE_THAT(jac(1, 0), WithinAbs(6.0f, 1e-1f));  // ∂f₁/∂x = 2y = 6
        REQUIRE_THAT(jac(1, 1), WithinAbs(4.0f, 1e-1f));  // ∂f₁/∂y = 2x = 4
    }
}

TEST_CASE("Finite Differences: Accuracy comparison forward vs central", "[calculus][finite_differences]") {
    auto f = [](float x) { return std::sin(x) * std::cos(x); };
    auto f_prime = [](float x) { 
        return std::cos(x) * std::cos(x) - std::sin(x) * std::sin(x);  // cos(2x)
    };
    
    float x = 0.5f;
    float expected = f_prime(x);
    
    float forward_error = std::abs(forward_difference_first(f, x, 1e-4f) - expected);
    float central_error = std::abs(central_difference_first(f, x, 1e-4f) - expected);
    
    // Both should be reasonably accurate
    REQUIRE(forward_error < 1e-3f);
    REQUIRE(central_error < 1e-3f);
}

TEST_CASE("Finite Differences: Extreme values", "[calculus][finite_differences]") {
    auto f = [](float x) { return x * x; };
    
    SECTION("Very small x") {
        float deriv = central_difference_first(f, 1e-6f, 1e-10f);
        REQUIRE(std::isfinite(deriv));
    }
    
    SECTION("Large x") {
        float deriv = central_difference_first(f, 1000.0f, 0.1f);
        REQUIRE_THAT(deriv, WithinAbs(2000.0f, 1.0f));
    }
}

TEST_CASE("Finite Differences: Exponential function", "[calculus][finite_differences]") {
    // f(x) = e^x, f'(x) = e^x
    auto f = [](float x) { return std::exp(x); };
    auto f_prime = [](float x) { return std::exp(x); };
    
    SECTION("At x=0") {
        float deriv = central_difference_first(f, 0.0f, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(f_prime(0.0f), 1e-3f));
    }
    
    SECTION("At x=1") {
        float deriv = central_difference_first(f, 1.0f, 1e-4f);
        REQUIRE_THAT(deriv, WithinAbs(f_prime(1.0f), 1e-2f));
    }
}

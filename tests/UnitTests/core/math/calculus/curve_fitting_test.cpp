#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <core/math/calculus/curve_fitting.hpp>
#include <cmath>

using namespace phynity::math::vectors;
using namespace phynity::math::calculus;

// ============================================================================
// Test linear polynomial fitting (degree 1)
// ============================================================================

TEST_CASE("Fit linear polynomial to exact data", "[curve_fitting]") {
    // y = 2*x + 1
    const int n = 5;
    const float x_vals[n] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    const float y_vals[n] = {1.0f, 3.0f, 5.0f, 7.0f, 9.0f};
    
    VecDynamic<float> x_data(static_cast<size_t>(n));
    VecDynamic<float> y_data(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        x_data[static_cast<size_t>(i)] = x_vals[i];
        y_data[static_cast<size_t>(i)] = y_vals[i];
    }
    
    auto coeffs = fit_polynomial(1, x_data, y_data);
    
    // Coefficients: [c0, c1] where y = c0 + c1*x = 1 + 2*x
    REQUIRE(coeffs.size() == 2);
    REQUIRE(coeffs[0] == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(coeffs[1] == Catch::Approx(2.0f).margin(1e-4f));
}

// ============================================================================
// Test quadratic polynomial fitting (degree 2)
// ============================================================================

TEST_CASE("Fit quadratic polynomial to exact data", "[curve_fitting]") {
    // y = x^2 + 2*x + 1
    const int n = 5;
    const float x_vals[n] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    const float y_vals[n] = {1.0f, 4.0f, 9.0f, 16.0f, 25.0f};
    
    VecDynamic<float> x_data(static_cast<size_t>(n));
    VecDynamic<float> y_data(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        x_data[static_cast<size_t>(i)] = x_vals[i];
        y_data[static_cast<size_t>(i)] = y_vals[i];
    }
    
    auto coeffs = fit_polynomial(2, x_data, y_data);
    
    REQUIRE(coeffs.size() == 3);
    REQUIRE(coeffs[0] == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(coeffs[1] == Catch::Approx(2.0f).margin(1e-4f));
    REQUIRE(coeffs[2] == Catch::Approx(1.0f).margin(1e-4f));
}

// ============================================================================
// Test cubic polynomial fitting (degree 3)
// ============================================================================

TEST_CASE("Fit cubic polynomial to exact data", "[curve_fitting]") {
    // y = x^3 + x^2 + x + 1
    const int n = 5;
    const float x_vals[n] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    const float y_vals[n] = {1.0f, 4.0f, 15.0f, 40.0f, 85.0f};
    
    VecDynamic<float> x_data(static_cast<size_t>(n));
    VecDynamic<float> y_data(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        x_data[static_cast<size_t>(i)] = x_vals[i];
        y_data[static_cast<size_t>(i)] = y_vals[i];
    }
    
    auto coeffs = fit_polynomial(3, x_data, y_data);
    
    REQUIRE(coeffs.size() == 4);
    REQUIRE(coeffs[0] == Catch::Approx(1.0f).margin(1e-3f));
    REQUIRE(coeffs[1] == Catch::Approx(1.0f).margin(1e-3f));
    REQUIRE(coeffs[2] == Catch::Approx(1.0f).margin(1e-3f));
    REQUIRE(coeffs[3] == Catch::Approx(1.0f).margin(1e-3f));
}

// ============================================================================
// Test polynomial evaluation
// ============================================================================

TEST_CASE("Evaluate polynomial at point", "[curve_fitting]") {
    // y = 2*x^2 + 3*x + 1
    VecDynamic<float> coeffs(3);
    coeffs[0] = 1.0f;  // c0
    coeffs[1] = 3.0f;  // c1
    coeffs[2] = 2.0f;  // c2
    
    float result = evaluate_polynomial(coeffs, 2.0f);
    // y(2) = 1 + 3*2 + 2*4 = 1 + 6 + 8 = 15
    REQUIRE(result == Catch::Approx(15.0f).margin(1e-4f));
}

// ============================================================================
// Test polynomial derivative computation
// ============================================================================

TEST_CASE("Compute polynomial derivative", "[curve_fitting]") {
    // y = x^3 + 2*x^2 + 3*x + 1
    // dy/dx = 3*x^2 + 4*x + 3
    VecDynamic<float> coeffs(4);
    coeffs[0] = 1.0f;  // c0
    coeffs[1] = 3.0f;  // c1
    coeffs[2] = 2.0f;  // c2
    coeffs[3] = 1.0f;  // c3
    
    float deriv_at_1 = polynomial_derivative(coeffs, 1.0f);
    // dy/dx(1) = 3 + 4 + 3 = 10
    REQUIRE(deriv_at_1 == Catch::Approx(10.0f).margin(1e-4f));
}

// ============================================================================
// Test error handling: insufficient data points
// ============================================================================

TEST_CASE("Fit polynomial with insufficient data", "[curve_fitting]") {
    // Try to fit degree 3 polynomial with only 2 points
    VecDynamic<float> x_data(2);
    VecDynamic<float> y_data(2);
    x_data[0] = 0.0f;
    x_data[1] = 1.0f;
    y_data[0] = 1.0f;
    y_data[1] = 2.0f;
    
    REQUIRE_THROWS(fit_polynomial(3, x_data, y_data));
}

// ============================================================================
// Test error handling: size mismatch
// ============================================================================

TEST_CASE("Fit polynomial with mismatched data sizes", "[curve_fitting]") {
    VecDynamic<float> x_data(3);
    VecDynamic<float> y_data(4);  // Different size
    
    REQUIRE_THROWS(fit_polynomial(1, x_data, y_data));
}

// ============================================================================
// Test derivative estimation from linear data
// ============================================================================

TEST_CASE("Estimate derivatives from linear data", "[curve_fitting]") {
    // y = 2*x + 1, so dy/dx = 2 everywhere
    const int n = 5;
    VecDynamic<float> x_data(static_cast<size_t>(n));
    VecDynamic<float> y_data(static_cast<size_t>(n));
    
    for (int i = 0; i < n; ++i) {
        float x = static_cast<float>(i);
        x_data[static_cast<size_t>(i)] = x;
        y_data[static_cast<size_t>(i)] = 2.0f * x + 1.0f;
    }
    
    auto derivs = estimate_derivatives_from_data(x_data, y_data, 1, 3);
    
    REQUIRE(derivs.size() == n);
    for (int i = 0; i < n; ++i) {
        REQUIRE(derivs[static_cast<size_t>(i)] == Catch::Approx(2.0f).margin(1e-3f));
    }
}

// ============================================================================
// Test derivative estimation from quadratic data
// ============================================================================

TEST_CASE("Estimate derivatives from quadratic data", "[curve_fitting]") {
    // y = x^2, so dy/dx = 2*x
    const int n = 5;
    VecDynamic<float> x_data(static_cast<size_t>(n));
    VecDynamic<float> y_data(static_cast<size_t>(n));
    
    for (int i = 0; i < n; ++i) {
        float x = static_cast<float>(i);
        x_data[static_cast<size_t>(i)] = x;
        y_data[static_cast<size_t>(i)] = x * x;
    }
    
    auto derivs = estimate_derivatives_from_data(x_data, y_data, 2, 5);
    
    REQUIRE(derivs.size() == n);
    for (int i = 0; i < n; ++i) {
        float expected = 2.0f * x_data[static_cast<size_t>(i)];
        REQUIRE(derivs[static_cast<size_t>(i)] == Catch::Approx(expected).margin(1e-2f));
    }
}

// ============================================================================
// Test error handling: invalid window size
// ============================================================================

TEST_CASE("Estimate derivatives with invalid window size", "[curve_fitting]") {
    VecDynamic<float> x_data(5);
    VecDynamic<float> y_data(5);
    
    // Even window size should throw
    REQUIRE_THROWS(estimate_derivatives_from_data(x_data, y_data, 2, 4));
    
    // Window size too small should throw
    REQUIRE_THROWS(estimate_derivatives_from_data(x_data, y_data, 2, 1));
}

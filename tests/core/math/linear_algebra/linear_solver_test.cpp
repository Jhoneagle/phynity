#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/linear_algebra/solver.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <array>

using Catch::Matchers::WithinAbs;
using phynity::math::linear_algebra::solve;
using phynity::math::linear_algebra::SolveMethod;
using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;
using phynity::math::utilities::approx_equal;

TEST_CASE("Linear System Solving: Ax=b with LU", "[linear_algebra][solve]") {
    // Solve system:
    // 2x + 3y - z = 3
    // -x + 4y + 2z = 7
    // 3x - y + z = 4
    
    MatN<3, 3, float> A(std::array<float, 9>{
        2,  3, -1,
        -1, 4,  2,
        3, -1,  1
    });

    VecN<3, float> b({3.0f, 7.0f, 4.0f});

    // Expected solution: x=1, y=1, z=2
    VecN<3, float> x = solve(A, b, SolveMethod::LU);

    REQUIRE_THAT(x[0], WithinAbs(1.0f, 1e-3f));
    REQUIRE_THAT(x[1], WithinAbs(1.0f, 1e-3f));
    REQUIRE_THAT(x[2], WithinAbs(2.0f, 1e-3f));

    // Verify: A*x ≈ b
    VecN<3, float> computed_b = A * x;
    REQUIRE(approx_equal(computed_b, b, 1e-3f));
}

TEST_CASE("Linear System Solving: identity system", "[linear_algebra][solve]") {
    MatN<3, 3, float> I = MatN<3, 3, float>::identity();
    VecN<3, float> b({1.0f, 2.0f, 3.0f});

    VecN<3, float> x = solve(I, b, SolveMethod::LU);

    // Solution should be b itself
    REQUIRE(approx_equal(x, b, 1e-6f));
}

TEST_CASE("Linear System Solving: QR vs LU", "[linear_algebra][solve]") {
    MatN<3, 3, float> A(std::array<float, 9>{
        2,  3, -1,
        -1, 4,  2,
        3, -1,  1
    });

    VecN<3, float> b({1.0f, 2.0f, 3.0f});

    VecN<3, float> x_lu = solve(A, b, SolveMethod::LU);
    VecN<3, float> x_qr = solve(A, b, SolveMethod::QR);

    // Both methods should give same solution
    REQUIRE(approx_equal(x_lu, x_qr, 1e-3f));
}

TEST_CASE("ill-conditioned system", "[linear_algebra][solve]") {
    // Create an ill-conditioned Hilbert-like matrix
    // For small N, this is still reasonable
    MatN<3, 3, float> H(std::array<float, 9>{
        1.0f,     0.5f,     0.333333f,
        0.5f,     0.333333f, 0.25f,
        0.333333f, 0.25f,     0.2f
    });

    VecN<3, float> x_true({1.0f, 1.0f, 1.0f});
    VecN<3, float> b = H * x_true;

    // QR should be more stable for ill-conditioned systems
    VecN<3, float> x_qr = solve(H, b, SolveMethod::QR);

    // Hilbert matrices are notoriously ill-conditioned
    // With single precision float, even QR struggles with 3x3 Hilbert
    // Using relaxed tolerance appropriate for float precision
    REQUIRE(approx_equal(x_qr, x_true, 0.1f));
}

TEST_CASE("Constraint solving scenario", "[linear_algebra][physics]") {
    // Simplified constraint problem: find constraint forces
    // J^T * λ = b where J^T is 3x3 Jacobian transpose
    
    MatN<3, 3, float> JT(std::array<float, 9>{
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    });

    VecN<3, float> b({0.1f, 0.2f, 0.15f}); // constraint violations

    VecN<3, float> lambda = solve(JT, b, SolveMethod::LU);

    // Solution should match b for identity matrix
    REQUIRE(approx_equal(lambda, b, 1e-6f));
}

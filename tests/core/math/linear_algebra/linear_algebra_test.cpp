#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/linear_algebra/lu_decomposition.hpp>
#include <core/math/linear_algebra/qr_decomposition.hpp>
#include <core/math/linear_algebra/solver.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <array>
#include <cmath>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using phynity::math::linear_algebra::LUDecomposition;
using phynity::math::linear_algebra::QRDecomposition;
using phynity::math::linear_algebra::solve;
using phynity::math::linear_algebra::inverse;
using phynity::math::linear_algebra::determinant;
using phynity::math::linear_algebra::SolveMethod;
using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;
using phynity::math::utilities::approx_equal;

TEST_CASE("LU Decomposition: 2x2 matrix", "[linear_algebra][lu]") {
    // A = [[4, 3], [6, 3]]
    MatN<2, 2, float> A(std::array<float, 4>{4, 3, 6, 3});
    LUDecomposition<2, float> lu(A);

    REQUIRE_FALSE(lu.is_singular);

    // Check that L*U ≈ P*A
    MatN<2, 2, float> L = lu.getL();
    MatN<2, 2, float> U = lu.getU();
    MatN<2, 2, float> P = lu.getP();
    MatN<2, 2, float> reconstructed = L * U;

    // Permuted A
    MatN<2, 2, float> PA = P * A;

    SECTION("L is lower triangular") {
        REQUIRE_THAT(L(0, 1), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("U is upper triangular") {
        REQUIRE_THAT(U(1, 0), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("LU reconstruction") {
        REQUIRE_THAT(reconstructed(0, 0), WithinRel(PA(0, 0), 1e-4f));
        REQUIRE_THAT(reconstructed(1, 1), WithinRel(PA(1, 1), 1e-4f));
    }

    SECTION("Determinant") {
        float det = lu.determinant();
        // det(A) = 4*3 - 3*6 = 12 - 18 = -6
        REQUIRE_THAT(det, WithinAbs(-6.0f, 1e-3f));
    }
}

TEST_CASE("LU Decomposition: 3x3 identity", "[linear_algebra][lu]") {
    MatN<3, 3, float> I = MatN<3, 3, float>::identity();
    LUDecomposition<3, float> lu(I);

    REQUIRE_FALSE(lu.is_singular);
    REQUIRE_THAT(lu.determinant(), WithinAbs(1.0f, 1e-6f));

    MatN<3, 3, float> L = lu.getL();
    MatN<3, 3, float> U = lu.getU();

    // For identity: L should be I and U should be I
    REQUIRE(approx_equal(L, MatN<3, 3, float>::identity(), 1e-6f));
    REQUIRE(approx_equal(U, MatN<3, 3, float>::identity(), 1e-6f));
}

TEST_CASE("LU Decomposition: diagonal matrix", "[linear_algebra][lu]") {
    // Diagonal matrix with known determinant
    MatN<3, 3, float> D(std::array<float, 9>{
        2, 0, 0,
        0, 3, 0,
        0, 0, 5
    });

    LUDecomposition<3, float> lu(D);

    REQUIRE_FALSE(lu.is_singular);
    // det = 2*3*5 = 30
    REQUIRE_THAT(lu.determinant(), WithinAbs(30.0f, 1e-3f));
}

TEST_CASE("LU Decomposition: singular matrix detection", "[linear_algebra][lu]") {
    // Singular matrix (rows are linearly dependent)
    MatN<3, 3, float> singular(std::array<float, 9>{
        1, 2, 3,
        2, 4, 6,  // 2 * row 0
        3, 6, 9   // 3 * row 0
    });

    LUDecomposition<3, float> lu(singular);

    REQUIRE(lu.is_singular);
    REQUIRE_THAT(lu.determinant(), WithinAbs(0.0f, 1e-6f));
}

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

TEST_CASE("Matrix Inverse", "[linear_algebra][inverse]") {
    MatN<3, 3, float> A(std::array<float, 9>{
        1, 2, 3,
        0, 1, 4,
        5, 6, 0
    });

    MatN<3, 3, float> A_inv = inverse(A);

    // A * A^-1 should be identity
    MatN<3, 3, float> product = A * A_inv;
    MatN<3, 3, float> I = MatN<3, 3, float>::identity();

    REQUIRE(approx_equal(product, I, 1e-3f));
}

TEST_CASE("Matrix Inverse: singular matrix", "[linear_algebra][inverse]") {
    MatN<3, 3, float> singular(std::array<float, 9>{
        1, 2, 3,
        2, 4, 6,
        3, 6, 9
    });

    MatN<3, 3, float> inv = inverse(singular);
    MatN<3, 3, float> zero = MatN<3, 3, float>::zero();

    // Inverse of singular should be zero
    REQUIRE(approx_equal(inv, zero, 1e-6f));
}

TEST_CASE("Determinant Computation", "[linear_algebra][determinant]") {
    SECTION("2x2 determinant") {
        MatN<2, 2, float> A(std::array<float, 4>{1, 2, 3, 4});
        // det = 1*4 - 2*3 = -2
        float det = determinant(A);
        REQUIRE_THAT(det, WithinAbs(-2.0f, 1e-3f));
    }

    SECTION("3x3 determinant") {
        MatN<3, 3, float> A(std::array<float, 9>{
            1, 2, 3,
            0, 1, 4,
            5, 6, 0
        });
        // det = 1*(0-24) - 2*(0-20) + 3*(0-5) = -24 + 40 - 15 = 1
        float det = determinant(A);
        REQUIRE_THAT(det, WithinAbs(1.0f, 1e-3f));
    }
}

TEST_CASE("QR Decomposition: 3x3 matrix", "[linear_algebra][qr]") {
    MatN<3, 3, float> A(std::array<float, 9>{
        1, 0, 0,
        1, 1, 0,
        1, 1, 1
    });

    QRDecomposition<3, float> qr(A);

    REQUIRE_FALSE(qr.is_singular);

    // Check orthogonality: Q^T * Q ≈ I
    MatN<3, 3, float> QT = qr.getQT();
    MatN<3, 3, float> should_be_I = QT * qr.Q;
    MatN<3, 3, float> I = MatN<3, 3, float>::identity();

    REQUIRE(approx_equal(should_be_I, I, 1e-4f));
}

TEST_CASE("QR Decomposition: Reconstruction", "[linear_algebra][qr]") {
    MatN<3, 3, float> A(std::array<float, 9>{
        2, 1, 0,
        0, 2, 1,
        0, 0, 3
    });

    QRDecomposition<3, float> qr(A);

    // Q*R should approximate A
    MatN<3, 3, float> reconstructed = qr.Q * qr.R;
    REQUIRE(approx_equal(reconstructed, A, 1e-4f));
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

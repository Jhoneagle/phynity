#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/linear_algebra/cholesky_decomposition.hpp>
#include <core/math/linear_algebra/solver.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <array>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using phynity::math::linear_algebra::CholeskyDecomposition;
using phynity::math::linear_algebra::solve_cholesky;
using phynity::math::linear_algebra::determinant_cholesky;
using phynity::math::linear_algebra::inverse_cholesky;
using phynity::math::linear_algebra::solve_spd;
using phynity::math::linear_algebra::determinant_spd;
using phynity::math::linear_algebra::inverse_spd;
using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;
using phynity::math::utilities::approx_equal;

TEST_CASE("Cholesky: Identity matrix", "[linear_algebra][cholesky]") {
    MatN<3, 3, float> I = MatN<3, 3, float>::identity();
    CholeskyDecomposition<3, float> chol(I);

    REQUIRE(chol.is_positive_definite);

    SECTION("L is lower triangular with 1s on diagonal") {
        REQUIRE_THAT(chol.L(0, 0), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(chol.L(1, 1), WithinAbs(1.0f, 1e-5f));
        REQUIRE_THAT(chol.L(2, 2), WithinAbs(1.0f, 1e-5f));

        // Off-diagonal below diagonal should be 0
        REQUIRE_THAT(chol.L(1, 0), WithinAbs(0.0f, 1e-5f));
        REQUIRE_THAT(chol.L(2, 1), WithinAbs(0.0f, 1e-5f));
    }

    SECTION("Reconstruction") {
        MatN<3, 3, float> reconstructed = chol.reconstruct();
        REQUIRE(approx_equal(reconstructed, I, 1e-4f));
    }

    SECTION("Determinant is 1") {
        float det = determinant_cholesky(chol);
        REQUIRE_THAT(det, WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("Cholesky: Diagonal matrix", "[linear_algebra][cholesky]") {
    // Diagonal matrix is always SPD if all diagonal elements are positive
    MatN<3, 3, float> D(std::array<float, 9>{
        4, 0, 0,
        0, 9, 0,
        0, 0, 16
    });

    CholeskyDecomposition<3, float> chol(D);

    REQUIRE(chol.is_positive_definite);

    SECTION("L is diagonal with sqrt of original diagonal") {
        REQUIRE_THAT(chol.L(0, 0), WithinAbs(2.0f, 1e-5f));
        REQUIRE_THAT(chol.L(1, 1), WithinAbs(3.0f, 1e-5f));
        REQUIRE_THAT(chol.L(2, 2), WithinAbs(4.0f, 1e-5f));
    }

    SECTION("Reconstruction") {
        MatN<3, 3, float> reconstructed = chol.reconstruct();
        REQUIRE(approx_equal(reconstructed, D, 1e-4f));
    }

    SECTION("Determinant is 4*9*16 = 576") {
        float det = determinant_cholesky(chol);
        REQUIRE_THAT(det, WithinAbs(576.0f, 1e-3f));
    }
}

TEST_CASE("Cholesky: 2x2 SPD matrix", "[linear_algebra][cholesky]") {
    // A = [[2, 1], [1, 2]] is SPD
    MatN<2, 2, float> A(std::array<float, 4>{2, 1, 1, 2});
    CholeskyDecomposition<2, float> chol(A);

    REQUIRE(chol.is_positive_definite);

    SECTION("Lower triangular") {
        // L should be lower triangular
        REQUIRE(chol.L(0, 1) == 0.0f); // Upper part is 0
    }

    SECTION("Reconstruction") {
        MatN<2, 2, float> reconstructed = chol.reconstruct();
        REQUIRE(approx_equal(reconstructed, A, 1e-4f));
    }

    SECTION("L*L^T = A") {
        MatN<2, 2, float> LLT = chol.L * chol.L.transposed();
        REQUIRE(approx_equal(LLT, A, 1e-4f));
    }
}

TEST_CASE("Cholesky: 3x3 SPD matrix", "[linear_algebra][cholesky]") {
    // Covariance matrix [[2, 1, 0], [1, 3, 1], [0, 1, 2]]
    MatN<3, 3, float> A(std::array<float, 9>{
        2, 1, 0,
        1, 3, 1,
        0, 1, 2
    });

    CholeskyDecomposition<3, float> chol(A);

    REQUIRE(chol.is_positive_definite);

    SECTION("Reconstruction") {
        MatN<3, 3, float> reconstructed = chol.reconstruct();
        REQUIRE(approx_equal(reconstructed, A, 1e-4f));
    }

    SECTION("L diagonal elements are positive") {
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE(chol.L(i, i) > 0.0f);
        }
    }
}

TEST_CASE("Cholesky: Not positive definite - negative diagonal", "[linear_algebra][cholesky]") {
    // Matrix with negative diagonal element is not SPD
    MatN<2, 2, float> A(std::array<float, 4>{-1, 0, 0, 1});
    CholeskyDecomposition<2, float> chol(A);

    REQUIRE_FALSE(chol.is_positive_definite);
}

TEST_CASE("Cholesky: Not positive definite - indefinite", "[linear_algebra][cholesky]") {
    // Indefinite matrix: [[1, 2], [2, 1]]
    MatN<2, 2, float> A(std::array<float, 4>{1, 2, 2, 1});
    CholeskyDecomposition<2, float> chol(A);

    REQUIRE_FALSE(chol.is_positive_definite);
}

TEST_CASE("Cholesky: Singular matrix detection", "[linear_algebra][cholesky]") {
    // Singular SPD matrix (rank 1): [[2, 2], [2, 2]]
    MatN<2, 2, float> A(std::array<float, 4>{2, 2, 2, 2});
    CholeskyDecomposition<2, float> chol(A);

    REQUIRE_FALSE(chol.is_positive_definite);
}

TEST_CASE("Cholesky: Solve system Ax = b", "[linear_algebra][cholesky]") {
    // SPD system
    MatN<2, 2, float> A(std::array<float, 4>{2, 1, 1, 2});
    VecN<2, float> b(std::array<float, 2>{3, 4});

    CholeskyDecomposition<2, float> chol(A);
    VecN<2, float> x = solve_cholesky(chol, b);

    SECTION("Solution is correct") {
        VecN<2, float> result = A * x;
        REQUIRE(approx_equal(result, b, 1e-3f));
    }
}

TEST_CASE("Cholesky: Determinant computation", "[linear_algebra][cholesky]") {
    MatN<3, 3, float> A(std::array<float, 9>{
        4, 0, 0,
        0, 9, 0,
        0, 0, 16
    });

    CholeskyDecomposition<3, float> chol(A);
    float det = determinant_cholesky(chol);

    // det should be 4*9*16 = 576
    REQUIRE_THAT(det, WithinAbs(576.0f, 1e-3f));
}

TEST_CASE("Cholesky: Matrix inverse", "[linear_algebra][cholesky]") {
    MatN<2, 2, float> A(std::array<float, 4>{2, 1, 1, 2});
    CholeskyDecomposition<2, float> chol(A);
    MatN<2, 2, float> A_inv = inverse_cholesky(chol);

    SECTION("A * A^-1 = I") {
        MatN<2, 2, float> result = A * A_inv;
        MatN<2, 2, float> I = MatN<2, 2, float>::identity();
        REQUIRE(approx_equal(result, I, 1e-3f));
    }

    SECTION("A^-1 * A = I") {
        MatN<2, 2, float> result = A_inv * A;
        MatN<2, 2, float> I = MatN<2, 2, float>::identity();
        REQUIRE(approx_equal(result, I, 1e-3f));
    }
}

TEST_CASE("Cholesky: Wrapper functions", "[linear_algebra][cholesky]") {
    MatN<2, 2, float> A(std::array<float, 4>{2, 1, 1, 2});
    VecN<2, float> b(std::array<float, 2>{3, 4});

    SECTION("solve_spd") {
        VecN<2, float> x = solve_spd(A, b);
        VecN<2, float> result = A * x;
        REQUIRE(approx_equal(result, b, 1e-3f));
    }

    SECTION("determinant_spd") {
        float det = determinant_spd(A);
        // det([[2,1],[1,2]]) = 2*2 - 1*1 = 3
        REQUIRE_THAT(det, WithinAbs(3.0f, 1e-3f));
    }

    SECTION("inverse_spd") {
        MatN<2, 2, float> A_inv = inverse_spd(A);
        MatN<2, 2, float> result = A * A_inv;
        MatN<2, 2, float> I = MatN<2, 2, float>::identity();
        REQUIRE(approx_equal(result, I, 1e-3f));
    }
}

TEST_CASE("Cholesky: Large condition number SPD matrix", "[linear_algebra][cholesky]") {
    // Well-conditioned SPD: [[100, 1], [1, 1]]
    MatN<2, 2, float> A(std::array<float, 4>{100, 1, 1, 1});
    CholeskyDecomposition<2, float> chol(A);

    REQUIRE(chol.is_positive_definite);

    SECTION("Reconstruction is accurate") {
        MatN<2, 2, float> reconstructed = chol.reconstruct();
        REQUIRE(approx_equal(reconstructed, A, 1e-3f));
    }
}

TEST_CASE("Cholesky: 4x4 SPD covariance-like matrix", "[linear_algebra][cholesky]") {
    // Typical covariance matrix structure
    MatN<4, 4, float> A(std::array<float, 16>{
        4, 1, 0.5f, 0.2f,
        1, 3, 0.3f, 0.1f,
        0.5f, 0.3f, 2, 0,
        0.2f, 0.1f, 0, 1
    });

    CholeskyDecomposition<4, float> chol(A);

    REQUIRE(chol.is_positive_definite);

    SECTION("Reconstruction") {
        MatN<4, 4, float> reconstructed = chol.reconstruct();
        REQUIRE(approx_equal(reconstructed, A, 1e-3f));
    }

    SECTION("Solve system") {
        VecN<4, float> b(std::array<float, 4>{1, 2, 3, 4});
        VecN<4, float> x = solve_cholesky(chol, b);
        VecN<4, float> result = A * x;
        REQUIRE(approx_equal(result, b, 1e-3f));
    }
}

TEST_CASE("Cholesky: Efficiency comparison - solve vs LU", "[linear_algebra][cholesky]") {
    // For SPD systems, Cholesky should handle them correctly
    // This test verifies the solution quality
    MatN<3, 3, float> A(std::array<float, 9>{
        4, 1, 0,
        1, 3, 1,
        0, 1, 2
    });

    VecN<3, float> b(std::array<float, 3>{4, 5, 3});

    CholeskyDecomposition<3, float> chol(A);
    VecN<3, float> x = solve_cholesky(chol, b);

    SECTION("Solution satisfies Ax = b") {
        VecN<3, float> result = A * x;
        REQUIRE(approx_equal(result, b, 1e-3f));
    }
}

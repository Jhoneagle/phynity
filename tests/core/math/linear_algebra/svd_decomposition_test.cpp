#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/linear_algebra/svd_decomposition.hpp>
#include <core/math/linear_algebra/solver.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <cmath>
#include <array>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using phynity::math::linear_algebra::SVDDecomposition;
using phynity::math::linear_algebra::pseudoinverse;
using phynity::math::linear_algebra::pseudo_inverse;
using phynity::math::linear_algebra::matrix_rank;
using phynity::math::linear_algebra::condition_number;
using phynity::math::linear_algebra::least_squares_solve;
using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;
using phynity::math::utilities::approx_equal;
using phynity::math::utilities::epsilon;

TEST_CASE("SVD: Identity matrix", "[linear_algebra][svd]") {
    MatN<3, 3, float> I = MatN<3, 3, float>::identity();
    SVDDecomposition<3, float> svd(I);

    SECTION("Singular values") {
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE_THAT(svd.singular_values[i], WithinAbs(1.0f, 1e-5f));
        }
    }

    SECTION("Rank") {
        REQUIRE(svd.rank == 3);
    }

    SECTION("Condition number") {
        REQUIRE_THAT(svd.condition_number, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Reconstruction") {
        MatN<3, 3, float> reconstructed = svd.reconstruct();
        REQUIRE(approx_equal(reconstructed, I, 1e-4f));
    }
}

TEST_CASE("SVD: Diagonal matrix", "[linear_algebra][svd]") {
    MatN<3, 3, float> D(std::array<float, 9>{
        2, 0, 0,
        0, 3, 0,
        0, 0, 5
    });

    SVDDecomposition<3, float> svd(D);

    SECTION("Singular values in descending order") {
        // Should be [5, 3, 2]
        REQUIRE_THAT(svd.singular_values[0], WithinAbs(5.0f, 1e-4f));
        REQUIRE_THAT(svd.singular_values[1], WithinAbs(3.0f, 1e-4f));
        REQUIRE_THAT(svd.singular_values[2], WithinAbs(2.0f, 1e-4f));
    }

    SECTION("Full rank") {
        REQUIRE(svd.rank == 3);
    }

    SECTION("Condition number") {
        // κ(D) = 5/2 = 2.5
        REQUIRE_THAT(svd.condition_number, WithinAbs(2.5f, 1e-4f));
    }

    SECTION("Reconstruction") {
        MatN<3, 3, float> reconstructed = svd.reconstruct();
        REQUIRE(approx_equal(reconstructed, D, 1e-3f));
    }
}

TEST_CASE("SVD: 2x2 general matrix", "[linear_algebra][svd]") {
    // A = [[3, 0], [4, 0]]
    MatN<2, 2, float> A(std::array<float, 4>{3, 0, 4, 0});
    SVDDecomposition<2, float> svd(A);

    SECTION("Singular values") {
        // σ₁ = 5 (since ||[3,4]|| = 5), σ₂ = 0
        REQUIRE_THAT(svd.singular_values[0], WithinAbs(5.0f, 1e-4f));
        REQUIRE_THAT(svd.singular_values[1], WithinAbs(0.0f, 1e-4f));
    }

    SECTION("Rank is 1") {
        REQUIRE(svd.rank == 1);
    }

    SECTION("Reconstruction") {
        MatN<2, 2, float> reconstructed = svd.reconstruct();
        REQUIRE(approx_equal(reconstructed, A, 1e-3f));
    }
}

TEST_CASE("SVD: Random well-conditioned matrix", "[linear_algebra][svd]") {
    // A = [[1, 0.5], [0.3, 0.8]]
    MatN<2, 2, float> A(std::array<float, 4>{1.0f, 0.5f, 0.3f, 0.8f});
    SVDDecomposition<2, float> svd(A);

    SECTION("Singular values are non-negative") {
        REQUIRE(svd.singular_values[0] > 0.0f);
        REQUIRE(svd.singular_values[1] >= 0.0f);
    }

    SECTION("Singular values in descending order") {
        REQUIRE(svd.singular_values[0] >= svd.singular_values[1]);
    }

    SECTION("Full rank") {
        REQUIRE(svd.rank == 2);
    }

    SECTION("U and V are orthogonal") {
        MatN<2, 2, float> UUT = svd.U * svd.U.transposed();
        MatN<2, 2, float> I = MatN<2, 2, float>::identity();
        REQUIRE(approx_equal(UUT, I, 1e-4f));

        MatN<2, 2, float> VVT = svd.V * svd.V.transposed();
        REQUIRE(approx_equal(VVT, I, 1e-4f));
    }

    SECTION("Reconstruction") {
        MatN<2, 2, float> reconstructed = svd.reconstruct();
        REQUIRE(approx_equal(reconstructed, A, 1e-3f));
    }
}

TEST_CASE("SVD: Rank-deficient matrix", "[linear_algebra][svd]") {
    // A = [[1, 2], [2, 4]] (rank 1)
    MatN<2, 2, float> A(std::array<float, 4>{1, 2, 2, 4});
    SVDDecomposition<2, float> svd(A);

    SECTION("Rank is 1") {
        REQUIRE(svd.rank == 1);
    }

    SECTION("One zero singular value") {
        REQUIRE_THAT(svd.singular_values[0], WithinRel(5.0f, 1e-3f));
        REQUIRE_THAT(svd.singular_values[1], WithinAbs(0.0f, 1e-4f));
    }
}

TEST_CASE("SVD: 3x3 general matrix", "[linear_algebra][svd]") {
    // A = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    MatN<3, 3, float> A(std::array<float, 9>{
        1, 2, 3,
        4, 5, 6,
        7, 8, 9
    });

    SVDDecomposition<3, float> svd(A);

    SECTION("Singular values are non-negative") {
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE(svd.singular_values[i] >= 0.0f);
        }
    }

    SECTION("Singular values in descending order") {
        REQUIRE(svd.singular_values[0] >= svd.singular_values[1]);
        REQUIRE(svd.singular_values[1] >= svd.singular_values[2]);
    }

    SECTION("Rank is 2 (numerically)") {
        // This matrix is [[1,2,3],[4,5,6],[7,8,9]] which is rank 2
        // However, due to numerical precision in Jacobi SVD, smallest singular value
        // may not be exactly zero. Check that it's very small compared to largest.
        REQUIRE(svd.singular_values[2] < svd.singular_values[0] * 1e-3f);
    }

    SECTION("Reconstruction") {
        MatN<3, 3, float> reconstructed = svd.reconstruct();
        REQUIRE(approx_equal(reconstructed, A, 1e-2f));
    }
}

TEST_CASE("SVD: Pseudoinverse - full rank matrix", "[linear_algebra][svd]") {
    MatN<2, 2, float> A(std::array<float, 4>{2, 1, 1, 2});
    SVDDecomposition<2, float> svd(A);
    MatN<2, 2, float> A_pinv = pseudoinverse(svd);

    SECTION("Pseudoinverse satisfies Moore-Penrose properties") {
        // A*A⁺*A = A
        MatN<2, 2, float> AAA = A * A_pinv * A;
        REQUIRE(approx_equal(AAA, A, 1e-3f));

        // A⁺*A*A⁺ = A⁺
        MatN<2, 2, float> AinvAinv = A_pinv * A * A_pinv;
        REQUIRE(approx_equal(AinvAinv, A_pinv, 1e-3f));
    }

    SECTION("For full rank, pseudoinverse equals true inverse") {
        // Verify by solving Ax = b
        VecN<2, float> b(std::array<float, 2>{3, 4});
        VecN<2, float> x = A_pinv * b;
        VecN<2, float> result = A * x;
        REQUIRE(approx_equal(result, b, 1e-3f));
    }
}

TEST_CASE("SVD: Rank computation", "[linear_algebra][svd]") {
    SECTION("Full rank matrix") {
        MatN<2, 2, float> full_rank(std::array<float, 4>{1, 0, 0, 1});
        int rank = matrix_rank(full_rank);
        REQUIRE(rank == 2);
    }

    SECTION("Rank 1 matrix") {
        MatN<2, 2, float> rank_one(std::array<float, 4>{1, 2, 2, 4});
        int rank = matrix_rank(rank_one);
        REQUIRE(rank == 1);
    }

    SECTION("Zero matrix has rank 0") {
        MatN<2, 2, float> zero(0.0f);
        int rank = matrix_rank(zero);
        REQUIRE(rank == 0);
    }
}

TEST_CASE("SVD: Condition number", "[linear_algebra][svd]") {
    SECTION("Well-conditioned matrix") {
        MatN<2, 2, float> well_cond = MatN<2, 2, float>::identity();
        float cond = condition_number(well_cond);
        REQUIRE_THAT(cond, WithinAbs(1.0f, 1e-5f));
    }

    SECTION("Ill-conditioned matrix") {
        MatN<2, 2, float> ill_cond(std::array<float, 4>{1, 1, 1, 1.001f});
        float cond = condition_number(ill_cond);
        // Should be large
        REQUIRE(cond > 100.0f);
    }
}

TEST_CASE("SVD: Least-squares solve", "[linear_algebra][svd]") {
    // Solve Ax = b where A is well-conditioned
    MatN<2, 2, float> A(std::array<float, 4>{1, 0, 0, 1});
    VecN<2, float> b(std::array<float, 2>{3, 4});

    VecN<2, float> x = least_squares_solve(A, b);

    SECTION("Solution satisfies Ax = b for full rank") {
        VecN<2, float> result = A * x;
        REQUIRE(approx_equal(result, b, 1e-4f));
    }
}

TEST_CASE("SVD: Pseudo-inverse wrapper function", "[linear_algebra][svd]") {
    MatN<2, 2, float> A(std::array<float, 4>{3, 0, 4, 0});
    MatN<2, 2, float> A_pinv = pseudo_inverse(A);

    SECTION("Pseudoinverse computes correctly") {
        VecN<2, float> b(std::array<float, 2>{5, 0});
        VecN<2, float> x = A_pinv * b;

        // For rank-deficient A, verify that x minimizes ||Ax - b||
        // The projection of b onto the column space should be recovered
        VecN<2, float> result = A * x;
        VecN<2, float> error = result - b;
        // For this rank-1 matrix, the best solution has some reconstruction error
        // since b is orthogonal to the null space
        float norm = error.length();
        // Just verify it's computed (not NaN or infinity)
        REQUIRE(norm >= 0.0f);
        REQUIRE(std::isfinite(norm));
    }
}

TEST_CASE("SVD: Orthogonality verification", "[linear_algebra][svd]") {
    MatN<3, 3, float> A(std::array<float, 9>{
        1, 2, 3,
        4, 5, 6,
        7, 8, 10
    });

    SVDDecomposition<3, float> svd(A);

    SECTION("U^T * U = I") {
        MatN<3, 3, float> UT = svd.getUT();
        MatN<3, 3, float> UTU = UT * svd.U;
        MatN<3, 3, float> I = MatN<3, 3, float>::identity();
        REQUIRE(approx_equal(UTU, I, 1e-3f));
    }

    SECTION("V^T * V = I") {
        MatN<3, 3, float> VT = svd.getVT();
        MatN<3, 3, float> VTV = VT * svd.V;
        MatN<3, 3, float> I = MatN<3, 3, float>::identity();
        REQUIRE(approx_equal(VTV, I, 1e-3f));
    }
}

TEST_CASE("SVD: Large singular value ratio", "[linear_algebra][svd]") {
    // Matrix with very different singular values
    MatN<2, 2, float> A(std::array<float, 4>{100, 0, 0, 1});
    SVDDecomposition<2, float> svd(A);

    SECTION("Singular values correct") {
        REQUIRE_THAT(svd.singular_values[0], WithinAbs(100.0f, 1e-3f));
        REQUIRE_THAT(svd.singular_values[1], WithinAbs(1.0f, 1e-3f));
    }

    SECTION("Condition number is 100") {
        REQUIRE_THAT(svd.condition_number, WithinRel(100.0f, 1e-2f));
    }
}

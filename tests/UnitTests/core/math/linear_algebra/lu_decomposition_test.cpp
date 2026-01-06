#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/linear_algebra/lu_decomposition.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <array>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using phynity::math::linear_algebra::LUDecomposition;
using phynity::math::matrices::MatN;
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

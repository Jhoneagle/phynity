#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/linear_algebra/qr_decomposition.hpp>
#include <core/math/utilities/comparison_utils.hpp>
#include <array>

using Catch::Matchers::WithinAbs;
using phynity::math::linear_algebra::QRDecomposition;
using phynity::math::matrices::MatN;
using phynity::math::utilities::approx_equal;

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

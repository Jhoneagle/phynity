#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/fluids/sph_kernels.hpp>

#include <cmath>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using phynity::math::vectors::Vec3f;
using phynity::physics::constants::PI;
using phynity::physics::fluids::poly6;
using phynity::physics::fluids::poly6_gradient;
using phynity::physics::fluids::poly6_laplacian;
using phynity::physics::fluids::spiky_gradient;
using phynity::physics::fluids::viscosity_laplacian;

// ============================================================================
// poly6 density kernel
// ============================================================================

TEST_CASE("poly6: value at r=0 matches closed form", "[fluids][kernels][poly6]")
{
    const float h = 1.0f;
    // W(0,h) = 315/(64 π h⁹) · (h²)³ = 315/(64 π h³)
    const float expected = 315.0f / (64.0f * PI * h * h * h);
    REQUIRE_THAT(poly6(0.0f, h), WithinRel(expected, 1e-5f));
}

TEST_CASE("poly6: vanishes at and beyond the support radius", "[fluids][kernels][poly6]")
{
    const float h = 2.0f;
    REQUIRE_THAT(poly6(h * h, h), WithinAbs(0.0f, 1e-6f));       // r = h
    REQUIRE_THAT(poly6(h * h * 1.5f, h), WithinAbs(0.0f, 1e-6f)); // r > h
}

TEST_CASE("poly6: strictly positive and decreasing inside support", "[fluids][kernels][poly6]")
{
    const float h = 1.5f;
    const float near = poly6(0.1f * 0.1f, h);
    const float mid = poly6(0.75f * 0.75f, h);
    const float far = poly6(1.4f * 1.4f, h);
    REQUIRE(near > mid);
    REQUIRE(mid > far);
    REQUIRE(far > 0.0f);
}

TEST_CASE("poly6: integrates to ~1 over the support (partition of unity)", "[fluids][kernels][poly6]")
{
    const float h = 1.0f;
    // Coarse midpoint quadrature over the cube [-h,h]³, accumulating only points
    // inside the support sphere. Loose tolerance — this is a normalization sanity
    // check, not a precision test.
    const int steps = 60;
    const float cell = (2.0f * h) / static_cast<float>(steps);
    const float dv = cell * cell * cell;
    double integral = 0.0;
    for (int i = 0; i < steps; ++i)
    {
        const float x = -h + (static_cast<float>(i) + 0.5f) * cell;
        for (int j = 0; j < steps; ++j)
        {
            const float y = -h + (static_cast<float>(j) + 0.5f) * cell;
            for (int k = 0; k < steps; ++k)
            {
                const float z = -h + (static_cast<float>(k) + 0.5f) * cell;
                const float r2 = x * x + y * y + z * z;
                integral += static_cast<double>(poly6(r2, h)) * static_cast<double>(dv);
            }
        }
    }
    REQUIRE_THAT(static_cast<float>(integral), WithinAbs(1.0f, 0.02f));
}

// ============================================================================
// Spiky pressure-kernel gradient
// ============================================================================

TEST_CASE("spiky_gradient: points along -r_vec (repulsive)", "[fluids][kernels][spiky]")
{
    const float h = 1.0f;
    const Vec3f r_vec(0.5f, 0.0f, 0.0f);
    const Vec3f grad = spiky_gradient(r_vec, r_vec.length(), h);
    // Gradient must be antiparallel to r_vec: negative x, zero elsewhere.
    REQUIRE(grad.x < 0.0f);
    REQUIRE_THAT(grad.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(grad.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("spiky_gradient: magnitude matches closed form", "[fluids][kernels][spiky]")
{
    const float h = 1.0f;
    const float r = 0.4f;
    const Vec3f r_vec(r, 0.0f, 0.0f);
    const Vec3f grad = spiky_gradient(r_vec, r, h);
    // ‖∇W‖ = 45/(π h⁶) · (h − r)²
    const float h6 = h * h * h * h * h * h;
    const float expected_mag = 45.0f / (PI * h6) * (h - r) * (h - r);
    REQUIRE_THAT(grad.length(), WithinRel(expected_mag, 1e-5f));
}

TEST_CASE("spiky_gradient: vanishes at and beyond the support radius", "[fluids][kernels][spiky]")
{
    const float h = 1.0f;
    const Vec3f at_h(h, 0.0f, 0.0f);
    REQUIRE_THAT(spiky_gradient(at_h, h, h).length(), WithinAbs(0.0f, 1e-6f));

    const Vec3f beyond(1.5f, 0.0f, 0.0f);
    REQUIRE_THAT(spiky_gradient(beyond, beyond.length(), h).length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("spiky_gradient: singularity guard returns zero at r~0", "[fluids][kernels][spiky]")
{
    const float h = 1.0f;
    const Vec3f grad = spiky_gradient(Vec3f(0.0f), 0.0f, h);
    REQUIRE_THAT(grad.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("spiky_gradient: antisymmetric under r_vec -> -r_vec", "[fluids][kernels][spiky]")
{
    const float h = 1.0f;
    const Vec3f r_vec(0.3f, -0.2f, 0.1f);
    const float r = r_vec.length();
    const Vec3f g_pos = spiky_gradient(r_vec, r, h);
    const Vec3f g_neg = spiky_gradient(-r_vec, r, h);
    REQUIRE_THAT((g_pos + g_neg).length(), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// poly6 gradient / laplacian (surface-tension color field)
// ============================================================================

TEST_CASE("poly6_gradient: vanishes at r=0 and beyond support, antisymmetric", "[fluids][kernels][poly6grad]")
{
    const float h = 1.0f;
    REQUIRE_THAT(poly6_gradient(Vec3f(0.0f), 0.0f, h).length(), WithinAbs(0.0f, 1e-6f));

    const Vec3f beyond(1.2f, 0.0f, 0.0f);
    REQUIRE_THAT(poly6_gradient(beyond, beyond.squaredLength(), h).length(), WithinAbs(0.0f, 1e-6f));

    const Vec3f r_vec(0.3f, -0.1f, 0.2f);
    const float r2 = r_vec.squaredLength();
    REQUIRE_THAT((poly6_gradient(r_vec, r2, h) + poly6_gradient(-r_vec, r2, h)).length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("poly6_gradient: points opposite r_vec (toward denser region)", "[fluids][kernels][poly6grad]")
{
    // Coefficient is negative, so ∇W is antiparallel to r_vec = r_i − r_j — i.e.
    // it points from i toward j (up the color-field gradient).
    const float h = 1.0f;
    const Vec3f r_vec(0.4f, 0.0f, 0.0f);
    const Vec3f grad = poly6_gradient(r_vec, r_vec.squaredLength(), h);
    REQUIRE(grad.x < 0.0f);
}

TEST_CASE("poly6_laplacian: negative near r=0, zero at support", "[fluids][kernels][poly6lap]")
{
    const float h = 1.0f;
    REQUIRE(poly6_laplacian(0.0f, h) < 0.0f);
    REQUIRE_THAT(poly6_laplacian(h * h, h), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(poly6_laplacian(h * h * 1.4f, h), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Viscosity-kernel laplacian
// ============================================================================

TEST_CASE("viscosity_laplacian: value at r=0 matches closed form", "[fluids][kernels][viscosity]")
{
    const float h = 1.0f;
    const float h6 = h * h * h * h * h * h;
    const float expected = 45.0f / (PI * h6) * h; // (h - 0)
    REQUIRE_THAT(viscosity_laplacian(0.0f, h), WithinRel(expected, 1e-5f));
}

TEST_CASE("viscosity_laplacian: non-negative, monotone, zero at h", "[fluids][kernels][viscosity]")
{
    const float h = 2.0f;
    const float near = viscosity_laplacian(0.2f, h);
    const float mid = viscosity_laplacian(1.0f, h);
    REQUIRE(near > mid);
    REQUIRE(mid > 0.0f);
    REQUIRE_THAT(viscosity_laplacian(h, h), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(viscosity_laplacian(h * 1.2f, h), WithinAbs(0.0f, 1e-6f));
}

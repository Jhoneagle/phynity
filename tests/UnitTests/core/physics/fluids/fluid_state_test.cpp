#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/fluids/fluid_particle.hpp>
#include <core/physics/fluids/sph_parameters.hpp>

using Catch::Matchers::WithinRel;
using phynity::math::vectors::Vec3f;
using phynity::physics::constants::WATER_DENSITY;
using phynity::physics::fluids::FluidParticle;
using phynity::physics::fluids::mass_for_spacing;
using phynity::physics::fluids::SphParameters;

TEST_CASE("FluidParticle: default-initialized to zero", "[fluids][state]")
{
    FluidParticle p;
    REQUIRE(p.position == Vec3f(0.0f));
    REQUIRE(p.velocity == Vec3f(0.0f));
    REQUIRE(p.force == Vec3f(0.0f));
    REQUIRE(p.mass == 0.0f);
    REQUIRE(p.density == 0.0f);
    REQUIRE(p.pressure == 0.0f);
}

TEST_CASE("SphParameters: sane defaults", "[fluids][params]")
{
    SphParameters params;
    REQUIRE(params.smoothing_radius > 0.0f);
    REQUIRE_THAT(params.rest_density, WithinRel(WATER_DENSITY, 1e-6f));
    REQUIRE(params.surface_tension == 0.0f); // off by default
    REQUIRE(params.bounds.is_valid());
}

TEST_CASE("mass_for_spacing: m = rho0 * s^3", "[fluids][params]")
{
    const float rho0 = 1000.0f;
    const float spacing = 0.05f;
    REQUIRE_THAT(mass_for_spacing(rho0, spacing), WithinRel(rho0 * spacing * spacing * spacing, 1e-6f));
    // Doubling spacing multiplies mass by 8 (cubic scaling).
    REQUIRE_THAT(mass_for_spacing(rho0, 2.0f * spacing), WithinRel(8.0f * mass_for_spacing(rho0, spacing), 1e-5f));
}

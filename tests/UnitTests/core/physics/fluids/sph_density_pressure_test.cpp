#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/fluids/sph_fluid_system.hpp>
#include <core/physics/fluids/sph_kernels.hpp>

using Catch::Matchers::WithinRel;
using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::poly6;
using phynity::physics::fluids::SphFluidSystem;
using phynity::physics::fluids::SphParameters;

namespace
{
SphParameters make_params(float h)
{
    SphParameters p;
    p.smoothing_radius = h;
    p.rest_density = 1000.0f;
    p.stiffness = 2000.0f;
    return p;
}
} // namespace

TEST_CASE("compute_density: isolated particle is just the self term", "[fluids][density]")
{
    const float h = 1.0f;
    const float mass = 0.2f;

    SphFluidSystem system(make_params(h));
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), mass);
    system.rebuild_neighbors();
    system.compute_density();

    REQUIRE_THAT(system.particles()[0].density, WithinRel(mass * poly6(0.0f, h), 1e-5f));
}

TEST_CASE("compute_density: symmetric pair yields equal, larger densities", "[fluids][density]")
{
    const float h = 1.0f;
    const float mass = 0.2f;
    const float d = 0.4f;

    SphFluidSystem system(make_params(h));
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), mass);
    system.spawn(Vec3f(d, 0.0f, 0.0f), Vec3f(0.0f), mass);
    system.rebuild_neighbors();
    system.compute_density();

    const float expected = mass * poly6(0.0f, h) + mass * poly6(d * d, h);
    REQUIRE_THAT(system.particles()[0].density, WithinRel(expected, 1e-5f));
    REQUIRE_THAT(system.particles()[1].density, WithinRel(expected, 1e-5f));
    // Neighbor contribution strictly increases density above the isolated value.
    REQUIRE(system.particles()[0].density > mass * poly6(0.0f, h));
}

TEST_CASE("compute_pressure: sign follows compression state", "[fluids][pressure]")
{
    SphFluidSystem system(make_params(1.0f));
    system.spawn(Vec3f(0.0f)); // one particle so we can hand-set density
    system.spawn(Vec3f(10.0f, 0.0f, 0.0f));

    // Compressed: ρ > ρ₀ ⇒ p > 0.  Rarefied: ρ < ρ₀ ⇒ p < 0.
    system.particles()[0].density = 1200.0f;
    system.particles()[1].density = 800.0f;
    system.compute_pressure();

    REQUIRE(system.particles()[0].pressure > 0.0f);
    REQUIRE(system.particles()[1].pressure < 0.0f);
    REQUIRE_THAT(system.particles()[0].pressure, WithinRel(2000.0f * 200.0f, 1e-5f));
}

TEST_CASE("compute_pressure: clamp flag suppresses negative pressure", "[fluids][pressure]")
{
    SphFluidSystem system(make_params(1.0f));
    system.parameters().clamp_negative_pressure = true;
    system.spawn(Vec3f(0.0f));
    system.particles()[0].density = 800.0f; // rarefied
    system.compute_pressure();

    REQUIRE(system.particles()[0].pressure == 0.0f);
}

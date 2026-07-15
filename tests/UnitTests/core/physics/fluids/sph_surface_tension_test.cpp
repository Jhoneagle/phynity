#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/fluids/sph_fluid_system.hpp>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::SphFluidSystem;
using phynity::physics::fluids::SphParameters;

namespace
{
// Surface tension in isolation: no pressure, no viscosity, no gravity.
SphParameters tension_only_params(float sigma, float threshold)
{
    SphParameters p;
    p.smoothing_radius = 1.0f;
    p.rest_density = 1000.0f;
    p.stiffness = 0.0f; // no pressure force
    p.viscosity = 0.0f; // no viscosity force
    p.surface_tension = sigma;
    p.surface_tension_threshold = threshold;
    return p;
}
} // namespace

TEST_CASE("surface tension: off by default contributes no force", "[fluids][tension]")
{
    SphParameters params;
    params.smoothing_radius = 1.0f;
    params.stiffness = 0.0f;
    params.viscosity = 0.0f;
    REQUIRE(params.surface_tension == 0.0f);

    SphFluidSystem system(params);
    system.set_ambient_gravity(Vec3f(0.0f));
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), 0.2f);
    system.spawn(Vec3f(0.3f, 0.0f, 0.0f), Vec3f(0.0f), 0.2f);
    system.rebuild_neighbors();
    system.compute_density();
    system.compute_pressure();
    system.compute_forces();

    REQUIRE_THAT(system.particles()[0].force.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("surface tension: symmetric interior neighborhood yields ~zero net force", "[fluids][tension]")
{
    SphFluidSystem system(tension_only_params(1.0f, 0.0f));
    system.set_ambient_gravity(Vec3f(0.0f));

    const float s = 0.3f;
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), 0.2f); // center (index 0)
    system.spawn(Vec3f(s, 0.0f, 0.0f), Vec3f(0.0f), 0.2f);
    system.spawn(Vec3f(-s, 0.0f, 0.0f), Vec3f(0.0f), 0.2f);
    system.spawn(Vec3f(0.0f, s, 0.0f), Vec3f(0.0f), 0.2f);
    system.spawn(Vec3f(0.0f, -s, 0.0f), Vec3f(0.0f), 0.2f);
    system.spawn(Vec3f(0.0f, 0.0f, s), Vec3f(0.0f), 0.2f);
    system.spawn(Vec3f(0.0f, 0.0f, -s), Vec3f(0.0f), 0.2f);
    system.rebuild_neighbors();
    system.compute_density();
    system.compute_pressure();
    system.compute_forces();

    // Symmetric neighbors ⇒ color-field gradient (the interface normal) cancels
    // at the center, so no cohesive force there (interior is "flat").
    REQUIRE_THAT(system.particles()[0].force.length(), WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("surface tension: an isolated blob pulls its edges inward (cohesion)", "[fluids][tension]")
{
    SphFluidSystem system(tension_only_params(1.0f, 0.0f));
    system.set_ambient_gravity(Vec3f(0.0f));

    // Three particles in a line, centroid at the origin.
    system.spawn(Vec3f(-0.2f, 0.0f, 0.0f), Vec3f(0.0f), 0.2f); // left  (index 0)
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), 0.2f); // center(index 1)
    system.spawn(Vec3f(0.2f, 0.0f, 0.0f), Vec3f(0.0f), 0.2f); // right (index 2)
    system.rebuild_neighbors();
    system.compute_density();
    system.compute_pressure();
    system.compute_forces();

    const Vec3f fL = system.particles()[0].force;
    const Vec3f fC = system.particles()[1].force;
    const Vec3f fR = system.particles()[2].force;

    REQUIRE(fL.x > 0.0f); // left edge pulled toward centroid (+x, inward)
    REQUIRE(fR.x < 0.0f); // right edge pulled inward (−x)
    REQUIRE_THAT(fC.x, WithinAbs(0.0f, 1e-4f)); // symmetric center, no net pull

    // Symmetric arrangement ⇒ zero net force (momentum preserved).
    REQUIRE_THAT((fL + fC + fR).length(), WithinAbs(0.0f, 1e-4f));
}

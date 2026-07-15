#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/fluids/sph_fluid_system.hpp>

#include <vector>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::SphFluidSystem;
using phynity::physics::fluids::SphParameters;

namespace
{
SphParameters pressure_only_params()
{
    SphParameters p;
    p.smoothing_radius = 1.0f;
    p.rest_density = 1000.0f;
    p.stiffness = 2000.0f;
    p.viscosity = 0.0f; // isolate the pressure term
    return p;
}
} // namespace

TEST_CASE("compute_forces: symmetric pair gives equal-and-opposite pressure forces", "[fluids][forces]")
{
    SphFluidSystem system(pressure_only_params());
    system.set_ambient_gravity(Vec3f(0.0f)); // isolate pressure
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), 0.2f);
    system.spawn(Vec3f(0.3f, 0.0f, 0.0f), Vec3f(0.0f), 0.2f);
    system.rebuild_neighbors();

    // Equal density & pressure so we test the ∇W_ij = −∇W_ji antisymmetry.
    for (auto &p : system.particles())
    {
        p.density = 1000.0f;
        p.pressure = 500.0f;
    }
    system.compute_forces();

    const Vec3f f0 = system.particles()[0].force;
    const Vec3f f1 = system.particles()[1].force;
    REQUIRE(f0.length() > 0.0f);
    REQUIRE_THAT((f0 + f1).length(), WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("compute_forces: N-particle cluster conserves linear momentum (pressure only)", "[fluids][forces]")
{
    SphFluidSystem system(pressure_only_params());
    system.set_ambient_gravity(Vec3f(0.0f));

    // Irregular cluster so densities/pressures genuinely differ per particle —
    // the Monaghan form must still sum forces to ~0.
    const std::vector<Vec3f> pts = {
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(0.2f, 0.05f, 0.0f),
        Vec3f(-0.15f, 0.1f, 0.05f),
        Vec3f(0.1f, -0.2f, 0.1f),
        Vec3f(-0.1f, -0.1f, -0.1f),
        Vec3f(0.25f, 0.2f, -0.05f),
    };
    for (const auto &p : pts)
    {
        system.spawn(p, Vec3f(0.0f), 0.2f);
    }

    system.rebuild_neighbors();
    system.compute_density();
    system.compute_pressure();
    system.compute_forces();

    Vec3f net(0.0f);
    float total_magnitude = 0.0f;
    for (const auto &p : system.particles())
    {
        net += p.force;
        total_magnitude += p.force.length();
    }
    // Internal (pressure) forces must cancel pairwise ⇒ net ≈ 0. The invariant is
    // scale-independent, so assert net relative to the total force magnitude: the
    // residual is pure float32 rounding of summing large canceling terms
    // (~1e-7 relative), whereas the double-counting bug produced ~20% relative.
    REQUIRE(net.length() <= 1e-5f * total_magnitude);
}

TEST_CASE("compute_forces: uniform field gives ~zero net pressure force at a symmetric center", "[fluids][forces]")
{
    SphFluidSystem system(pressure_only_params());
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

    for (auto &p : system.particles())
    {
        p.density = 1000.0f;
        p.pressure = 500.0f;
    }
    system.compute_forces();

    // Octahedrally symmetric neighbors ⇒ Σ ∇W cancels at the center.
    REQUIRE_THAT(system.particles()[0].force.length(), WithinAbs(0.0f, 1e-3f));
}

TEST_CASE("compute_forces: viscosity opposes relative velocity", "[fluids][forces]")
{
    SphParameters params;
    params.smoothing_radius = 1.0f;
    params.rest_density = 1000.0f;
    params.stiffness = 0.0f; // no pressure — isolate viscosity
    params.viscosity = 0.5f;

    SphFluidSystem system(params);
    system.set_ambient_gravity(Vec3f(0.0f));
    system.spawn(Vec3f(0.0f), Vec3f(1.0f, 0.0f, 0.0f), 0.2f); // moving +x
    system.spawn(Vec3f(0.3f, 0.0f, 0.0f), Vec3f(0.0f), 0.2f); // at rest
    system.rebuild_neighbors();
    system.compute_density();
    system.compute_pressure(); // all zero (stiffness 0)
    system.compute_forces();

    // Drag on the moving particle must point against its velocity.
    REQUIRE(system.particles()[0].force.dot(Vec3f(1.0f, 0.0f, 0.0f)) < 0.0f);
    // And equal-and-opposite at uniform density (both densities computed equal here).
    REQUIRE_THAT((system.particles()[0].force + system.particles()[1].force).length(), WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("integrate: semi-implicit Euler updates velocity then position", "[fluids][forces]")
{
    SphParameters params;
    params.smoothing_radius = 1.0f;
    SphFluidSystem system(params);
    system.spawn(Vec3f(0.0f), Vec3f(0.0f), 2.0f);

    // Constant force of 4 N on a 2 kg particle ⇒ a = 2 m/s².
    system.particles()[0].force = Vec3f(4.0f, 0.0f, 0.0f);
    const float dt = 0.5f;
    system.integrate(dt);

    // v = a·dt = 1;  x = v·dt = 0.5.
    REQUIRE_THAT(system.particles()[0].velocity.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(system.particles()[0].position.x, WithinAbs(0.5f, 1e-6f));
}

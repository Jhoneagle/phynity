#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/fluids/pbf_fluid_system.hpp>
#include <core/physics/shapes/aabb.hpp>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::PbfFluidSystem;
using phynity::physics::fluids::PbfParameters;
using phynity::physics::shapes::AABB;

namespace
{
PbfParameters base_params()
{
    PbfParameters p;
    p.sph.smoothing_radius = 1.0f;
    p.sph.rest_density = 1000.0f; // sparse pairs are rarefied ⇒ constraint attracts
    p.sph.particle_mass = 0.125f;
    p.sph.bounds = AABB(Vec3f(-10.0f), Vec3f(10.0f)); // no walls
    p.solver_iterations = 4;
    return p;
}

// One constraint iteration on a close pair; returns the x-component of the
// correction applied to particle 0 (which sits at the origin with a neighbor at
// +x). The rest density is set to the pair's own density so the *base*
// constraint is neutral (C ≈ 0) and any correction is attributable to s_corr.
float delta0_x(bool tensile_on)
{
    PbfParameters params = base_params();

    PbfFluidSystem probe(params);
    probe.set_ambient_gravity(Vec3f(0.0f));
    probe.spawn(Vec3f(0.0f, 0.0f, 0.0f));
    probe.spawn(Vec3f(0.2f, 0.0f, 0.0f));
    probe.predict(0.0001f);
    probe.rebuild_neighbors_predicted();
    probe.compute_densities();
    const float pair_density = probe.particles()[0].density;

    params.sph.rest_density = pair_density; // C ≈ 0 ⇒ neutral base constraint
    if (tensile_on)
    {
        params.scorr_k = 0.1f;
        params.scorr_dq = 0.2f;
        params.scorr_n = 4.0f;
    }

    PbfFluidSystem system(params);
    system.set_ambient_gravity(Vec3f(0.0f));
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f));
    system.spawn(Vec3f(0.2f, 0.0f, 0.0f));
    system.predict(0.0001f);
    system.rebuild_neighbors_predicted();
    system.iterate();
    return system.delta_positions()[0].x;
}
} // namespace

TEST_CASE("PBF tensile correction: adds a repulsive correction that resists clumping", "[fluids][pbf][scorr]")
{
    const float without = delta0_x(false);
    const float with = delta0_x(true);

    // With a neutral base constraint the bare correction is ~0; s_corr adds a
    // repulsion that pushes particle 0 away from its +x neighbor (−x direction).
    REQUIRE(std::abs(without) < 1e-4f);
    REQUIRE(with < -1e-4f);
}

TEST_CASE("PBF XSPH: reduces relative velocity without adding net momentum", "[fluids][pbf][xsph]")
{
    PbfParameters params = base_params();
    params.xsph_c = 0.1f;

    PbfFluidSystem system(params);
    system.set_ambient_gravity(Vec3f(0.0f));
    // Two equal-mass particles approaching each other head-on.
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f));
    system.spawn(Vec3f(0.3f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f));

    // Drive the stages up to (but not through) velocity finalization so we can
    // isolate XSPH acting on the given velocities.
    system.predict(0.0001f); // tiny dt, gravity 0 ⇒ velocities ~unchanged
    system.rebuild_neighbors_predicted();
    system.compute_densities();

    const Vec3f rel_before = system.particles()[0].velocity - system.particles()[1].velocity;
    const Vec3f mom_before = system.particles()[0].velocity + system.particles()[1].velocity; // equal mass

    system.apply_xsph();

    const Vec3f rel_after = system.particles()[0].velocity - system.particles()[1].velocity;
    const Vec3f mom_after = system.particles()[0].velocity + system.particles()[1].velocity;

    // Relative velocity magnitude shrinks (velocities pulled toward each other).
    REQUIRE(rel_after.length() < rel_before.length());
    // Symmetric pair (equal densities) ⇒ net momentum unchanged.
    REQUIRE_THAT((mom_after - mom_before).length(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("PBF stabilization: all terms off by default is well-behaved", "[fluids][pbf][defaults]")
{
    PbfParameters params = base_params();
    REQUIRE(params.scorr_k == 0.0f);
    REQUIRE(params.xsph_c == 0.0f);
    REQUIRE(params.vorticity_epsilon == 0.0f);

    PbfFluidSystem system(params);
    system.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f));
    system.spawn(Vec3f(0.15f, 0.0f, 0.0f));

    // Off-by-default stabilization must not introduce NaNs/inf over a few steps.
    for (int step = 0; step < 10; ++step)
    {
        system.update(0.005f);
    }
    for (const auto &p : system.particles())
    {
        REQUIRE(std::isfinite(p.position.x));
        REQUIRE(std::isfinite(p.velocity.x));
    }
}

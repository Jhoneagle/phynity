#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/fluids/pbf_fluid_system.hpp>
#include <core/physics/shapes/aabb.hpp>

#include <algorithm>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::PbfFluidSystem;
using phynity::physics::fluids::PbfParameters;
using phynity::physics::shapes::AABB;

namespace
{
float max_density(const PbfFluidSystem &s)
{
    float m = 0.0f;
    for (const auto &p : s.particles())
    {
        m = std::max(m, p.density);
    }
    return m;
}

Vec3f centroid(const std::vector<Vec3f> &pts)
{
    Vec3f c(0.0f);
    for (const auto &p : pts)
    {
        c += p;
    }
    return c * (1.0f / static_cast<float>(pts.size()));
}
} // namespace

TEST_CASE("PBF: lambda is negative when compressed and correction pushes apart", "[fluids][pbf]")
{
    PbfParameters params;
    params.sph.smoothing_radius = 1.0f;
    params.sph.rest_density = 0.2f; // low enough that a close pair is "over-dense"
    params.sph.particle_mass = 0.125f;
    params.sph.bounds = AABB(Vec3f(-10.0f), Vec3f(10.0f));

    PbfFluidSystem system(params);
    system.set_ambient_gravity(Vec3f(0.0f));
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f));
    system.spawn(Vec3f(0.3f, 0.0f, 0.0f));

    system.predict(0.001f); // v=0, gravity=0 ⇒ predicted ≈ positions
    system.rebuild_neighbors_predicted();
    system.iterate();

    // Compressed (ρ > ρ₀) ⇒ C > 0 ⇒ λ < 0.
    REQUIRE(system.lambdas()[0] < 0.0f);
    REQUIRE(system.lambdas()[1] < 0.0f);

    // Correction on particle 0 points away from particle 1 (which sits at +x).
    REQUIRE(system.delta_positions()[0].x < 0.0f);
    REQUIRE(system.delta_positions()[1].x > 0.0f);
}

TEST_CASE("PBF: constraint projection is centroid-preserving (momentum)", "[fluids][pbf]")
{
    PbfParameters params;
    params.sph.smoothing_radius = 1.0f;
    params.sph.rest_density = 0.2f;
    params.sph.particle_mass = 0.125f;
    params.sph.bounds = AABB(Vec3f(-10.0f), Vec3f(10.0f)); // no clamping

    PbfFluidSystem system(params);
    system.set_ambient_gravity(Vec3f(0.0f));
    // An irregular compressed cluster, away from walls.
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f));
    system.spawn(Vec3f(0.3f, 0.1f, 0.0f));
    system.spawn(Vec3f(-0.2f, 0.25f, 0.1f));
    system.spawn(Vec3f(0.15f, -0.2f, -0.1f));

    system.predict(0.001f);
    system.rebuild_neighbors_predicted();

    const Vec3f before = centroid(system.predicted());
    system.iterate();
    const Vec3f after = centroid(system.predicted());

    // ΣΔp = 0 (pairwise-symmetric correction) ⇒ centroid is preserved.
    REQUIRE_THAT((after - before).length(), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("PBF: iterating a compressed cluster drives peak density toward rest", "[fluids][pbf]")
{
    PbfParameters params;
    params.sph.smoothing_radius = 0.4f;
    params.sph.particle_mass = 0.5f;
    params.sph.bounds = AABB(Vec3f(-10.0f), Vec3f(10.0f));
    params.relaxation = 1.0e-3f;

    PbfFluidSystem system(params);
    system.set_ambient_gravity(Vec3f(0.0f));

    // A 4×4×4 block packed tighter than rest spacing.
    const float packed = 0.12f;
    for (int ix = 0; ix < 4; ++ix)
    {
        for (int iy = 0; iy < 4; ++iy)
        {
            for (int iz = 0; iz < 4; ++iz)
            {
                system.spawn(Vec3f(static_cast<float>(ix) * packed, static_cast<float>(iy) * packed,
                                   static_cast<float>(iz) * packed));
            }
        }
    }

    system.predict(0.001f);
    system.rebuild_neighbors_predicted();
    system.compute_densities();
    const float initial_max = max_density(system);

    // Set the rest density below the packed density so the cluster is compressed.
    params.sph.rest_density = 0.6f * initial_max;
    system.set_parameters(params);

    float prev = initial_max;
    for (int iter = 0; iter < 5; ++iter)
    {
        system.iterate();
        system.compute_densities();
        const float current = max_density(system);
        // Non-increasing (allowing a tiny numerical slack) and trending down.
        REQUIRE(current <= prev + 1e-2f * initial_max);
        prev = current;
    }

    // After projection the peak density is meaningfully lower than the packed start.
    REQUIRE(prev < initial_max);
}

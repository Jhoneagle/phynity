#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/physics/fluids/pbf_fluid_system.hpp>
#include <core/physics/fluids/sph_fluid_system.hpp>
#include <core/physics/shapes/aabb.hpp>

#include <algorithm>

using phynity::math::utilities::is_finite;
using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::mass_for_spacing;
using phynity::physics::fluids::PbfFluidSystem;
using phynity::physics::fluids::PbfParameters;
using phynity::physics::fluids::SphFluidSystem;
using phynity::physics::fluids::SphParameters;
using phynity::physics::shapes::AABB;

// ============================================================================
// PBF Fluid Validation Tests
// ============================================================================
// Tolerance-based validation of the position-based fluids solver, mirroring the
// WCSPH suite (bounded/finite, density near ρ₀). The head-to-head test is PBF's
// concrete payoff: at a timestep where a stiff (strongly incompressible) WCSPH
// fluid explodes, PBF stays bounded and near rest density.

namespace
{
constexpr float kRestDensity = 1000.0f;
constexpr float kSpacing = 0.05f;
constexpr float kSmoothing = 0.1f;
constexpr float kDt = 0.002f;

PbfParameters stable_params()
{
    PbfParameters p;
    p.sph.smoothing_radius = kSmoothing;
    p.sph.rest_density = kRestDensity;
    p.sph.particle_mass = mass_for_spacing(kRestDensity, kSpacing);
    p.sph.bounds = AABB(Vec3f(-0.5f), Vec3f(0.5f));
    p.solver_iterations = 10; // enough to converge incompressibility
    p.relaxation = 1.0e-4f;
    p.clamp_density_deficiency = true; // compression-only ⇒ no free-surface collapse
    p.xsph_c = 0.02f; // mild velocity smoothing
    return p;
}

template <typename S> void seed_block(S &system, int nx, int ny, int nz, const Vec3f &origin)
{
    for (int ix = 0; ix < nx; ++ix)
    {
        for (int iy = 0; iy < ny; ++iy)
        {
            for (int iz = 0; iz < nz; ++iz)
            {
                system.spawn(origin + Vec3f(static_cast<float>(ix) * kSpacing,
                                            static_cast<float>(iy) * kSpacing,
                                            static_cast<float>(iz) * kSpacing));
            }
        }
    }
}

template <typename S> float max_speed(const S &system)
{
    float m = 0.0f;
    for (const auto &p : system.particles())
    {
        m = std::max(m, p.velocity.length());
    }
    return m;
}

template <typename S> float max_density(const S &system)
{
    float m = 0.0f;
    for (const auto &p : system.particles())
    {
        m = std::max(m, p.density);
    }
    return m;
}

template <typename S> bool all_finite_and_in_bounds(const S &system)
{
    for (const auto &p : system.particles())
    {
        if (!is_finite(p.position.x) || !is_finite(p.position.y) || !is_finite(p.position.z))
        {
            return false;
        }
        if (!system.parameters().sph.bounds.contains_point(p.position))
        {
            return false;
        }
    }
    return true;
}
} // namespace

TEST_CASE("PBF Validation - Hydrostatic column stays bounded and incompressible", "[fluids_validation]")
{
    PbfFluidSystem system(stable_params());
    system.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));

    seed_block(system, 5, 8, 5, Vec3f(-0.15f, -0.49f, -0.15f));
    REQUIRE(system.particle_count() == 200);

    for (int step = 0; step < 400; ++step)
    {
        system.update(kDt);
        REQUIRE(all_finite_and_in_bounds(system));
    }

    REQUIRE(max_speed(system) < 6.0f);
    // Compression-only projection keeps peak density from exceeding ρ₀ by more
    // than a few percent.
    REQUIRE(max_density(system) < 1.05f * kRestDensity);
}

TEST_CASE("PBF Validation - Dropped block settles within bounds", "[fluids_validation]")
{
    PbfFluidSystem system(stable_params());
    system.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));

    seed_block(system, 5, 5, 5, Vec3f(-0.12f, 0.1f, -0.12f));
    REQUIRE(system.particle_count() == 125);

    float min_y_seen = 1e9f;
    for (int step = 0; step < 500; ++step)
    {
        system.update(kDt);
        REQUIRE(all_finite_and_in_bounds(system));
        for (const auto &p : system.particles())
        {
            min_y_seen = std::min(min_y_seen, p.position.y);
        }
    }

    REQUIRE(min_y_seen < 0.0f); // it fell
    REQUIRE(max_speed(system) < 6.0f); // stayed bounded

    float min_x = 1e9f;
    float max_x = -1e9f;
    for (const auto &p : system.particles())
    {
        min_x = std::min(min_x, p.position.x);
        max_x = std::max(max_x, p.position.x);
    }
    REQUIRE((max_x - min_x) > 4.0f * kSpacing); // spread beyond the initial width
}

TEST_CASE("PBF Validation - Head-to-head: PBF stable where a viscous WCSPH blows up", "[fluids_validation]")
{
    // WCSPH integrates its viscosity term explicitly, so a fluid with a
    // meaningful viscosity is stable only below a small dt. PBF instead smooths
    // velocities with XSPH, which is unconditionally stable. At dt = 2 ms a
    // viscous WCSPH fluid blows up, while PBF stays bounded and near ρ₀ — a
    // concrete payoff of position-based fluids.
    const int steps = 300;
    const Vec3f drop_origin(-0.12f, 0.1f, -0.12f);

    // --- Stiff WCSPH at the large dt ---
    SphParameters sph;
    sph.smoothing_radius = kSmoothing;
    sph.rest_density = kRestDensity;
    sph.stiffness = 100.0f;
    sph.viscosity = 1.0f; // meaningful viscosity ⇒ explicit term is stiff, needs a small dt
    sph.clamp_negative_pressure = true;
    sph.boundary_restitution = 0.0f;
    sph.particle_mass = mass_for_spacing(kRestDensity, kSpacing);
    sph.bounds = AABB(Vec3f(-0.5f), Vec3f(0.5f));

    SphFluidSystem wcsph(sph);
    wcsph.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    seed_block(wcsph, 5, 5, 5, drop_origin);
    for (int step = 0; step < steps; ++step)
    {
        wcsph.update(kDt);
    }
    const float wcsph_density = max_density(wcsph);

    // --- PBF at the same large dt ---
    PbfFluidSystem pbf(stable_params());
    pbf.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));
    seed_block(pbf, 5, 5, 5, drop_origin);
    for (int step = 0; step < steps; ++step)
    {
        pbf.update(kDt);
    }

    // WCSPH is unstable at this dt: its explicit viscosity blows the density far
    // past ρ₀ (or to non-finite) ...
    REQUIRE((!is_finite(wcsph_density) || wcsph_density > 2.0f * kRestDensity));
    // ... while PBF stays bounded, finite, confined, and near rest density.
    REQUIRE(all_finite_and_in_bounds(pbf));
    REQUIRE(max_speed(pbf) < 6.0f);
    REQUIRE(max_density(pbf) < 1.05f * kRestDensity);
}

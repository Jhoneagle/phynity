#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/physics/fluids/sph_fluid_system.hpp>
#include <core/physics/shapes/aabb.hpp>

#include <algorithm>
#include <vector>

using phynity::math::utilities::is_finite;
using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::mass_for_spacing;
using phynity::physics::fluids::SphFluidSystem;
using phynity::physics::fluids::SphParameters;
using phynity::physics::shapes::AABB;

// ============================================================================
// WCSPH Fluid Validation Tests
// ============================================================================
// Multi-step, tolerance-based validation of emergent WCSPH behavior. Following
// the fields-work rationale, these assert *bounded/finite* behavior and coarse
// physical properties (density near ρ₀, no blow-up) rather than exact
// trajectories — WCSPH is subject to a CFL-like stability limit and a golden
// baseline would be cross-toolchain fragile. Wall-adjacent assertions are kept
// loose because the clamp-and-reflect boundary applies no boundary pressure.
//
// Parameters (stiffness 100, viscosity 0.05, dt 5e-4) were tuned for stability:
// the Müller viscosity term is integrated explicitly, so a large μ is
// unconditionally unstable at this dt/h — a small μ both damps and stays stable.

namespace
{
constexpr float kRestDensity = 1000.0f;
constexpr float kSpacing = 0.05f;
constexpr float kSmoothing = 0.1f; // h = 2·spacing
constexpr float kDt = 0.0005f;

SphParameters stable_params()
{
    SphParameters p;
    p.smoothing_radius = kSmoothing;
    p.rest_density = kRestDensity;
    p.stiffness = 100.0f; // c ≈ 10 m/s (weakly compressible, CFL-friendly)
    p.viscosity = 0.05f; // small: explicit viscosity is stiff at this dt/h
    p.surface_tension = 0.0f;
    p.clamp_negative_pressure = true; // suppress free-surface tensile instability
    p.boundary_restitution = 0.0f; // fully damped container walls
    p.particle_mass = mass_for_spacing(kRestDensity, kSpacing);
    p.bounds = AABB(Vec3f(-0.5f), Vec3f(0.5f));
    return p;
}

void seed_block(SphFluidSystem &system, int nx, int ny, int nz, float spacing, const Vec3f &origin)
{
    for (int ix = 0; ix < nx; ++ix)
    {
        for (int iy = 0; iy < ny; ++iy)
        {
            for (int iz = 0; iz < nz; ++iz)
            {
                system.spawn(origin + Vec3f(static_cast<float>(ix) * spacing,
                                            static_cast<float>(iy) * spacing,
                                            static_cast<float>(iz) * spacing));
            }
        }
    }
}

float max_speed(const SphFluidSystem &system)
{
    float m = 0.0f;
    for (const auto &p : system.particles())
    {
        m = std::max(m, p.velocity.length());
    }
    return m;
}

float max_density(const SphFluidSystem &system)
{
    float m = 0.0f;
    for (const auto &p : system.particles())
    {
        m = std::max(m, p.density);
    }
    return m;
}

bool all_finite_and_in_bounds(const SphFluidSystem &system)
{
    return std::ranges::all_of(system.particles(), [&](const auto &p) {
        return is_finite(p.position.x) && is_finite(p.position.y) && is_finite(p.position.z) &&
               system.parameters().bounds.contains_point(p.position);
    });
}
} // namespace

TEST_CASE("SPH Validation - Hydrostatic column stays bounded and settles", "[fluids_validation]")
{
    SphFluidSystem system(stable_params());
    system.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));

    // A 5×8×5 column resting near the floor of a 1 m cube.
    seed_block(system, 5, 8, 5, kSpacing, Vec3f(-0.15f, -0.49f, -0.15f));
    REQUIRE(system.particle_count() == 200);

    for (int step = 0; step < 1200; ++step)
    {
        system.update(kDt);
        REQUIRE(all_finite_and_in_bounds(system));
    }

    // Settled under gravity: no blow-up, near rest.
    REQUIRE(max_speed(system) < 1.0f);
}

TEST_CASE("SPH Validation - Incompressibility: density stays near rest density", "[fluids_validation]")
{
    SphFluidSystem system(stable_params());
    system.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));

    seed_block(system, 5, 8, 5, kSpacing, Vec3f(-0.15f, -0.49f, -0.15f));

    for (int step = 0; step < 1200; ++step)
    {
        system.update(kDt);
    }

    // Weakly-compressible: peak density stays within ~10% of ρ₀ (observed ~5%).
    REQUIRE(max_density(system) < 1.10f * kRestDensity);
    REQUIRE(max_density(system) > 0.5f * kRestDensity);
}

TEST_CASE("SPH Validation - Dropped block settles within bounds, energy bounded", "[fluids_validation]")
{
    SphFluidSystem system(stable_params());
    system.set_ambient_gravity(Vec3f(0.0f, -9.81f, 0.0f));

    // A block released from mid-air: it should fall, hit the floor, spread, and
    // stay confined — never diverging to NaN/inf or leaving the container.
    seed_block(system, 5, 5, 5, kSpacing, Vec3f(-0.12f, 0.1f, -0.12f));
    REQUIRE(system.particle_count() == 125);

    float min_y_seen = 1e9f;
    for (int step = 0; step < 2000; ++step)
    {
        system.update(kDt);
        REQUIRE(all_finite_and_in_bounds(system));
        for (const auto &p : system.particles())
        {
            min_y_seen = std::min(min_y_seen, p.position.y);
        }
    }

    // It fell below its release height and stayed bounded.
    REQUIRE(min_y_seen < 0.0f);
    REQUIRE(max_speed(system) < 5.0f);

    // Spread along the floor: horizontal extent exceeds the initial block width.
    float min_x = 1e9f;
    float max_x = -1e9f;
    for (const auto &p : system.particles())
    {
        min_x = std::min(min_x, p.position.x);
        max_x = std::max(max_x, p.position.x);
    }
    REQUIRE((max_x - min_x) > 4.0f * kSpacing); // initial width was 4·spacing
}

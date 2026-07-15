#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/dynamics/coulomb_interaction.hpp>
#include <core/physics/particles/particle.hpp>

#include <cmath>
#include <vector>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::Vec3f;
using phynity::physics::accumulate_coulomb_forces;
using phynity::physics::Particle;

namespace
{
Particle make_charge(const Vec3f &position, float charge)
{
    Particle p;
    p.position = position;
    p.material.charge = charge;
    return p;
}
} // namespace

TEST_CASE("Coulomb: Like charges repel with equal and opposite forces", "[coulomb]")
{
    std::vector<Particle> particles;
    particles.push_back(make_charge(Vec3f(0.0f, 0.0f, 0.0f), 1.0f));
    particles.push_back(make_charge(Vec3f(2.0f, 0.0f, 0.0f), 1.0f));

    accumulate_coulomb_forces(particles, 1.0f, 1e-3f);

    // |F| = k q1 q2 / r² = 1/4. Particle 0 (left) pushed -x, particle 1 pushed +x.
    REQUIRE_THAT(particles[0].force_accumulator.x, WithinAbs(-0.25f, 1e-6f));
    REQUIRE_THAT(particles[1].force_accumulator.x, WithinAbs(0.25f, 1e-6f));

    // Equal and opposite.
    REQUIRE_THAT(particles[0].force_accumulator.x, WithinAbs(-particles[1].force_accumulator.x, 1e-6f));
    REQUIRE_THAT(particles[0].force_accumulator.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(particles[0].force_accumulator.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Coulomb: Opposite charges attract", "[coulomb]")
{
    std::vector<Particle> particles;
    particles.push_back(make_charge(Vec3f(0.0f, 0.0f, 0.0f), 1.0f));
    particles.push_back(make_charge(Vec3f(2.0f, 0.0f, 0.0f), -1.0f));

    accumulate_coulomb_forces(particles, 1.0f, 1e-3f);

    // Particle 0 (left) pulled toward +x; particle 1 pulled toward -x.
    REQUIRE(particles[0].force_accumulator.x > 0.0f);
    REQUIRE(particles[1].force_accumulator.x < 0.0f);
    REQUIRE_THAT(particles[0].force_accumulator.x, WithinAbs(-particles[1].force_accumulator.x, 1e-6f));
}

TEST_CASE("Coulomb: Total force on a cluster sums to zero (momentum conservation)", "[coulomb]")
{
    std::vector<Particle> particles;
    particles.push_back(make_charge(Vec3f(0.0f, 0.0f, 0.0f), 1.0f));
    particles.push_back(make_charge(Vec3f(1.5f, 0.5f, 0.0f), -2.0f));
    particles.push_back(make_charge(Vec3f(-1.0f, 1.0f, 0.5f), 0.5f));
    particles.push_back(make_charge(Vec3f(0.3f, -1.2f, 2.0f), 3.0f));

    accumulate_coulomb_forces(particles, 2.0f, 1e-3f);

    Vec3f total(0.0f);
    for (const auto &p : particles)
    {
        total += p.force_accumulator;
    }

    REQUIRE_THAT(total.x, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(total.y, WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(total.z, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("Coulomb: Neutral particle neither feels nor exerts force", "[coulomb]")
{
    std::vector<Particle> particles;
    particles.push_back(make_charge(Vec3f(0.0f, 0.0f, 0.0f), 1.0f));
    particles.push_back(make_charge(Vec3f(2.0f, 0.0f, 0.0f), 0.0f)); // neutral
    particles.push_back(make_charge(Vec3f(4.0f, 0.0f, 0.0f), 1.0f));

    accumulate_coulomb_forces(particles, 1.0f, 1e-3f);

    // The neutral particle feels nothing.
    REQUIRE_THAT(particles[1].force_accumulator.length(), WithinAbs(0.0f, 1e-6f));

    // The two charged particles interact only with each other (distance 4 -> 1/16),
    // exactly as if the neutral one were absent.
    REQUIRE_THAT(particles[0].force_accumulator.x, WithinAbs(-1.0f / 16.0f, 1e-6f));
    REQUIRE_THAT(particles[2].force_accumulator.x, WithinAbs(1.0f / 16.0f, 1e-6f));
}

TEST_CASE("Coulomb: Softening clamp keeps coincident charges finite", "[coulomb]")
{
    std::vector<Particle> particles;
    particles.push_back(make_charge(Vec3f(0.0f, 0.0f, 0.0f), 1.0f));
    particles.push_back(make_charge(Vec3f(0.0f, 0.0f, 0.0f), 1.0f)); // same position

    accumulate_coulomb_forces(particles, 1.0f, 1e-3f);

    // Direction is undefined at zero separation -> zero force, never NaN/Inf.
    for (const auto &p : particles)
    {
        REQUIRE(std::isfinite(p.force_accumulator.x));
        REQUIRE(std::isfinite(p.force_accumulator.y));
        REQUIRE(std::isfinite(p.force_accumulator.z));
    }
}

TEST_CASE("Coulomb: Deterministic across repeated runs", "[coulomb]")
{
    auto build = []
    {
        std::vector<Particle> particles;
        particles.push_back(make_charge(Vec3f(0.0f, 0.0f, 0.0f), 1.0f));
        particles.push_back(make_charge(Vec3f(1.5f, 0.5f, 0.0f), -2.0f));
        particles.push_back(make_charge(Vec3f(-1.0f, 1.0f, 0.5f), 0.5f));
        return particles;
    };

    std::vector<Particle> run_a = build();
    std::vector<Particle> run_b = build();

    accumulate_coulomb_forces(run_a, 2.0f, 1e-3f);
    accumulate_coulomb_forces(run_b, 2.0f, 1e-3f);

    for (size_t i = 0; i < run_a.size(); ++i)
    {
        REQUIRE(run_a[i].force_accumulator.x == run_b[i].force_accumulator.x);
        REQUIRE(run_a[i].force_accumulator.y == run_b[i].force_accumulator.y);
        REQUIRE(run_a[i].force_accumulator.z == run_b[i].force_accumulator.z);
    }
}

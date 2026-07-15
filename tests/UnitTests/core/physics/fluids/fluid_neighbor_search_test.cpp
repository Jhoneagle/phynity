#include <catch2/catch_test_macros.hpp>
#include <core/physics/fluids/fluid_neighbor_search.hpp>
#include <core/physics/fluids/sph_fluid_system.hpp>

#include <algorithm>
#include <vector>

using phynity::math::vectors::Vec3f;
using phynity::physics::fluids::FluidNeighborSearch;
using phynity::physics::fluids::SphFluidSystem;

namespace
{
bool contains(std::span<const uint32_t> s, uint32_t v)
{
    return std::find(s.begin(), s.end(), v) != s.end();
}
} // namespace

TEST_CASE("FluidNeighborSearch: finds neighbors within h, excludes beyond h", "[fluids][neighbors]")
{
    const float h = 1.0f;
    // Particle 0 at origin; 1 just inside h; 2 exactly at h (inclusive); 3 well beyond.
    std::vector<Vec3f> positions = {
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(0.5f, 0.0f, 0.0f),
        Vec3f(1.0f, 0.0f, 0.0f),
        Vec3f(5.0f, 0.0f, 0.0f),
    };

    FluidNeighborSearch search;
    search.rebuild(positions, h);

    auto n0 = search.neighbors(0);
    REQUIRE(contains(n0, 1)); // within h
    REQUIRE(contains(n0, 2)); // exactly at h (inclusive)
    REQUIRE_FALSE(contains(n0, 3)); // beyond h
    REQUIRE_FALSE(contains(n0, 0)); // self excluded
}

TEST_CASE("FluidNeighborSearch: self is never a neighbor", "[fluids][neighbors]")
{
    std::vector<Vec3f> positions = {Vec3f(0.0f), Vec3f(0.1f, 0.0f, 0.0f)};
    FluidNeighborSearch search;
    search.rebuild(positions, 1.0f);
    for (size_t i = 0; i < positions.size(); ++i)
    {
        REQUIRE_FALSE(contains(search.neighbors(i), static_cast<uint32_t>(i)));
    }
}

TEST_CASE("FluidNeighborSearch: neighbor lists are ascending (canonical order)", "[fluids][neighbors]")
{
    // A tight cluster so every particle sees every other.
    std::vector<Vec3f> positions;
    positions.reserve(8);
    for (int i = 0; i < 8; ++i)
    {
        positions.emplace_back(0.1f * static_cast<float>(i), 0.0f, 0.0f);
    }
    FluidNeighborSearch search;
    search.rebuild(positions, 2.0f);

    for (size_t i = 0; i < positions.size(); ++i)
    {
        auto n = search.neighbors(i);
        REQUIRE(std::is_sorted(n.begin(), n.end()));
    }
}

TEST_CASE("FluidNeighborSearch: neighbor lists contain no duplicates", "[fluids][neighbors]")
{
    // The underlying SpatialGrid's cell hash is not collision-free for large
    // offset coordinates, so its 27-cell gather can return a particle more than
    // once; the neighbor search must dedupe. Un-deduped lists silently inflate
    // density and break pairwise force cancellation.
    std::vector<Vec3f> positions = {
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(0.2f, 0.05f, 0.0f),
        Vec3f(-0.15f, 0.1f, 0.05f),
        Vec3f(0.1f, -0.2f, 0.1f),
        Vec3f(-0.1f, -0.1f, -0.1f),
        Vec3f(0.25f, 0.2f, -0.05f),
    };
    FluidNeighborSearch search;
    search.rebuild(positions, 1.0f);

    for (size_t i = 0; i < positions.size(); ++i)
    {
        auto n = search.neighbors(i);
        std::vector<uint32_t> copy(n.begin(), n.end());
        copy.erase(std::unique(copy.begin(), copy.end()), copy.end());
        REQUIRE(copy.size() == n.size()); // sorted + already unique
    }
}

TEST_CASE("FluidNeighborSearch: identical input yields identical ordering (determinism)", "[fluids][neighbors]")
{
    std::vector<Vec3f> positions;
    positions.reserve(20);
    for (int i = 0; i < 20; ++i)
    {
        const int col = i % 5;
        const int row = i / 5;
        positions.emplace_back(0.05f * static_cast<float>(col), 0.05f * static_cast<float>(row), 0.0f);
    }

    FluidNeighborSearch a;
    FluidNeighborSearch b;
    a.rebuild(positions, 0.3f);
    b.rebuild(positions, 0.3f);

    for (size_t i = 0; i < positions.size(); ++i)
    {
        auto na = a.neighbors(i);
        auto nb = b.neighbors(i);
        REQUIRE(na.size() == nb.size());
        REQUIRE(std::equal(na.begin(), na.end(), nb.begin()));
    }
}

TEST_CASE("SphFluidSystem: spawn, accessors, and neighbor rebuild", "[fluids][system]")
{
    SphFluidSystem system;
    system.parameters().smoothing_radius = 1.0f;
    system.spawn(Vec3f(0.0f, 0.0f, 0.0f));
    system.spawn(Vec3f(0.5f, 0.0f, 0.0f));
    system.spawn(Vec3f(10.0f, 0.0f, 0.0f));

    REQUIRE(system.particle_count() == 3);
    REQUIRE(system.particles()[0].mass == system.parameters().particle_mass);

    system.rebuild_neighbors();
    const auto &search = system.neighbor_search();
    REQUIRE(search.particle_count() == 3);
    REQUIRE(contains(search.neighbors(0), 1));
    REQUIRE_FALSE(contains(search.neighbors(0), 2)); // far particle
}

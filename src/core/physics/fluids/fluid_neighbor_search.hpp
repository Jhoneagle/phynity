#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/collision/broadphase/spatial_grid.hpp>

#include <algorithm>
#include <cstdint>
#include <span>
#include <vector>

namespace phynity::physics::fluids
{

using phynity::math::vectors::Vec3f;
using phynity::physics::collision::SpatialGrid;

/// Deterministic fixed-radius neighbor search for the fluid solvers.
///
/// A thin wrapper over the engine's particle-agnostic `SpatialGrid` (cell size =
/// smoothing radius h). Shared by both WCSPH and PBF — PBF simply rebuilds on
/// *predicted* positions each substep.
///
/// Determinism: the grid's 27-cell traversal visits cells in a fixed
/// dx,dy,dz ∈ [-1,1] order and appends each cell's insertion-ordered indices
/// (the hash map is never iterated during a query). On top of that guarantee we
/// additionally sort every neighbor list ascending by index, so the summation
/// order stays stable even if particle removal/reindexing or incremental grid
/// updates are added later — float addition is not associative, so a canonical
/// order is what keeps results reproducible.
///
/// Storage is CSR-style: a flat `neighbor_data_` buffer indexed by
/// `neighbor_offsets_[i] .. neighbor_offsets_[i+1]`, so `neighbors(i)` is a
/// zero-copy span.
class FluidNeighborSearch
{
public:
    FluidNeighborSearch() = default;

    /// Rebuild neighbor lists for the given particle positions.
    ///
    /// Clears and re-inserts every particle into the grid (cell_size = h), then
    /// for each particle gathers its 27-cell candidates, keeps only those within
    /// the smoothing radius h (excluding self), and sorts them ascending.
    ///
    /// @param positions Current particle positions (index i is particle i)
    /// @param h         Smoothing radius (also the grid cell size)
    void rebuild(const std::vector<Vec3f> &positions, float h)
    {
        const size_t count = positions.size();
        smoothing_radius_ = h;

        grid_.set_cell_size(h);
        grid_.clear();
        for (size_t i = 0; i < count; ++i)
        {
            grid_.insert(static_cast<uint32_t>(i), positions[i]);
        }

        neighbor_offsets_.assign(count + 1, 0);
        neighbor_data_.clear();

        const float h2 = h * h;
        for (size_t i = 0; i < count; ++i)
        {
            const auto candidates = grid_.get_neighbor_objects(positions[i]);

            const size_t run_start = neighbor_data_.size();
            for (const uint32_t j : candidates)
            {
                if (j == i)
                {
                    continue; // skip self
                }
                const float r2 = (positions[i] - positions[j]).squaredLength();
                if (r2 <= h2)
                {
                    neighbor_data_.push_back(j);
                }
            }

            // Canonical ascending order for a stable summation order.
            std::sort(neighbor_data_.begin() + static_cast<std::ptrdiff_t>(run_start), neighbor_data_.end());
            neighbor_offsets_[i + 1] = neighbor_data_.size();
        }
    }

    /// Neighbors of particle i (indices only, ascending), as a zero-copy span.
    [[nodiscard]] std::span<const uint32_t> neighbors(size_t i) const
    {
        const size_t begin = neighbor_offsets_[i];
        const size_t end = neighbor_offsets_[i + 1];
        return std::span<const uint32_t>(neighbor_data_.data() + begin, end - begin);
    }

    /// Number of particles the last rebuild covered.
    [[nodiscard]] size_t particle_count() const noexcept
    {
        return neighbor_offsets_.empty() ? 0 : neighbor_offsets_.size() - 1;
    }

    /// Smoothing radius used by the last rebuild.
    [[nodiscard]] float smoothing_radius() const noexcept
    {
        return smoothing_radius_;
    }

private:
    SpatialGrid grid_{1.0f};
    std::vector<uint32_t> neighbor_data_;   ///< Flat CSR neighbor indices
    std::vector<size_t> neighbor_offsets_;  ///< CSR row offsets (size = count + 1)
    float smoothing_radius_{0.0f};
};

} // namespace phynity::physics::fluids

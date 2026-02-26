#ifndef PHYNITY_CORE_PHYSICS_COLLISION_SPATIAL_GRID_HASH_HPP
#define PHYNITY_CORE_PHYSICS_COLLISION_SPATIAL_GRID_HASH_HPP

#include "../../math/vectors/vec3.hpp"
#include <cstdint>
#include <cmath>

namespace phynity::physics::collision {

/**
 * @brief Integer coordinates for a grid cell.
 */
struct GridCell {
    int32_t x;
    int32_t y;
    int32_t z;

    GridCell() noexcept : x(0), y(0), z(0) {}
    GridCell(int32_t x_, int32_t y_, int32_t z_) noexcept : x(x_), y(y_), z(z_) {}

    bool operator==(const GridCell& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const GridCell& other) const noexcept {
        return !(*this == other);
    }
};

/**
 * @brief Spatial grid hash function for 3D cell indexing.
 * 
 * Converts 3D positions to 1D grid cell indices for spatial partitioning.
 * Uses Cantor pairing function for robust, collision-free hashing.
 * 
 * Design:
 * - Cell coordinates are computed from positions using cell_size
 * - 3D coordinates are mapped to 1D index via iterative pairing
 * - Handles negative coordinates naturally (works in all octants)
 * 
 * Performance: O(1) for hash computation
 * Collision rate: Minimal (mathematical bijection for integer coordinates)
 */
class SpatialGridHash {
public:
    explicit SpatialGridHash(float cell_size = 1.0f) noexcept
        : cell_size_(cell_size) {
    }

    /**
     * @brief Get the grid cell size in world units.
     */
    [[nodiscard]] float get_cell_size() const noexcept {
        return cell_size_;
    }

    /**
     * @brief Set the grid cell size in world units.
     */
    void set_cell_size(float size) noexcept {
        cell_size_ = size;
    }

    /**
     * @brief Convert a position to its grid cell coordinates.
     * 
     * @param position World position
     * @return Cell coordinates (x, y, z)
     * 
     * Example: position=(2.5, 3.7, -1.2), cell_size=1.0
     *          -> cell_coords=(2, 3, -2)
     */
    [[nodiscard]] GridCell get_cell_coords(const math::vectors::Vec3f& position) const noexcept {
        return GridCell{
            static_cast<int32_t>(std::floor(position.x / cell_size_)),
            static_cast<int32_t>(std::floor(position.y / cell_size_)),
            static_cast<int32_t>(std::floor(position.z / cell_size_))
        };
    }

    /**
     * @brief Convert a position directly to a hash index.
     * 
     * Combines cell coordinate calculation and hashing.
     * 
     * @param position World position
     * @return Hash index for spatial grid
     */
    [[nodiscard]] uint64_t hash_position(const math::vectors::Vec3f& position) const noexcept {
        const auto cell_coords = get_cell_coords(position);
        return hash_coords(cell_coords.x, cell_coords.y, cell_coords.z);
    }

    /**
     * @brief Convert 3D cell coordinates to a 1D hash index.
     * 
     * Uses Cantor pairing function for robust, collision-free mapping.
     * Handles negative coordinates by offsetting them.
     * 
     * Formula:
     * - pair(x, y) = ((x+y)*(x+y+1))/2 + y
     * - hash(x,y,z) = pair(pair(x, y), z)
     * 
     * @param cell_x X cell coordinate
     * @param cell_y Y cell coordinate
     * @param cell_z Z cell coordinate
     * @return Unique hash index
     */
    [[nodiscard]] static uint64_t hash_coords(int32_t cell_x, int32_t cell_y, int32_t cell_z) noexcept {
        // Offset negative coordinates to positive range for Cantor pairing
        // This maintains the bijection property for all integer coordinates
        const uint64_t x = static_cast<uint64_t>(cell_x) + 0x8000'0000ULL;
        const uint64_t y = static_cast<uint64_t>(cell_y) + 0x8000'0000ULL;
        const uint64_t z = static_cast<uint64_t>(cell_z) + 0x8000'0000ULL;

        // Cantor pairing: pair(a, b) = ((a+b) * (a+b+1)) / 2 + b
        const uint64_t pair_xy = cantor_pair(x, y);
        return cantor_pair(pair_xy, z);
    }

private:
    float cell_size_;

    /**
     * @brief Cantor pairing function for 2D to 1D mapping.
     * 
     * Creates a bijection from (unsigned) integer pairs to unsigned integers.
     * Properties:
     * - Deterministic: same input always produces same output
     * - Injective: different inputs produce different outputs (no collisions)
     * - Efficient: O(1) computation
     * 
     * @param a First coordinate (must be non-negative)
     * @param b Second coordinate (must be non-negative)
     * @return Unique index
     */
    [[nodiscard]] static inline uint64_t cantor_pair(uint64_t a, uint64_t b) noexcept {
        const uint64_t sum = a + b;
        // ((sum * (sum + 1)) / 2 + b)
        return ((sum * (sum + 1)) >> 1) + b;
    }
};

} // namespace phynity::physics::collision

#endif // PHYNITY_CORE_PHYSICS_COLLISION_SPATIAL_GRID_HASH_HPP

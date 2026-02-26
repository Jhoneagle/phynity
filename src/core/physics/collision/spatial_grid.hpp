#ifndef PHYNITY_CORE_PHYSICS_COLLISION_SPATIAL_GRID_HPP
#define PHYNITY_CORE_PHYSICS_COLLISION_SPATIAL_GRID_HPP

#include "spatial_grid_hash.hpp"
#include "../../math/vectors/vec3.hpp"
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <algorithm>

namespace phynity::physics::collision {

/**
 * @brief Cell in the spatial grid containing object indices.
 * 
 * Each cell stores indices of objects that occupy that grid cell.
 * Indices are external IDs provided by the caller (e.g., particle system indices).
 */
struct GridCellData {
    std::vector<uint32_t> object_indices;

    void clear() noexcept {
        object_indices.clear();
    }

    [[nodiscard]] bool empty() const noexcept {
        return object_indices.empty();
    }

    [[nodiscard]] size_t size() const noexcept {
        return object_indices.size();
    }
};

/**
 * @brief Spatial grid for broad-phase collision detection.
 * 
 * Partitions 3D space into uniform cells and indices objects by grid cell.
 * Enables efficient querying of nearby objects for collision detection.
 * 
 * Design:
 * - Hash-based storage (unordered_map) for sparse occupancy
 * - Dynamic insertion/removal/updating of objects
 * - Query methods for adjacent cells and ray casting
 * - Per-frame updates: clear and re-insert objects
 * 
 * Usage Pattern:
 * 1. create_spatial_grid(cell_size)
 * 2. Each frame: update(), then query_candidates()
 * 3. insert() objects at their current positions
 * 4. get_objects_in_cell() and get_neighbors() for collision pairs
 * 
 * Performance:
 * - Insertion: O(1) average per object
 * - Query: O(cell_height³ * objects_per_cell) for neighbor cells
 * - Clear: O(cells_occupied)
 */
class SpatialGrid {
public:
    explicit SpatialGrid(float cell_size = 1.0f) noexcept
        : hash_(cell_size) {
    }

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Get the current cell size in world units.
     */
    [[nodiscard]] float get_cell_size() const noexcept {
        return hash_.get_cell_size();
    }

    /**
     * @brief Set the cell size in world units.
     * 
     * Note: Does not clear existing objects. Call clear() if needed.
     * Typically set before the first insertion.
     */
    void set_cell_size(float size) noexcept {
        hash_.set_cell_size(size);
    }

    // ========================================================================
    // Object Management
    // ========================================================================

    /**
     * @brief Insert an object at the given world position.
     * 
     * Maps the object to the grid cell containing the position.
     * If object_id is already in grid, its cell is updated.
     * 
     * @param object_id Unique identifier for the object (external ID)
     * @param position World position of the object
     */
    void insert(uint32_t object_id, const math::vectors::Vec3f& position) noexcept {
        const auto cell_hash = hash_.hash_position(position);
        cells_[cell_hash].object_indices.push_back(object_id);
    }

    /**
     * @brief Update an object's position in the grid.
     * 
     * Removes object from current cell and re-inserts at new position.
     * 
     * @param object_id Object to update
     * @param position New world position
     */
    void update(uint32_t object_id, const math::vectors::Vec3f& position) noexcept {
        remove(object_id);
        insert(object_id, position);
    }

    /**
     * @brief Remove an object from the grid.
     * 
     * Searches all cells for the object and removes it.
     * Does nothing if object is not found.
     * 
     * @param object_id Object to remove
     */
    void remove(uint32_t object_id) noexcept {
        for (auto& [hash, cell] : cells_) {
            auto& indices = cell.object_indices;
            auto it = std::find(indices.begin(), indices.end(), object_id);
            if (it != indices.end()) {
                indices.erase(it);
                // Clean up empty cells
                if (cell.empty()) {
                    cells_.erase(hash);
                }
                return;
            }
        }
    }

    /**
     * @brief Clear all objects from the grid.
     * 
     * Typically called at the start of each frame before re-inserting objects.
     */
    void clear() noexcept {
        cells_.clear();
    }

    // ========================================================================
    // Querying
    // ========================================================================

    /**
     * @brief Get all objects in a specific grid cell.
     * 
     * @param position World position (will be converted to cell)
     * @return Vector of object IDs in that cell (empty if cell unoccupied)
     */
    [[nodiscard]] std::vector<uint32_t> get_objects_in_cell(
        const math::vectors::Vec3f& position) const noexcept {
        const auto cell_hash = hash_.hash_position(position);
        auto it = cells_.find(cell_hash);
        if (it != cells_.end()) {
            return it->second.object_indices;
        }
        return {};
    }

    /**
     * @brief Get all objects in neighboring cells (3×3×3 cube).
     * 
     * Returns objects in the queried cell plus the 26 adjacent cells.
     * Useful for finding potential collision candidates.
     * 
     * @param position World position (will be converted to cell)
     * @return Vector of object IDs in neighboring cells
     * 
     * Note: May contain duplicates if same object spans multiple cells
     *       (caller should deduplicate for AABB-based broadphase)
     */
    [[nodiscard]] std::vector<uint32_t> get_neighbor_objects(
        const math::vectors::Vec3f& position) const noexcept {
        const auto center_cell = hash_.get_cell_coords(position);
        return get_neighbor_objects_from_cell(center_cell);
    }

    /**
     * @brief Get all objects in neighboring cells from a grid cell.
     * 
     * Retrieves objects from the 3×3×3 neighborhood around cell (cx, cy, cz).
     * 
     * @param cell Grid cell coordinates
     * @return Vector of object IDs in neighboring cells
     */
    [[nodiscard]] std::vector<uint32_t> get_neighbor_objects_from_cell(
        const GridCell& cell) const noexcept {
        std::vector<uint32_t> neighbors;

        // Iterate through 3×3×3 neighborhood
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const auto neighbor_cell = GridCell{
                        cell.x + dx,
                        cell.y + dy,
                        cell.z + dz
                    };
                    const auto neighbor_hash = SpatialGridHash::hash_coords(
                        neighbor_cell.x, neighbor_cell.y, neighbor_cell.z);

                    auto it = cells_.find(neighbor_hash);
                    if (it != cells_.end()) {
                        const auto& indices = it->second.object_indices;
                        neighbors.insert(neighbors.end(), indices.begin(), indices.end());
                    }
                }
            }
        }

        return neighbors;
    }

    /**
     * @brief Get a list of all occupied cells.
     * 
     * Useful for debugging and visualizing grid occupancy.
     * 
     * @return Vector of occupied cell hashes
     */
    [[nodiscard]] std::vector<uint64_t> get_occupied_cells() const noexcept {
        std::vector<uint64_t> occupied;
        occupied.reserve(cells_.size());
        for (const auto& [hash, cell] : cells_) {
            occupied.push_back(hash);
        }
        return occupied;
    }

    /**
     * @brief Get the number of occupied cells.
     */
    [[nodiscard]] size_t get_cell_count() const noexcept {
        return cells_.size();
    }

    /**
     * @brief Get the total number of objects in the grid.
     */
    [[nodiscard]] size_t get_object_count() const noexcept {
        size_t count = 0;
        for (const auto& [hash, cell] : cells_) {
            count += cell.size();
        }
        return count;
    }

private:
    SpatialGridHash hash_;
    std::unordered_map<uint64_t, GridCellData> cells_;
};

} // namespace phynity::physics::collision

#endif // PHYNITY_CORE_PHYSICS_COLLISION_SPATIAL_GRID_HPP

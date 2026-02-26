#include <catch2/catch_all.hpp>
#include "../../../../../src/core/physics/collision/spatial_grid.hpp"
#include <tests/test_utils/physics_test_helpers.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;

TEST_CASE("SpatialGrid::Basic Construction and Configuration", "[physics][collision]") {
    SECTION("Default cell size is 1.0") {
        const SpatialGrid grid;
        REQUIRE_THAT(grid.get_cell_size(), Catch::Matchers::WithinAbs(1.0f, tolerance::STRICT));
    }

    SECTION("Custom cell size in constructor") {
        const SpatialGrid grid(2.5f);
        REQUIRE_THAT(grid.get_cell_size(), Catch::Matchers::WithinAbs(2.5f, tolerance::STRICT));
    }

    SECTION("Cell size can be changed") {
        SpatialGrid grid(1.0f);
        grid.set_cell_size(0.5f);
        REQUIRE_THAT(grid.get_cell_size(), Catch::Matchers::WithinAbs(0.5f, tolerance::STRICT));
    }

    SECTION("New grid is empty") {
        const SpatialGrid grid;
        REQUIRE(grid.get_cell_count() == 0);
        REQUIRE(grid.get_object_count() == 0);
    }
}

TEST_CASE("SpatialGrid::Insertion", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Insert single object") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        REQUIRE(grid.get_cell_count() == 1);
        REQUIRE(grid.get_object_count() == 1);
    }

    SECTION("Insert multiple objects in same cell") {
        grid.insert(0, Vec3f{0.3f, 0.4f, 0.2f});
        grid.insert(1, Vec3f{0.7f, 0.6f, 0.8f});
        grid.insert(2, Vec3f{0.5f, 0.5f, 0.5f});
        
        REQUIRE(grid.get_cell_count() == 1); // All in cell (0,0,0)
        REQUIRE(grid.get_object_count() == 3);
    }

    SECTION("Insert objects in different cells") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});   // Cell (0,0,0)
        grid.insert(1, Vec3f{1.5f, 0.5f, 0.5f});   // Cell (1,0,0)
        grid.insert(2, Vec3f{0.5f, 1.5f, 0.5f});   // Cell (0,1,0)
        grid.insert(3, Vec3f{0.5f, 0.5f, 1.5f});   // Cell (0,0,1)
        
        REQUIRE(grid.get_cell_count() == 4);
        REQUIRE(grid.get_object_count() == 4);
    }

    SECTION("Insert same object multiple times adds duplicates") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        
        REQUIRE(grid.get_cell_count() == 1);
        REQUIRE(grid.get_object_count() == 2); // Duplicates are allowed
    }
}

TEST_CASE("SpatialGrid::Get Objects in Cell", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Get objects from occupied cell") {
        grid.insert(5, Vec3f{0.5f, 0.5f, 0.5f});
        grid.insert(7, Vec3f{0.3f, 0.7f, 0.2f});
        
        const auto objects = grid.get_objects_in_cell(Vec3f{0.6f, 0.4f, 0.5f});
        REQUIRE(objects.size() == 2);
        REQUIRE(std::find(objects.begin(), objects.end(), 5u) != objects.end());
        REQUIRE(std::find(objects.begin(), objects.end(), 7u) != objects.end());
    }

    SECTION("Get objects from empty cell") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        
        const auto objects = grid.get_objects_in_cell(Vec3f{5.0f, 5.0f, 5.0f});
        REQUIRE(objects.empty());
    }

    SECTION("Query at cell boundary returns correct cell") {
        grid.insert(0, Vec3f{0.99f, 0.5f, 0.5f}); // Cell (0,0,0)
        grid.insert(1, Vec3f{1.01f, 0.5f, 0.5f}); // Cell (1,0,0)
        
        const auto objects_a = grid.get_objects_in_cell(Vec3f{0.99f, 0.5f, 0.5f});
        const auto objects_b = grid.get_objects_in_cell(Vec3f{1.01f, 0.5f, 0.5f});
        
        REQUIRE(objects_a.size() == 1);
        REQUIRE(objects_b.size() == 1);
        REQUIRE(objects_a[0] == 0);
        REQUIRE(objects_b[0] == 1);
    }
}

TEST_CASE("SpatialGrid::Removal", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Remove object from grid") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        grid.insert(1, Vec3f{0.5f, 0.5f, 0.5f});
        
        grid.remove(0);
        
        REQUIRE(grid.get_cell_count() == 1);
        REQUIRE(grid.get_object_count() == 1);
        
        const auto objects = grid.get_objects_in_cell(Vec3f{0.5f, 0.5f, 0.5f});
        REQUIRE(objects.size() == 1);
        REQUIRE(objects[0] == 1);
    }

    SECTION("Remove last object from cell cleans up empty cell") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        
        grid.remove(0);
        
        REQUIRE(grid.get_cell_count() == 0);
        REQUIRE(grid.get_object_count() == 0);
    }

    SECTION("Remove non-existent object does nothing") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        
        grid.remove(999);
        
        REQUIRE(grid.get_cell_count() == 1);
        REQUIRE(grid.get_object_count() == 1);
    }

    SECTION("Remove object from multi-cell grid") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});   // Cell (0,0,0)
        grid.insert(1, Vec3f{1.5f, 0.5f, 0.5f});   // Cell (1,0,0)
        grid.insert(2, Vec3f{0.5f, 1.5f, 0.5f});   // Cell (0,1,0)
        
        grid.remove(1);
        
        REQUIRE(grid.get_cell_count() == 2);
        REQUIRE(grid.get_object_count() == 2);
    }
}

TEST_CASE("SpatialGrid::Update", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Update object to new cell") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        
        grid.update(0, Vec3f{1.5f, 0.5f, 0.5f});
        
        REQUIRE(grid.get_cell_count() == 1);
        REQUIRE(grid.get_object_count() == 1);
        
        const auto old_cell = grid.get_objects_in_cell(Vec3f{0.5f, 0.5f, 0.5f});
        const auto new_cell = grid.get_objects_in_cell(Vec3f{1.5f, 0.5f, 0.5f});
        
        REQUIRE(old_cell.empty());
        REQUIRE(new_cell.size() == 1);
        REQUIRE(new_cell[0] == 0);
    }

    SECTION("Update object within same cell") {
        grid.insert(0, Vec3f{0.3f, 0.3f, 0.3f});
        
        grid.update(0, Vec3f{0.7f, 0.7f, 0.7f});
        
        REQUIRE(grid.get_cell_count() == 1);
        REQUIRE(grid.get_object_count() == 1);
    }
}

TEST_CASE("SpatialGrid::Clear", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Clear empty grid") {
        grid.clear();
        REQUIRE(grid.get_cell_count() == 0);
        REQUIRE(grid.get_object_count() == 0);
    }

    SECTION("Clear populated grid") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        grid.insert(1, Vec3f{1.5f, 1.5f, 1.5f});
        grid.insert(2, Vec3f{-0.5f, -0.5f, -0.5f});
        
        grid.clear();
        
        REQUIRE(grid.get_cell_count() == 0);
        REQUIRE(grid.get_object_count() == 0);
    }
}

TEST_CASE("SpatialGrid::Get Neighbor Objects", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Single object has no neighbors") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        
        const auto neighbors = grid.get_neighbor_objects(Vec3f{0.5f, 0.5f, 0.5f});
        REQUIRE(neighbors.size() == 1);
        REQUIRE(neighbors[0] == 0);
    }

    SECTION("Adjacent objects are neighbors") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});   // Cell (0,0,0)
        grid.insert(1, Vec3f{1.5f, 0.5f, 0.5f});   // Cell (1,0,0) - adjacent
        
        const auto neighbors = grid.get_neighbor_objects(Vec3f{0.5f, 0.5f, 0.5f});
        REQUIRE(neighbors.size() == 2);
        REQUIRE(std::find(neighbors.begin(), neighbors.end(), 0u) != neighbors.end());
        REQUIRE(std::find(neighbors.begin(), neighbors.end(), 1u) != neighbors.end());
    }

    SECTION("3×3×3 neighborhood retrieval") {
        // Insert objects in all 8 corners of a 2×2×2 block of cells
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});    // Cell (0,0,0)
        grid.insert(1, Vec3f{1.5f, 0.5f, 0.5f});    // Cell (1,0,0)
        grid.insert(2, Vec3f{0.5f, 1.5f, 0.5f});    // Cell (0,1,0)
        grid.insert(3, Vec3f{1.5f, 1.5f, 0.5f});    // Cell (1,1,0)
        grid.insert(4, Vec3f{0.5f, 0.5f, 1.5f});    // Cell (0,0,1)
        grid.insert(5, Vec3f{1.5f, 0.5f, 1.5f});    // Cell (1,0,1)
        grid.insert(6, Vec3f{0.5f, 1.5f, 1.5f});    // Cell (0,1,1)
        grid.insert(7, Vec3f{1.5f, 1.5f, 1.5f});    // Cell (1,1,1)
        
        const auto neighbors = grid.get_neighbor_objects(Vec3f{0.5f, 0.5f, 0.5f});
        
        // All 8 objects should be in the 3×3×3 neighborhood
        REQUIRE(neighbors.size() == 8);
        for (uint32_t i = 0; i < 8; ++i) {
            REQUIRE(std::find(neighbors.begin(), neighbors.end(), i) != neighbors.end());
        }
    }

    SECTION("Distant objects are not neighbors") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});    // Cell (0,0,0)
        grid.insert(1, Vec3f{5.5f, 5.5f, 5.5f});    // Cell (5,5,5) - far away
        
        const auto neighbors = grid.get_neighbor_objects(Vec3f{0.5f, 0.5f, 0.5f});
        REQUIRE(neighbors.size() == 1);
        REQUIRE(neighbors[0] == 0);
    }
}

TEST_CASE("SpatialGrid::Get Occupied Cells", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Empty grid has no occupied cells") {
        const auto occupied = grid.get_occupied_cells();
        REQUIRE(occupied.empty());
    }

    SECTION("Occupied cells are tracked") {
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});
        grid.insert(1, Vec3f{1.5f, 0.5f, 0.5f});
        grid.insert(2, Vec3f{2.5f, 0.5f, 0.5f});
        
        const auto occupied = grid.get_occupied_cells();
        REQUIRE(occupied.size() == 3);
    }
}

TEST_CASE("SpatialGrid::Large Scale Operations", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Insert 100 objects") {
        for (int i = 0; i < 100; ++i) {
            const float x = static_cast<float>(i % 10);
            const float y = static_cast<float>((i / 10) % 10);
            const float z = 0.5f;
            grid.insert(i, Vec3f{x + 0.5f, y + 0.5f, z});
        }
        
        REQUIRE(grid.get_object_count() == 100);
        REQUIRE(grid.get_cell_count() == 100); // One object per cell
    }

    SECTION("Insert 1000 objects in same cell") {
        for (int i = 0; i < 1000; ++i) {
            grid.insert(i, Vec3f{0.5f, 0.5f, 0.5f});
        }
        
        REQUIRE(grid.get_object_count() == 1000);
        REQUIRE(grid.get_cell_count() == 1);
    }

    SECTION("Update and remove many objects") {
        // Insert
        for (int i = 0; i < 50; ++i) {
            grid.insert(i, Vec3f{0.5f, 0.5f, 0.5f});
        }
        REQUIRE(grid.get_object_count() == 50);
        
        // Update some
        for (int i = 0; i < 25; ++i) {
            grid.update(i, Vec3f{1.5f, 0.5f, 0.5f});
        }
        REQUIRE(grid.get_cell_count() == 2); // Two cells occupied
        
        // Remove some
        for (int i = 0; i < 10; ++i) {
            grid.remove(i);
        }
        REQUIRE(grid.get_object_count() == 40);
    }
}

TEST_CASE("SpatialGrid::Negative Coordinates", "[physics][collision]") {
    SpatialGrid grid(1.0f);

    SECTION("Objects in negative space") {
        grid.insert(0, Vec3f{-0.5f, -0.5f, -0.5f});  // Cell (-1,-1,-1)
        grid.insert(1, Vec3f{-1.5f, -1.5f, -1.5f});  // Cell (-2,-2,-2)
        
        REQUIRE(grid.get_cell_count() == 2);
        REQUIRE(grid.get_object_count() == 2);
    }

    SECTION("Query negative space") {
        grid.insert(0, Vec3f{-0.5f, -0.5f, -0.5f});
        
        const auto objects = grid.get_objects_in_cell(Vec3f{-0.7f, -0.3f, -0.2f});
        REQUIRE(objects.size() == 1);
        REQUIRE(objects[0] == 0);
    }

    SECTION("Neighbors across origin") {
        grid.insert(0, Vec3f{-0.5f, 0.5f, 0.5f});   // Cell (-1,0,0)
        grid.insert(1, Vec3f{0.5f, 0.5f, 0.5f});    // Cell (0,0,0)
        
        const auto neighbors = grid.get_neighbor_objects(Vec3f{0.1f, 0.5f, 0.5f});
        REQUIRE(neighbors.size() == 2);
    }
}

TEST_CASE("SpatialGrid::Different Cell Sizes", "[physics][collision]") {
    SECTION("Coarse cell size (2.0)") {
        SpatialGrid grid(2.0f);
        
        grid.insert(0, Vec3f{0.5f, 0.5f, 0.5f});    // All in cell (0,0,0)
        grid.insert(1, Vec3f{1.9f, 1.9f, 1.9f});
        grid.insert(2, Vec3f{0.1f, 0.1f, 0.1f});
        
        REQUIRE(grid.get_cell_count() == 1);
        REQUIRE(grid.get_object_count() == 3);
    }

    SECTION("Fine cell size (0.5)") {
        SpatialGrid grid(0.5f);
        
        grid.insert(0, Vec3f{0.2f, 0.2f, 0.2f});    // Cell (0,0,0)
        grid.insert(1, Vec3f{0.7f, 0.7f, 0.7f});    // Cell (1,1,1)
        grid.insert(2, Vec3f{0.9f, 0.9f, 0.9f});    // Cell (1,1,1)
        
        REQUIRE(grid.get_cell_count() == 2);
    }
}

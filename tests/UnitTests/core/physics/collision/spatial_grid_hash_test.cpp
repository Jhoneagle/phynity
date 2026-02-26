#include <catch2/catch_all.hpp>
#include "../../../../../src/core/physics/collision/spatial_grid_hash.hpp"
#include <tests/test_utils/physics_test_helpers.hpp>
#include <unordered_set>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;

TEST_CASE("SpatialGridHash::get_cell_coords", "[physics][collision]") {
    const SpatialGridHash hasher(1.0f);

    SECTION("Origin maps to cell (0,0,0)") {
        const auto cell = hasher.get_cell_coords(Vec3f{0.0f, 0.0f, 0.0f});
        REQUIRE(cell.x == 0);
        REQUIRE(cell.y == 0);
        REQUIRE(cell.z == 0);
    }

    SECTION("Positive coordinates") {
        const auto cell = hasher.get_cell_coords(Vec3f{2.5f, 3.7f, 1.2f});
        REQUIRE(cell.x == 2);
        REQUIRE(cell.y == 3);
        REQUIRE(cell.z == 1);
    }

    SECTION("Negative coordinates") {
        const auto cell = hasher.get_cell_coords(Vec3f{-2.5f, -3.7f, -1.2f});
        REQUIRE(cell.x == -3);
        REQUIRE(cell.y == -4);
        REQUIRE(cell.z == -2);
    }

    SECTION("Mixed positive and negative") {
        const auto cell = hasher.get_cell_coords(Vec3f{5.3f, -2.1f, 0.9f});
        REQUIRE(cell.x == 5);
        REQUIRE(cell.y == -3);
        REQUIRE(cell.z == 0);
    }

    SECTION("Custom cell size") {
        const SpatialGridHash hasher2(2.0f);
        const auto cell = hasher2.get_cell_coords(Vec3f{4.5f, 5.9f, 3.1f});
        REQUIRE(cell.x == 2);
        REQUIRE(cell.y == 2);
        REQUIRE(cell.z == 1);
    }

    SECTION("Cell boundaries") {
        const auto cell1 = hasher.get_cell_coords(Vec3f{0.99f, 0.99f, 0.99f});
        const auto cell2 = hasher.get_cell_coords(Vec3f{1.0f, 1.0f, 1.0f});
        REQUIRE(cell1.x == 0);
        REQUIRE(cell1.y == 0);
        REQUIRE(cell1.z == 0);
        REQUIRE(cell2.x == 1);
        REQUIRE(cell2.y == 1);
        REQUIRE(cell2.z == 1);
    }
}

TEST_CASE("SpatialGridHash::hash_coords", "[physics][collision]") {
    SECTION("Same coordinates produce same hash") {
        const auto hash1 = SpatialGridHash::hash_coords(1, 2, 3);
        const auto hash2 = SpatialGridHash::hash_coords(1, 2, 3);
        REQUIRE(hash1 == hash2);
    }

    SECTION("Different coordinates produce different hashes") {
        const auto hash1 = SpatialGridHash::hash_coords(1, 2, 3);
        const auto hash2 = SpatialGridHash::hash_coords(1, 2, 4);
        const auto hash3 = SpatialGridHash::hash_coords(1, 3, 3);
        const auto hash4 = SpatialGridHash::hash_coords(2, 2, 3);
        
        REQUIRE(hash1 != hash2);
        REQUIRE(hash1 != hash3);
        REQUIRE(hash1 != hash4);
        REQUIRE(hash2 != hash3);
        REQUIRE(hash2 != hash4);
        REQUIRE(hash3 != hash4);
    }

    SECTION("Origin hashing") {
        const auto hash = SpatialGridHash::hash_coords(0, 0, 0);
        REQUIRE(hash != 0); // Should be a valid hash value
    }

    SECTION("Negative coordinates can be hashed") {
        const auto hash1 = SpatialGridHash::hash_coords(-1, -2, -3);
        const auto hash2 = SpatialGridHash::hash_coords(-1, -2, -3);
        REQUIRE(hash1 == hash2);
    }

    SECTION("Negative and positive coordinates produce different hashes") {
        const auto hash_pos = SpatialGridHash::hash_coords(1, 2, 3);
        const auto hash_neg = SpatialGridHash::hash_coords(-1, -2, -3);
        REQUIRE(hash_pos != hash_neg);
    }

    SECTION("Large positive coordinates") {
        const auto hash1 = SpatialGridHash::hash_coords(1000, 2000, 3000);
        const auto hash2 = SpatialGridHash::hash_coords(1000, 2000, 3000);
        REQUIRE(hash1 == hash2);
    }

    SECTION("Large negative coordinates") {
        const auto hash1 = SpatialGridHash::hash_coords(-1000, -2000, -3000);
        const auto hash2 = SpatialGridHash::hash_coords(-1000, -2000, -3000);
        REQUIRE(hash1 == hash2);
    }

    SECTION("Symmetry property - permutations produce different hashes") {
        const auto hash_123 = SpatialGridHash::hash_coords(1, 2, 3);
        const auto hash_132 = SpatialGridHash::hash_coords(1, 3, 2);
        const auto hash_213 = SpatialGridHash::hash_coords(2, 1, 3);
        
        REQUIRE(hash_123 != hash_132);
        REQUIRE(hash_123 != hash_213);
        REQUIRE(hash_132 != hash_213);
    }
}

TEST_CASE("SpatialGridHash::hash_position", "[physics][collision]") {
    const SpatialGridHash hasher(1.0f);

    SECTION("Same position produces same hash") {
        const auto hash1 = hasher.hash_position(Vec3f{2.5f, 3.7f, 1.2f});
        const auto hash2 = hasher.hash_position(Vec3f{2.5f, 3.7f, 1.2f});
        REQUIRE(hash1 == hash2);
    }

    SECTION("Nearby positions in same cell produce same hash") {
        const auto hash1 = hasher.hash_position(Vec3f{2.1f, 3.3f, 1.9f});
        const auto hash2 = hasher.hash_position(Vec3f{2.9f, 3.9f, 1.1f});
        REQUIRE(hash1 == hash2); // Both in cell (2,3,1)
    }

    SECTION("Positions in different cells produce different hashes") {
        const auto hash1 = hasher.hash_position(Vec3f{0.5f, 0.5f, 0.5f});
        const auto hash2 = hasher.hash_position(Vec3f{1.5f, 0.5f, 0.5f});
        const auto hash3 = hasher.hash_position(Vec3f{0.5f, 1.5f, 0.5f});
        const auto hash4 = hasher.hash_position(Vec3f{0.5f, 0.5f, 1.5f});
        
        REQUIRE(hash1 != hash2);
        REQUIRE(hash1 != hash3);
        REQUIRE(hash1 != hash4);
    }

    SECTION("Negative positions work correctly") {
        const auto hash1 = hasher.hash_position(Vec3f{-1.5f, -2.5f, -3.5f});
        const auto hash2 = hasher.hash_position(Vec3f{-1.1f, -2.9f, -3.1f});
        REQUIRE(hash1 == hash2); // Both in cell (-2,-3,-4)
    }

    SECTION("Custom cell size affects hashing") {
        const SpatialGridHash hasher2(2.0f);
        const auto hash1 = hasher.hash_position(Vec3f{1.5f, 1.5f, 1.5f});   // Cell size 1.0 -> (1,1,1)
        const auto hash2 = hasher2.hash_position(Vec3f{1.5f, 1.5f, 1.5f}); // Cell size 2.0 -> (0,0,0)
        REQUIRE(hash1 != hash2);
    }

    SECTION("Cell boundary positions in different cells") {
        const auto hash1 = hasher.hash_position(Vec3f{0.99f, 0.99f, 0.99f});
        const auto hash2 = hasher.hash_position(Vec3f{1.01f, 1.01f, 1.01f});
        REQUIRE(hash1 != hash2);
    }
}

TEST_CASE("SpatialGridHash::Grid cell indexing in 1D grid", "[physics][collision]") {
    const SpatialGridHash hasher(1.0f);

    SECTION("10x10x10 grid produces unique hashes") {
        std::unordered_set<uint64_t> hashes;
        int collision_count = 0;

        for (int x = 0; x < 10; ++x) {
            for (int y = 0; y < 10; ++y) {
                for (int z = 0; z < 10; ++z) {
                    const auto hash = SpatialGridHash::hash_coords(x, y, z);
                    if (hashes.count(hash) > 0) {
                        collision_count++;
                    }
                    hashes.insert(hash);
                }
            }
        }

        REQUIRE(collision_count == 0);
        REQUIRE(hashes.size() == 1000);
    }

    SECTION("Grid expansion: each cell in octants are unique") {
        // Test cells in all octants
        const auto hash_ppp = SpatialGridHash::hash_coords(1, 1, 1);   // +,+,+
        const auto hash_npp = SpatialGridHash::hash_coords(-1, 1, 1);  // -,+,+
        const auto hash_pnp = SpatialGridHash::hash_coords(1, -1, 1);  // +,-,+
        const auto hash_ppn = SpatialGridHash::hash_coords(1, 1, -1);  // +,+,-
        const auto hash_nnp = SpatialGridHash::hash_coords(-1, -1, 1); // -,-,+
        const auto hash_npn = SpatialGridHash::hash_coords(-1, 1, -1); // -,+,-
        const auto hash_pnn = SpatialGridHash::hash_coords(1, -1, -1); // +,-,-
        const auto hash_nnn = SpatialGridHash::hash_coords(-1, -1, -1);// -,-,-

        std::unordered_set<uint64_t> hashes{
            hash_ppp, hash_npp, hash_pnp, hash_ppn,
            hash_nnp, hash_npn, hash_pnn, hash_nnn
        };

        REQUIRE(hashes.size() == 8); // All octants have unique hashes
    }
}

TEST_CASE("SpatialGridHash::Cell size configuration", "[physics][collision]") {
    SECTION("Default cell size is 1.0") {
        const SpatialGridHash hasher;
        REQUIRE_THAT(hasher.get_cell_size(), WithinAbs(1.0f, tolerance::STRICT));
    }

    SECTION("Custom cell size in constructor") {
        const SpatialGridHash hasher(2.5f);
        REQUIRE_THAT(hasher.get_cell_size(), WithinAbs(2.5f, tolerance::STRICT));
    }

    SECTION("Cell size can be changed") {
        SpatialGridHash hasher(1.0f);
        hasher.set_cell_size(0.5f);
        REQUIRE_THAT(hasher.get_cell_size(), WithinAbs(0.5f, tolerance::STRICT));
    }

    SECTION("Smaller cell size produces more cells") {
        const Vec3f position{5.0f, 5.0f, 5.0f};
        const SpatialGridHash hasher1(1.0f);
        const SpatialGridHash hasher2(0.5f);
        
        const auto cell1 = hasher1.get_cell_coords(position);
        const auto cell2 = hasher2.get_cell_coords(position);
        
        // With 0.5 cell size, position 5.0 is in cell 10
        // With 1.0 cell size, position 5.0 is in cell 5
        REQUIRE(cell1.x == 5);
        REQUIRE(cell2.x == 10);
    }

    SECTION("Larger cell size produces fewer cells") {
        const Vec3f position{5.0f, 5.0f, 5.0f};
        const SpatialGridHash hasher1(1.0f);
        const SpatialGridHash hasher2(2.0f);
        
        const auto cell1 = hasher1.get_cell_coords(position);
        const auto cell2 = hasher2.get_cell_coords(position);
        
        REQUIRE(cell1.x == 5);
        REQUIRE(cell2.x == 2); // 5.0 / 2.0 = 2.5 -> floor(2.5) = 2
    }
}

TEST_CASE("SpatialGridHash::GridCell struct", "[physics][collision]") {
    SECTION("Default construction") {
        const GridCell cell;
        REQUIRE(cell.x == 0);
        REQUIRE(cell.y == 0);
        REQUIRE(cell.z == 0);
    }

    SECTION("Value construction") {
        const GridCell cell(1, 2, 3);
        REQUIRE(cell.x == 1);
        REQUIRE(cell.y == 2);
        REQUIRE(cell.z == 3);
    }

    SECTION("Equality comparison") {
        const GridCell cell1(1, 2, 3);
        const GridCell cell2(1, 2, 3);
        const GridCell cell3(1, 2, 4);
        
        REQUIRE(cell1 == cell2);
        REQUIRE(!(cell1 == cell3));
    }

    SECTION("Inequality comparison") {
        const GridCell cell1(1, 2, 3);
        const GridCell cell2(1, 2, 3);
        const GridCell cell3(1, 2, 4);
        
        REQUIRE(!(cell1 != cell2));
        REQUIRE(cell1 != cell3);
    }
}

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/convex_hull.hpp>
#include <core/physics/collision/shape_factory.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers;
using Catch::Matchers::WithinAbs;

// ============================================================================
// ConvexHull2D Tests
// ============================================================================

TEST_CASE("ConvexHull2D - Default construction", "[collision][convex_hull]") {
    ConvexHull2D hull;
    
    REQUIRE(hull.vertices.empty());
    REQUIRE(hull.normals.empty());
    REQUIRE_THAT(hull.position.x, WithinAbs(0.0f, tolerance::STRICT));
    REQUIRE_THAT(hull.position.y, WithinAbs(0.0f, tolerance::STRICT));
}

TEST_CASE("ConvexHull2D - Construction with vertices", "[collision][convex_hull]") {
    std::vector<Vec2f> vertices = {
        Vec2f(-1.0f, -1.0f),
        Vec2f(1.0f, -1.0f),
        Vec2f(1.0f, 1.0f),
        Vec2f(-1.0f, 1.0f)
    };
    
    ConvexHull2D hull(vertices, Vec2f(0.0f));
    
    REQUIRE(hull.vertices.size() == 4);
    REQUIRE(hull.normals.size() == 4);
    REQUIRE(hull.bounding_radius > 0.0f);
}

TEST_CASE("ConvexHull2D - Check CCW winding", "[collision][convex_hull]") {
    // CCW vertices
    std::vector<Vec2f> ccw_vertices = {
        Vec2f(-1.0f, -1.0f),
        Vec2f(1.0f, -1.0f),
        Vec2f(1.0f, 1.0f),
        Vec2f(-1.0f, 1.0f)
    };
    
    ConvexHull2D hull(ccw_vertices, Vec2f(0.0f));
    REQUIRE(hull.is_ccw());
}

TEST_CASE("ConvexHull2D - Support point", "[collision][convex_hull]") {
    std::vector<Vec2f> vertices = {
        Vec2f(-1.0f, -1.0f),
        Vec2f(1.0f, -1.0f),
        Vec2f(1.0f, 1.0f),
        Vec2f(-1.0f, 1.0f)
    };
    
    ConvexHull2D hull(vertices, Vec2f(0.0f));
    
    // Support point in +X direction should be (1, ?)
    Vec2f support = hull.support_point_2d(Vec2f(1.0f, 0.0f));
    REQUIRE_THAT(support.x, WithinAbs(1.0f, tolerance::STRICT));
    
    // Support point in +Y direction should be (?, 1)
    support = hull.support_point_2d(Vec2f(0.0f, 1.0f));
    REQUIRE_THAT(support.y, WithinAbs(1.0f, tolerance::STRICT));
}

TEST_CASE("ConvexHull2D - Triangle shape", "[collision][convex_hull]") {
    std::vector<Vec2f> vertices = {
        Vec2f(0.0f, -1.0f),
        Vec2f(1.0f, 1.0f),
        Vec2f(-1.0f, 1.0f)
    };
    
    ConvexHull2D hull(vertices, Vec2f(0.0f));
    
    REQUIRE(hull.vertices.size() == 3);
    REQUIRE(hull.normals.size() == 3);
}

TEST_CASE("ConvexHull2D - AABB bounds", "[collision][convex_hull]") {
    std::vector<Vec2f> vertices = {
        Vec2f(-2.0f, -3.0f),
        Vec2f(2.0f, -3.0f),
        Vec2f(2.0f, 3.0f),
        Vec2f(-2.0f, 3.0f)
    };
    
    ConvexHull2D hull(vertices, Vec2f(0.0f));
    AABB bound = hull.get_aabb();
    
    REQUIRE_THAT(bound.min.x, WithinAbs(-2.0f, tolerance::STRICT));
    REQUIRE_THAT(bound.min.y, WithinAbs(-3.0f, tolerance::STRICT));
    REQUIRE_THAT(bound.max.x, WithinAbs(2.0f, tolerance::STRICT));
    REQUIRE_THAT(bound.max.y, WithinAbs(3.0f, tolerance::STRICT));
}

// ============================================================================
// ConvexHull3D Tests
// ============================================================================

TEST_CASE("ConvexHull3D - Default construction", "[collision][convex_hull]") {
    ConvexHull3D hull;
    
    REQUIRE(hull.vertices.empty());
    REQUIRE_THAT(hull.position.x, WithinAbs(0.0f, tolerance::STRICT));
}

TEST_CASE("ConvexHull3D - Construction with vertices", "[collision][convex_hull]") {
    std::vector<Vec3f> vertices = {
        Vec3f(-1.0f, -1.0f, -1.0f),
        Vec3f(1.0f, -1.0f, -1.0f),
        Vec3f(1.0f, 1.0f, -1.0f),
        Vec3f(-1.0f, 1.0f, -1.0f)
    };
    
    ConvexHull3D hull(vertices, Vec3f(0.0f));
    
    REQUIRE(hull.vertices.size() == 4);
    REQUIRE(hull.bounding_radius > 0.0f);
}

TEST_CASE("ConvexHull3D - Support point", "[collision][convex_hull]") {
    std::vector<Vec3f> vertices = {
        Vec3f(-1.0f, -1.0f, -1.0f),
        Vec3f(1.0f, -1.0f, -1.0f),
        Vec3f(1.0f, 1.0f, -1.0f),
        Vec3f(1.0f, 1.0f, 1.0f)
    };
    
    ConvexHull3D hull(vertices, Vec3f(0.0f));
    
    Vec3f support = hull.support_point(Vec3f(1.0f, 0.0f, 0.0f));
    REQUIRE_THAT(support.x, WithinAbs(1.0f, tolerance::STRICT));
}

// ============================================================================
// ShapeFactory Tests
// ============================================================================

TEST_CASE("ShapeFactory - Create 2D box", "[collision][shape_factory]") {
    auto box = ShapeFactory::create_box_2d(2.0f, 3.0f, Vec2f(0.0f));
    
    REQUIRE(box.vertices.size() == 4);
    REQUIRE(box.normals.size() == 4);
    REQUIRE(box.is_ccw());
}

TEST_CASE("ShapeFactory - Create 2D regular polygon", "[collision][shape_factory]") {
    auto hexagon = ShapeFactory::create_regular_polygon_2d(6, 1.0f, 0.0f);
    
    REQUIRE(hexagon.vertices.size() == 6);
    REQUIRE(hexagon.normals.size() == 6);
    REQUIRE(hexagon.is_ccw());
}

TEST_CASE("ShapeFactory - Create 2D triangle", "[collision][shape_factory]") {
    auto triangle = ShapeFactory::create_triangle_2d(
        Vec2f(0.0f, -1.0f),
        Vec2f(1.0f, 1.0f),
        Vec2f(-1.0f, 1.0f)
    );
    
    REQUIRE(triangle.vertices.size() == 3);
    REQUIRE(triangle.is_ccw());
}

TEST_CASE("ShapeFactory - Create 3D box", "[collision][shape_factory]") {
    auto box = ShapeFactory::create_box_3d(1.0f, 2.0f, 3.0f, Vec3f(0.0f));
    
    REQUIRE(box.vertices.size() == 8);
}

TEST_CASE("ShapeFactory - Create 3D tetrahedron", "[collision][shape_factory]") {
    auto tet = ShapeFactory::create_tetrahedron_3d(1.0f);
    
    REQUIRE(tet.vertices.size() == 4);
}

TEST_CASE("ShapeFactory - Create 3D cylinder", "[collision][shape_factory]") {
    auto cyl = ShapeFactory::create_cylinder_3d(1.0f, 2.0f, 8);
    
    // 8 vertices on bottom + 8 on top
    REQUIRE(cyl.vertices.size() == 16);
}

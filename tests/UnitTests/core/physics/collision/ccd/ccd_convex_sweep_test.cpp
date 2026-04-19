#include <catch2/catch_all.hpp>
#include <core/physics/collision/ccd/convex_sweep.hpp>
#include <core/physics/shapes/box.hpp>
#include <core/physics/shapes/sphere.hpp>

using namespace phynity::physics;
using namespace phynity::physics::shapes;
using namespace phynity::physics::collision::ccd;
using namespace phynity::math::vectors;

TEST_CASE("ConvexSweepSolver: Box-box head-on collision", "[unit][physics][collision][ccd][convex]")
{
    BoxShape box_a(Vec3f(0.5f, 0.5f, 0.5f));
    BoxShape box_b(Vec3f(0.5f, 0.5f, 0.5f));

    ConvexSweepSolver::MovingConvex a{&box_a, Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(5.0f, 0.0f, 0.0f)};

    ConvexSweepSolver::MovingConvex b{&box_b, Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f)};

    TimeOfImpactResult result = ConvexSweepSolver::solve(a, b, 1.0f, 16, 1e-4f);

    REQUIRE(result.collision_occurs);
    REQUIRE_THAT(result.toi, Catch::Matchers::WithinAbs(0.6f, 0.1f));
    REQUIRE(std::abs(result.contact_normal.x) > 0.5f);
}

TEST_CASE("ConvexSweepSolver: No collision when moving away", "[unit][physics][collision][ccd][convex]")
{
    BoxShape box_a(Vec3f(0.5f, 0.5f, 0.5f));
    BoxShape box_b(Vec3f(0.5f, 0.5f, 0.5f));

    ConvexSweepSolver::MovingConvex a{&box_a, Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(-2.0f, 0.0f, 0.0f)};

    ConvexSweepSolver::MovingConvex b{&box_b, Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f)};

    TimeOfImpactResult result = ConvexSweepSolver::solve(a, b, 1.0f, 16, 1e-4f);

    REQUIRE(!result.collision_occurs);
}

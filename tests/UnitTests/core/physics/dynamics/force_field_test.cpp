#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/dynamics/force_field.hpp>
#include <core/physics/shapes/aabb.hpp>

#include <cmath>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::Vec3f;
using phynity::physics::BuoyancyField;
using phynity::physics::DragField;
using phynity::physics::ForceField;
using phynity::physics::GravityField;
using phynity::physics::PointGravityField;
using phynity::physics::QuadraticDragField;
using phynity::physics::SpringDamperField;
using phynity::physics::SpringField;
using phynity::physics::UniformElectricField;
using phynity::physics::WindField;
using phynity::physics::shapes::AABB;

// ============================================================================
// Gravity Field Tests
// ============================================================================

TEST_CASE("GravityField: Default constructor (Earth gravity)", "[ForceField][GravityField]")
{
    GravityField gravity;
    Vec3f force = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(-phynity::physics::constants::EARTH_GRAVITY, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
    REQUIRE(gravity.name() == std::string("GravityField"));
}

TEST_CASE("GravityField: Custom gravity vector", "[ForceField][GravityField]")
{
    Vec3f custom_g(1.0f, -5.0f, 2.0f);
    GravityField gravity(custom_g);

    Vec3f force = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(-5.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("GravityField: Force proportional to mass", "[ForceField][GravityField]")
{
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));

    Vec3f force_m1 = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});
    Vec3f force_m5 = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 5.0f});

    REQUIRE_THAT(force_m5.y / force_m1.y, WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("GravityField: Position independent", "[ForceField][GravityField]")
{
    GravityField gravity(Vec3f(0.0f, -phynity::physics::constants::EARTH_GRAVITY, 0.0f));

    Vec3f force1 = gravity.apply({Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    Vec3f force2 = gravity.apply({Vec3f(100.0f, 200.0f, -50.0f), Vec3f(0.0f), 1.0f});

    REQUIRE_THAT(force1.y, WithinAbs(force2.y, 1e-6f));
}

TEST_CASE("GravityField: Velocity independent", "[ForceField][GravityField]")
{
    GravityField gravity(Vec3f(0.0f, -phynity::physics::constants::EARTH_GRAVITY, 0.0f));

    Vec3f force1 = gravity.apply({Vec3f(0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f});
    Vec3f force2 = gravity.apply({Vec3f(0.0f), Vec3f(10.0f, 20.0f, 30.0f), 1.0f});

    REQUIRE_THAT(force1.y, WithinAbs(force2.y, 1e-6f));
}

TEST_CASE("GravityField: Set gravity", "[ForceField][GravityField]")
{
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));
    gravity.set_gravity(Vec3f(0.0f, -20.0f, 0.0f));

    Vec3f force = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.y, WithinAbs(-20.0f, 1e-6f));
}

TEST_CASE("GravityField: Zero gravity", "[ForceField][GravityField]")
{
    GravityField gravity(Vec3f(0.0f, 0.0f, 0.0f));
    Vec3f force = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Linear Drag Field Tests
// ============================================================================

TEST_CASE("DragField: Default constructor", "[ForceField][DragField]")
{
    DragField drag;
    Vec3f force = drag.apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE(drag.name() == std::string("DragField"));
}

TEST_CASE("DragField: Linear drag opposes velocity", "[ForceField][DragField]")
{
    DragField drag(0.5f);
    Vec3f velocity(10.0f, 0.0f, 0.0f);
    Vec3f force = drag.apply({Vec3f(0.0f), velocity, 1.0f});

    // F = -c * v, so F should be opposite to v
    REQUIRE_THAT(force.x, WithinAbs(-5.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("DragField: Proportional to velocity", "[ForceField][DragField]")
{
    DragField drag(0.5f);

    Vec3f force1 = drag.apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});
    Vec3f force2 = drag.apply({Vec3f(0.0f), Vec3f(20.0f, 0.0f, 0.0f), 1.0f});

    // With doubled velocity, force should double
    REQUIRE_THAT(force2.x / force1.x, WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("DragField: Independent of mass", "[ForceField][DragField]")
{
    DragField drag(0.5f);
    Vec3f velocity(10.0f, 0.0f, 0.0f);

    Vec3f force1 = drag.apply({Vec3f(0.0f), velocity, 1.0f});
    Vec3f force2 = drag.apply({Vec3f(0.0f), velocity, 5.0f});

    // Linear drag should be independent of mass
    REQUIRE_THAT(force1.x, WithinAbs(force2.x, 1e-6f));
}

TEST_CASE("DragField: Zero velocity produces zero force", "[ForceField][DragField]")
{
    DragField drag(10.0f);
    Vec3f force = drag.apply({Vec3f(0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("DragField: Position independent", "[ForceField][DragField]")
{
    DragField drag(0.5f);
    Vec3f velocity(5.0f, 10.0f, -3.0f);

    Vec3f force1 = drag.apply({Vec3f(0.0f, 0.0f, 0.0f), velocity, 1.0f});
    Vec3f force2 = drag.apply({Vec3f(100.0f, -50.0f, 200.0f), velocity, 1.0f});

    REQUIRE_THAT(force1.x, WithinAbs(force2.x, 1e-6f));
    REQUIRE_THAT(force1.y, WithinAbs(force2.y, 1e-6f));
    REQUIRE_THAT(force1.z, WithinAbs(force2.z, 1e-6f));
}

TEST_CASE("DragField: Set drag coefficient", "[ForceField][DragField]")
{
    DragField drag(0.5f);
    drag.set_drag_coefficient(2.0f);

    Vec3f force = drag.apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});
    REQUIRE_THAT(force.x, WithinAbs(-20.0f, 1e-6f));
}

// ============================================================================
// Quadratic Drag Field Tests
// ============================================================================

TEST_CASE("QuadraticDragField: Default constructor", "[ForceField][QuadraticDragField]")
{
    QuadraticDragField drag;
    Vec3f force = drag.apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE(drag.name() == std::string("QuadraticDragField"));
}

TEST_CASE("QuadraticDragField: Quadratic drag opposes velocity", "[ForceField][QuadraticDragField]")
{
    QuadraticDragField drag(0.5f);
    Vec3f velocity(10.0f, 0.0f, 0.0f);
    Vec3f force = drag.apply({Vec3f(0.0f), velocity, 1.0f});

    // F = -c * |v| * v = -0.5 * 10 * 10 = -50
    REQUIRE_THAT(force.x, WithinAbs(-50.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("QuadraticDragField: Scales with velocity squared", "[ForceField][QuadraticDragField]")
{
    QuadraticDragField drag(0.5f);

    Vec3f force1 = drag.apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});
    Vec3f force2 = drag.apply({Vec3f(0.0f), Vec3f(20.0f, 0.0f, 0.0f), 1.0f});

    // With doubled velocity, force should quadruple (|v| * v relationship)
    REQUIRE_THAT(std::abs(force2.x) / std::abs(force1.x), WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("QuadraticDragField: Zero velocity produces zero force", "[ForceField][QuadraticDragField]")
{
    QuadraticDragField drag(10.0f);
    Vec3f force = drag.apply({Vec3f(0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("QuadraticDragField: Direction preserved", "[ForceField][QuadraticDragField]")
{
    QuadraticDragField drag(0.1f);
    Vec3f velocity(3.0f, 4.0f, 0.0f); // magnitude = 5
    Vec3f force = drag.apply({Vec3f(0.0f), velocity, 1.0f});

    // F = -0.1 * 5 * (3, 4, 0) = (-1.5, -2.0, 0)
    REQUIRE_THAT(force.x, WithinAbs(-1.5f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(-2.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Spring Field Tests
// ============================================================================

TEST_CASE("SpringField: Restoring force toward center", "[ForceField][SpringField]")
{
    SpringField spring(Vec3f(0.0f), 1.0f);
    Vec3f force = spring.apply({Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});

    // F = -k * (x - center) = -1 * (10 - 0) = -10
    REQUIRE_THAT(force.x, WithinAbs(-10.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("SpringField: At equilibrium, zero force", "[ForceField][SpringField]")
{
    Vec3f center(5.0f, 3.0f, -2.0f);
    SpringField spring(center, 2.0f);

    Vec3f force = spring.apply({center, Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("SpringField: Proportional to spring constant", "[ForceField][SpringField]")
{
    Vec3f pos(10.0f, 0.0f, 0.0f);
    SpringField spring1(Vec3f(0.0f), 1.0f);
    SpringField spring2(Vec3f(0.0f), 2.0f);

    Vec3f force1 = spring1.apply({pos, Vec3f(0.0f), 1.0f});
    Vec3f force2 = spring2.apply({pos, Vec3f(0.0f), 1.0f});

    REQUIRE_THAT(force2.x / force1.x, WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("SpringField: Velocity independent", "[ForceField][SpringField]")
{
    SpringField spring(Vec3f(0.0f), 1.0f);
    Vec3f pos(5.0f, 0.0f, 0.0f);

    Vec3f force1 = spring.apply({pos, Vec3f(0.0f, 0.0f, 0.0f), 1.0f});
    Vec3f force2 = spring.apply({pos, Vec3f(10.0f, 20.0f, 30.0f), 1.0f});

    REQUIRE_THAT(force1.x, WithinAbs(force2.x, 1e-6f));
}

TEST_CASE("SpringField: Set center", "[ForceField][SpringField]")
{
    SpringField spring(Vec3f(0.0f), 1.0f);
    spring.set_center(Vec3f(5.0f, 0.0f, 0.0f));

    Vec3f force = spring.apply({Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    // displacement = (10, 0, 0) - (5, 0, 0) = (5, 0, 0)
    // F = -1 * (5, 0, 0) = (-5, 0, 0)
    REQUIRE_THAT(force.x, WithinAbs(-5.0f, 1e-6f));
}

TEST_CASE("SpringField: Set spring constant", "[ForceField][SpringField]")
{
    SpringField spring(Vec3f(0.0f), 1.0f);
    spring.set_spring_constant(5.0f);

    Vec3f force = spring.apply({Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.x, WithinAbs(-50.0f, 1e-6f));
}

// ============================================================================
// Polymorphic Interface Tests
// ============================================================================

TEST_CASE("ForceField: Polymorphic usage of GravityField", "[ForceField][polymorphism]")
{
    std::unique_ptr<ForceField> field = std::make_unique<GravityField>(Vec3f(0.0f, -10.0f, 0.0f));

    Vec3f force = field->apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.y, WithinAbs(-10.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "GravityField");
}

TEST_CASE("ForceField: Polymorphic usage of DragField", "[ForceField][polymorphism]")
{
    std::unique_ptr<ForceField> field = std::make_unique<DragField>(0.5f);

    Vec3f force = field->apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});
    REQUIRE_THAT(force.x, WithinAbs(-5.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "DragField");
}

TEST_CASE("ForceField: Polymorphic usage of SpringField", "[ForceField][polymorphism]")
{
    std::unique_ptr<ForceField> field = std::make_unique<SpringField>(Vec3f(5.0f), 2.0f);

    Vec3f force = field->apply({Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.x, WithinAbs(-10.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "SpringField");
}

// ============================================================================
// Combined Field Tests
// ============================================================================

TEST_CASE("ForceField: Gravity + Drag combination", "[ForceField][combination]")
{
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));
    DragField drag(0.1f);

    Vec3f pos(0.0f);
    Vec3f vel(5.0f, 0.0f, 0.0f);
    float mass = 1.0f;

    Vec3f f_grav = gravity.apply({pos, vel, mass});
    Vec3f f_drag = drag.apply({pos, vel, mass});
    Vec3f f_total = f_grav + f_drag;

    REQUIRE_THAT(f_total.x, WithinAbs(-0.5f, 1e-6f));
    REQUIRE_THAT(f_total.y, WithinAbs(-10.0f, 1e-6f));
}

TEST_CASE("ForceField: Spring + Drag oscillation setup", "[ForceField][combination]")
{
    SpringField spring(Vec3f(0.0f), 1.0f);
    DragField damping(0.2f);

    Vec3f pos(5.0f, 0.0f, 0.0f);
    Vec3f vel(0.0f, 0.0f, 0.0f);

    Vec3f f_spring = spring.apply({pos, vel, 1.0f});
    Vec3f f_damp = damping.apply({pos, vel, 1.0f});

    REQUIRE_THAT(f_spring.x, WithinAbs(-5.0f, 1e-6f));
    REQUIRE_THAT(f_damp.x, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_CASE("ForceField: Very small velocity (near zero)", "[ForceField][edge-cases]")
{
    DragField drag(1.0f);
    Vec3f force = drag.apply({Vec3f(0.0f), Vec3f(1e-7f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(-1e-7f, 1e-9f));
}

TEST_CASE("ForceField: Very large velocity", "[ForceField][edge-cases]")
{
    QuadraticDragField drag(0.01f);
    Vec3f force = drag.apply({Vec3f(0.0f), Vec3f(1000.0f, 0.0f, 0.0f), 1.0f});

    // F = -0.01 * 1000 * 1000 = -10000
    REQUIRE_THAT(force.x, WithinAbs(-10000.0f, 1e-3f));
}

TEST_CASE("ForceField: Very large mass on gravity", "[ForceField][edge-cases]")
{
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));
    Vec3f force = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 1000.0f});

    REQUIRE_THAT(force.y, WithinAbs(-10000.0f, 1e-3f));
}

TEST_CASE("ForceField: Zero spring constant", "[ForceField][edge-cases]")
{
    SpringField spring(Vec3f(0.0f), 0.0f);
    Vec3f force = spring.apply({Vec3f(100.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});

    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Point Gravity Field Tests (radial gravity well)
// ============================================================================

TEST_CASE("PointGravityField: Pulls toward center", "[ForceField][PointGravityField]")
{
    PointGravityField well(Vec3f(0.0f), 10.0f);
    // Particle to the right of the center should be pulled left (-x).
    Vec3f force = well.apply({Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});

    REQUIRE(force.x < 0.0f);
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
    REQUIRE(well.name() == std::string("PointGravityField"));
}

TEST_CASE("PointGravityField: Inverse-square falloff", "[ForceField][PointGravityField]")
{
    PointGravityField well(Vec3f(0.0f), 10.0f, 1e-3f);

    Vec3f f_near = well.apply({Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    Vec3f f_far = well.apply({Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});

    // At r=1: |F| = 10; at r=2: |F| = 10/4 = 2.5. Ratio should be 4.
    REQUIRE_THAT(std::abs(f_near.x), WithinAbs(10.0f, 1e-4f));
    REQUIRE_THAT(std::abs(f_far.x), WithinAbs(2.5f, 1e-4f));
    REQUIRE_THAT(std::abs(f_near.x) / std::abs(f_far.x), WithinAbs(4.0f, 1e-4f));
}

TEST_CASE("PointGravityField: Proportional to mass", "[ForceField][PointGravityField]")
{
    PointGravityField well(Vec3f(0.0f), 10.0f, 1e-3f);

    Vec3f f_m1 = well.apply({Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    Vec3f f_m5 = well.apply({Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f), 5.0f});

    REQUIRE_THAT(f_m5.x / f_m1.x, WithinAbs(5.0f, 1e-4f));
}

TEST_CASE("PointGravityField: Softening clamp bounds force near center", "[ForceField][PointGravityField]")
{
    // With min_distance = 1, r² is clamped so the force at the center is finite.
    PointGravityField well(Vec3f(0.0f), 10.0f, 1.0f);
    Vec3f force = well.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});

    // At the exact center dir is zero -> zero force, and never NaN/Inf.
    REQUIRE(std::isfinite(force.x));
    REQUIRE(std::isfinite(force.y));
    REQUIRE(std::isfinite(force.z));
    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("PointGravityField: Velocity independent", "[ForceField][PointGravityField]")
{
    PointGravityField well(Vec3f(0.0f), 10.0f, 1e-3f);

    Vec3f f1 = well.apply({Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f});
    Vec3f f2 = well.apply({Vec3f(2.0f, 0.0f, 0.0f), Vec3f(10.0f, -3.0f, 7.0f), 1.0f});

    REQUIRE_THAT(f1.x, WithinAbs(f2.x, 1e-6f));
}

TEST_CASE("ForceField: Polymorphic usage of PointGravityField", "[ForceField][polymorphism]")
{
    std::unique_ptr<ForceField> field = std::make_unique<PointGravityField>(Vec3f(0.0f), 10.0f, 1e-3f);

    Vec3f force = field->apply({Vec3f(1.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.x, WithinAbs(-10.0f, 1e-4f));
    REQUIRE(std::string(field->name()) == "PointGravityField");
}

// ============================================================================
// Wind Field Tests (bounded drag volume)
// ============================================================================

TEST_CASE("WindField: Drives body toward wind velocity", "[ForceField][WindField]")
{
    WindField wind(Vec3f(10.0f, 0.0f, 0.0f), 0.5f);
    // Body at rest: F = 0.5 * (10 - 0) = 5 in +x.
    Vec3f force = wind.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
    REQUIRE(wind.name() == std::string("WindField"));
}

TEST_CASE("WindField: Zero force when body matches wind", "[ForceField][WindField]")
{
    WindField wind(Vec3f(10.0f, 0.0f, 0.0f), 0.5f);
    Vec3f force = wind.apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("WindField: Opposes motion against the wind", "[ForceField][WindField]")
{
    WindField wind(Vec3f(0.0f), 0.5f); // still air
    // Body moving +x through still air is dragged back: F = 0.5 * (0 - 10) = -5.
    Vec3f force = wind.apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(-5.0f, 1e-6f));
}

TEST_CASE("WindField: Unbounded acts everywhere", "[ForceField][WindField]")
{
    WindField wind(Vec3f(10.0f, 0.0f, 0.0f), 0.5f);

    Vec3f f_here = wind.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});
    Vec3f f_far = wind.apply({Vec3f(1000.0f, -500.0f, 200.0f), Vec3f(0.0f), 1.0f});

    REQUIRE(!wind.is_bounded());
    REQUIRE_THAT(f_here.x, WithinAbs(f_far.x, 1e-6f));
}

TEST_CASE("WindField: Bounded region gates the force", "[ForceField][WindField]")
{
    AABB region(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    WindField wind(Vec3f(10.0f, 0.0f, 0.0f), 0.5f, region);

    REQUIRE(wind.is_bounded());

    // Inside the region: wind applies.
    Vec3f inside = wind.apply({Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(inside.x, WithinAbs(5.0f, 1e-6f));

    // Outside the region: no force.
    Vec3f outside = wind.apply({Vec3f(5.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(outside.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("ForceField: Polymorphic usage of WindField", "[ForceField][polymorphism]")
{
    std::unique_ptr<ForceField> field = std::make_unique<WindField>(Vec3f(10.0f, 0.0f, 0.0f), 0.5f);

    Vec3f force = field->apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "WindField");
}

// ============================================================================
// Spring–Damper Field Tests
// ============================================================================

TEST_CASE("SpringDamperField: Zero force at equilibrium with zero velocity", "[ForceField][SpringDamperField]")
{
    Vec3f center(5.0f, 3.0f, -2.0f);
    SpringDamperField sd(center, 2.0f, 0.5f);

    Vec3f force = sd.apply({center, Vec3f(0.0f), 1.0f});
    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
    REQUIRE(sd.name() == std::string("SpringDamperField"));
}

TEST_CASE("SpringDamperField: With zero damping matches SpringField", "[ForceField][SpringDamperField]")
{
    Vec3f center(0.0f);
    Vec3f pos(10.0f, -4.0f, 2.0f);
    Vec3f vel(3.0f, 3.0f, 3.0f);

    SpringField spring(center, 2.0f);
    SpringDamperField sd(center, 2.0f, 0.0f);

    Vec3f f_spring = spring.apply({pos, vel, 1.0f});
    Vec3f f_sd = sd.apply({pos, vel, 1.0f});

    REQUIRE_THAT(f_sd.x, WithinAbs(f_spring.x, 1e-6f));
    REQUIRE_THAT(f_sd.y, WithinAbs(f_spring.y, 1e-6f));
    REQUIRE_THAT(f_sd.z, WithinAbs(f_spring.z, 1e-6f));
}

TEST_CASE("SpringDamperField: Damping opposes velocity", "[ForceField][SpringDamperField]")
{
    // At equilibrium, only the damping term acts: F = -c * v.
    SpringDamperField sd(Vec3f(0.0f), 2.0f, 0.5f);
    Vec3f force = sd.apply({Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(-5.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("SpringDamperField: Combined spring and damping", "[ForceField][SpringDamperField]")
{
    SpringDamperField sd(Vec3f(0.0f), 2.0f, 0.5f);
    // F = -2 * (4, 0, 0) - 0.5 * (10, 0, 0) = (-8 - 5, 0, 0) = (-13, 0, 0)
    Vec3f force = sd.apply({Vec3f(4.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});

    REQUIRE_THAT(force.x, WithinAbs(-13.0f, 1e-6f));
}

TEST_CASE("ForceField: Polymorphic usage of SpringDamperField", "[ForceField][polymorphism]")
{
    std::unique_ptr<ForceField> field = std::make_unique<SpringDamperField>(Vec3f(0.0f), 2.0f, 0.5f);

    Vec3f force = field->apply({Vec3f(4.0f, 0.0f, 0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f});
    REQUIRE_THAT(force.x, WithinAbs(-13.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "SpringDamperField");
}

// ============================================================================
// Buoyancy Field Tests
// ============================================================================

// Gravity is supplied to buoyancy via the shared ForceContext (4th field), not
// stored on the field — mirroring how the systems publish their ambient gravity.
static constexpr Vec3f kTestGravity(0.0f, -10.0f, 0.0f);

TEST_CASE("BuoyancyField: Submerged body pushed upward", "[ForceField][BuoyancyField]")
{
    // fluid = object = 1000, mass 1 -> V = 0.001; gravity = (0,-10,0).
    // F = -gravity * fluid_density * V = (0, +10, 0).
    BuoyancyField buoyancy(1000.0f, 1000.0f, 0.0f);
    Vec3f force = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 1.0f, kTestGravity});

    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(10.0f, 1e-5f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
    REQUIRE(buoyancy.name() == std::string("BuoyancyField"));
}

TEST_CASE("BuoyancyField: Body above surface gets no force", "[ForceField][BuoyancyField]")
{
    BuoyancyField buoyancy(1000.0f, 1000.0f, 0.0f);
    Vec3f force = buoyancy.apply({Vec3f(0.0f, 5.0f, 0.0f), Vec3f(0.0f), 1.0f, kTestGravity});

    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// ForceContext charge member (Phase 0 plumbing)
// ============================================================================

TEST_CASE("ForceContext: Carries charge with a zero default", "[ForceField][ForceContext]")
{
    using phynity::physics::ForceContext;

    ForceContext ctx;
    REQUIRE_THAT(ctx.charge, WithinAbs(0.0f, 1e-6f));

    ForceContext charged{Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, -2.5f};
    REQUIRE_THAT(charged.charge, WithinAbs(-2.5f, 1e-6f));
}

TEST_CASE("ForceField: Existing fields ignore charge", "[ForceField][ForceContext]")
{
    // A charged context must not change the output of a non-electromagnetic field.
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));

    Vec3f neutral = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 2.0f, kTestGravity, 0.0f});
    Vec3f charged = gravity.apply({Vec3f(0.0f), Vec3f(0.0f), 2.0f, kTestGravity, 5.0f});

    REQUIRE_THAT(charged.x, WithinAbs(neutral.x, 1e-6f));
    REQUIRE_THAT(charged.y, WithinAbs(neutral.y, 1e-6f));
    REQUIRE_THAT(charged.z, WithinAbs(neutral.z, 1e-6f));
}

// ============================================================================
// Uniform Electric Field Tests
// ============================================================================

TEST_CASE("UniformElectricField: Positive charge accelerates along +E", "[ForceField][UniformElectricField]")
{
    UniformElectricField efield(Vec3f(3.0f, 0.0f, 0.0f));
    // F = q * E = +2 * (3,0,0) = (6,0,0).
    Vec3f force = efield.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, 2.0f});

    REQUIRE_THAT(force.x, WithinAbs(6.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
    REQUIRE(efield.name() == std::string("UniformElectricField"));
}

TEST_CASE("UniformElectricField: Negative charge accelerates along -E", "[ForceField][UniformElectricField]")
{
    UniformElectricField efield(Vec3f(3.0f, 0.0f, 0.0f));
    // F = q * E = -2 * (3,0,0) = (-6,0,0).
    Vec3f force = efield.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, -2.0f});

    REQUIRE_THAT(force.x, WithinAbs(-6.0f, 1e-6f));
}

TEST_CASE("UniformElectricField: Zero charge feels no force", "[ForceField][UniformElectricField]")
{
    UniformElectricField efield(Vec3f(3.0f, -1.0f, 5.0f));
    Vec3f force = efield.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, 0.0f});

    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("UniformElectricField: Force scales linearly with charge and field",
          "[ForceField][UniformElectricField]")
{
    UniformElectricField weak(Vec3f(1.0f, 0.0f, 0.0f));
    UniformElectricField strong(Vec3f(4.0f, 0.0f, 0.0f));

    Vec3f f_q1 = weak.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, 1.0f});
    Vec3f f_q3 = weak.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, 3.0f});
    Vec3f f_e4 = strong.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, 1.0f});

    REQUIRE_THAT(f_q3.x / f_q1.x, WithinAbs(3.0f, 1e-6f)); // linear in q
    REQUIRE_THAT(f_e4.x / f_q1.x, WithinAbs(4.0f, 1e-6f)); // linear in |E|
}

TEST_CASE("UniformElectricField: Velocity and position independent", "[ForceField][UniformElectricField]")
{
    UniformElectricField efield(Vec3f(2.0f, -3.0f, 1.0f));

    Vec3f f1 = efield.apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, 1.5f});
    Vec3f f2 = efield.apply({Vec3f(100.0f, -5.0f, 9.0f), Vec3f(4.0f, 4.0f, 4.0f), 1.0f, kTestGravity, 1.5f});

    REQUIRE_THAT(f1.x, WithinAbs(f2.x, 1e-6f));
    REQUIRE_THAT(f1.y, WithinAbs(f2.y, 1e-6f));
    REQUIRE_THAT(f1.z, WithinAbs(f2.z, 1e-6f));
}

TEST_CASE("ForceField: Polymorphic usage of UniformElectricField", "[ForceField][polymorphism]")
{
    std::unique_ptr<ForceField> field = std::make_unique<UniformElectricField>(Vec3f(3.0f, 0.0f, 0.0f));

    Vec3f force = field->apply({Vec3f(0.0f), Vec3f(0.0f), 1.0f, kTestGravity, 2.0f});
    REQUIRE_THAT(force.x, WithinAbs(6.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "UniformElectricField");
}

TEST_CASE("BuoyancyField: Dense object correct magnitude", "[ForceField][BuoyancyField]")
{
    // object_density = 2000, mass 2 -> V = 0.001; fluid = 1000, gravity = (0,-10,0).
    // Buoyant force = fluid_density * V * |g| = 1000 * 0.001 * 10 = 10 (upward).
    BuoyancyField buoyancy(1000.0f, 2000.0f, 0.0f);
    Vec3f force = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 2.0f, kTestGravity});

    REQUIRE_THAT(force.y, WithinAbs(10.0f, 1e-5f));
}

TEST_CASE("BuoyancyField: Velocity independent", "[ForceField][BuoyancyField]")
{
    BuoyancyField buoyancy(1000.0f, 1000.0f, 0.0f);

    Vec3f f1 = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f, kTestGravity});
    Vec3f f2 = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(3.0f, 7.0f, -2.0f), 1.0f, kTestGravity});

    REQUIRE_THAT(f1.y, WithinAbs(f2.y, 1e-6f));
}

TEST_CASE("BuoyancyField: Force scales with mass (volume)", "[ForceField][BuoyancyField]")
{
    BuoyancyField buoyancy(1000.0f, 1000.0f, 0.0f);

    Vec3f f_m1 = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 1.0f, kTestGravity});
    Vec3f f_m3 = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 3.0f, kTestGravity});

    REQUIRE_THAT(f_m3.y / f_m1.y, WithinAbs(3.0f, 1e-5f));
}

TEST_CASE("BuoyancyField: Reads gravity from the shared context", "[ForceField][BuoyancyField]")
{
    // Same field, two different ambient-gravity contexts: the force must track
    // the context's gravity, proving no stale copy is held on the field.
    BuoyancyField buoyancy(1000.0f, 1000.0f, 0.0f);

    Vec3f f_earthlike = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 1.0f, Vec3f(0.0f, -10.0f, 0.0f)});
    Vec3f f_strong = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 1.0f, Vec3f(0.0f, -20.0f, 0.0f)});

    REQUIRE_THAT(f_earthlike.y, WithinAbs(10.0f, 1e-5f));
    REQUIRE_THAT(f_strong.y, WithinAbs(20.0f, 1e-5f));
}

TEST_CASE("BuoyancyField: Zero object density is safe", "[ForceField][BuoyancyField]")
{
    BuoyancyField buoyancy(1000.0f, 0.0f, 0.0f);
    Vec3f force = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 1.0f, kTestGravity});

    REQUIRE(std::isfinite(force.y));
    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("BuoyancyField: Zero ambient gravity is safe", "[ForceField][BuoyancyField]")
{
    BuoyancyField buoyancy(1000.0f, 1000.0f, 0.0f);
    Vec3f force = buoyancy.apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 1.0f, Vec3f(0.0f)});

    REQUIRE(std::isfinite(force.y));
    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("ForceField: Polymorphic usage of BuoyancyField", "[ForceField][polymorphism]")
{
    std::unique_ptr<ForceField> field = std::make_unique<BuoyancyField>(1000.0f, 1000.0f, 0.0f);

    Vec3f force = field->apply({Vec3f(0.0f, -5.0f, 0.0f), Vec3f(0.0f), 1.0f, kTestGravity});
    REQUIRE_THAT(force.y, WithinAbs(10.0f, 1e-5f));
    REQUIRE(std::string(field->name()) == "BuoyancyField");
}

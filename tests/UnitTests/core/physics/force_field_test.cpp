#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/force_field.hpp>
#include <cmath>

using phynity::physics::ForceField;
using phynity::physics::GravityField;
using phynity::physics::DragField;
using phynity::physics::QuadraticDragField;
using phynity::physics::SpringField;
using phynity::math::vectors::Vec3f;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Gravity Field Tests
// ============================================================================

TEST_CASE("GravityField: Default constructor (Earth gravity)", "[ForceField][GravityField]") {
    GravityField gravity;
    Vec3f force = gravity.apply(Vec3f(0.0f), Vec3f(0.0f), 1.0f);
    
    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(-9.81f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
    REQUIRE(gravity.name() == std::string("GravityField"));
}

TEST_CASE("GravityField: Custom gravity vector", "[ForceField][GravityField]") {
    Vec3f custom_g(1.0f, -5.0f, 2.0f);
    GravityField gravity(custom_g);
    
    Vec3f force = gravity.apply(Vec3f(0.0f), Vec3f(0.0f), 1.0f);
    REQUIRE_THAT(force.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(-5.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("GravityField: Force proportional to mass", "[ForceField][GravityField]") {
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));
    
    Vec3f force_m1 = gravity.apply(Vec3f(0.0f), Vec3f(0.0f), 1.0f);
    Vec3f force_m5 = gravity.apply(Vec3f(0.0f), Vec3f(0.0f), 5.0f);
    
    REQUIRE_THAT(force_m5.y / force_m1.y, WithinAbs(5.0f, 1e-6f));
}

TEST_CASE("GravityField: Position independent", "[ForceField][GravityField]") {
    GravityField gravity(Vec3f(0.0f, -9.81f, 0.0f));
    
    Vec3f force1 = gravity.apply(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f);
    Vec3f force2 = gravity.apply(Vec3f(100.0f, 200.0f, -50.0f), Vec3f(0.0f), 1.0f);
    
    REQUIRE_THAT(force1.y, WithinAbs(force2.y, 1e-6f));
}

TEST_CASE("GravityField: Velocity independent", "[ForceField][GravityField]") {
    GravityField gravity(Vec3f(0.0f, -9.81f, 0.0f));
    
    Vec3f force1 = gravity.apply(Vec3f(0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    Vec3f force2 = gravity.apply(Vec3f(0.0f), Vec3f(10.0f, 20.0f, 30.0f), 1.0f);
    
    REQUIRE_THAT(force1.y, WithinAbs(force2.y, 1e-6f));
}

TEST_CASE("GravityField: Set gravity", "[ForceField][GravityField]") {
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));
    gravity.set_gravity(Vec3f(0.0f, -20.0f, 0.0f));
    
    Vec3f force = gravity.apply(Vec3f(0.0f), Vec3f(0.0f), 1.0f);
    REQUIRE_THAT(force.y, WithinAbs(-20.0f, 1e-6f));
}

TEST_CASE("GravityField: Zero gravity", "[ForceField][GravityField]") {
    GravityField gravity(Vec3f(0.0f, 0.0f, 0.0f));
    Vec3f force = gravity.apply(Vec3f(0.0f), Vec3f(0.0f), 1.0f);
    
    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Linear Drag Field Tests
// ============================================================================

TEST_CASE("DragField: Default constructor", "[ForceField][DragField]") {
    DragField drag;
    Vec3f force = drag.apply(Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f);
    
    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE(drag.name() == std::string("DragField"));
}

TEST_CASE("DragField: Linear drag opposes velocity", "[ForceField][DragField]") {
    DragField drag(0.5f);
    Vec3f velocity(10.0f, 0.0f, 0.0f);
    Vec3f force = drag.apply(Vec3f(0.0f), velocity, 1.0f);
    
    // F = -c * v, so F should be opposite to v
    REQUIRE_THAT(force.x, WithinAbs(-5.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("DragField: Proportional to velocity", "[ForceField][DragField]") {
    DragField drag(0.5f);
    
    Vec3f force1 = drag.apply(Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f);
    Vec3f force2 = drag.apply(Vec3f(0.0f), Vec3f(20.0f, 0.0f, 0.0f), 1.0f);
    
    // With doubled velocity, force should double
    REQUIRE_THAT(force2.x / force1.x, WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("DragField: Independent of mass", "[ForceField][DragField]") {
    DragField drag(0.5f);
    Vec3f velocity(10.0f, 0.0f, 0.0f);
    
    Vec3f force1 = drag.apply(Vec3f(0.0f), velocity, 1.0f);
    Vec3f force2 = drag.apply(Vec3f(0.0f), velocity, 5.0f);
    
    // Linear drag should be independent of mass
    REQUIRE_THAT(force1.x, WithinAbs(force2.x, 1e-6f));
}

TEST_CASE("DragField: Zero velocity produces zero force", "[ForceField][DragField]") {
    DragField drag(10.0f);
    Vec3f force = drag.apply(Vec3f(0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    
    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("DragField: Position independent", "[ForceField][DragField]") {
    DragField drag(0.5f);
    Vec3f velocity(5.0f, 10.0f, -3.0f);
    
    Vec3f force1 = drag.apply(Vec3f(0.0f, 0.0f, 0.0f), velocity, 1.0f);
    Vec3f force2 = drag.apply(Vec3f(100.0f, -50.0f, 200.0f), velocity, 1.0f);
    
    REQUIRE_THAT(force1.x, WithinAbs(force2.x, 1e-6f));
    REQUIRE_THAT(force1.y, WithinAbs(force2.y, 1e-6f));
    REQUIRE_THAT(force1.z, WithinAbs(force2.z, 1e-6f));
}

TEST_CASE("DragField: Set drag coefficient", "[ForceField][DragField]") {
    DragField drag(0.5f);
    drag.set_drag_coefficient(2.0f);
    
    Vec3f force = drag.apply(Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f);
    REQUIRE_THAT(force.x, WithinAbs(-20.0f, 1e-6f));
}

// ============================================================================
// Quadratic Drag Field Tests
// ============================================================================

TEST_CASE("QuadraticDragField: Default constructor", "[ForceField][QuadraticDragField]") {
    QuadraticDragField drag;
    Vec3f force = drag.apply(Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f);
    
    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE(drag.name() == std::string("QuadraticDragField"));
}

TEST_CASE("QuadraticDragField: Quadratic drag opposes velocity", "[ForceField][QuadraticDragField]") {
    QuadraticDragField drag(0.5f);
    Vec3f velocity(10.0f, 0.0f, 0.0f);
    Vec3f force = drag.apply(Vec3f(0.0f), velocity, 1.0f);
    
    // F = -c * |v| * v = -0.5 * 10 * 10 = -50
    REQUIRE_THAT(force.x, WithinAbs(-50.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("QuadraticDragField: Scales with velocity squared", "[ForceField][QuadraticDragField]") {
    QuadraticDragField drag(0.5f);
    
    Vec3f force1 = drag.apply(Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f);
    Vec3f force2 = drag.apply(Vec3f(0.0f), Vec3f(20.0f, 0.0f, 0.0f), 1.0f);
    
    // With doubled velocity, force should quadruple (|v| * v relationship)
    REQUIRE_THAT(std::abs(force2.x) / std::abs(force1.x), WithinAbs(4.0f, 1e-6f));
}

TEST_CASE("QuadraticDragField: Zero velocity produces zero force", "[ForceField][QuadraticDragField]") {
    QuadraticDragField drag(10.0f);
    Vec3f force = drag.apply(Vec3f(0.0f), Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    
    REQUIRE_THAT(force.x, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("QuadraticDragField: Direction preserved", "[ForceField][QuadraticDragField]") {
    QuadraticDragField drag(0.1f);
    Vec3f velocity(3.0f, 4.0f, 0.0f);  // magnitude = 5
    Vec3f force = drag.apply(Vec3f(0.0f), velocity, 1.0f);
    
    // F = -0.1 * 5 * (3, 4, 0) = (-1.5, -2.0, 0)
    REQUIRE_THAT(force.x, WithinAbs(-1.5f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(-2.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Spring Field Tests
// ============================================================================

TEST_CASE("SpringField: Restoring force toward center", "[ForceField][SpringField]") {
    SpringField spring(Vec3f(0.0f), 1.0f);
    Vec3f force = spring.apply(Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f);
    
    // F = -k * (x - center) = -1 * (10 - 0) = -10
    REQUIRE_THAT(force.x, WithinAbs(-10.0f, 1e-6f));
    REQUIRE_THAT(force.y, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(force.z, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("SpringField: At equilibrium, zero force", "[ForceField][SpringField]") {
    Vec3f center(5.0f, 3.0f, -2.0f);
    SpringField spring(center, 2.0f);
    
    Vec3f force = spring.apply(center, Vec3f(0.0f), 1.0f);
    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("SpringField: Proportional to spring constant", "[ForceField][SpringField]") {
    Vec3f pos(10.0f, 0.0f, 0.0f);
    SpringField spring1(Vec3f(0.0f), 1.0f);
    SpringField spring2(Vec3f(0.0f), 2.0f);
    
    Vec3f force1 = spring1.apply(pos, Vec3f(0.0f), 1.0f);
    Vec3f force2 = spring2.apply(pos, Vec3f(0.0f), 1.0f);
    
    REQUIRE_THAT(force2.x / force1.x, WithinAbs(2.0f, 1e-6f));
}

TEST_CASE("SpringField: Velocity independent", "[ForceField][SpringField]") {
    SpringField spring(Vec3f(0.0f), 1.0f);
    Vec3f pos(5.0f, 0.0f, 0.0f);
    
    Vec3f force1 = spring.apply(pos, Vec3f(0.0f, 0.0f, 0.0f), 1.0f);
    Vec3f force2 = spring.apply(pos, Vec3f(10.0f, 20.0f, 30.0f), 1.0f);
    
    REQUIRE_THAT(force1.x, WithinAbs(force2.x, 1e-6f));
}

TEST_CASE("SpringField: Set center", "[ForceField][SpringField]") {
    SpringField spring(Vec3f(0.0f), 1.0f);
    spring.set_center(Vec3f(5.0f, 0.0f, 0.0f));
    
    Vec3f force = spring.apply(Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f);
    // displacement = (10, 0, 0) - (5, 0, 0) = (5, 0, 0)
    // F = -1 * (5, 0, 0) = (-5, 0, 0)
    REQUIRE_THAT(force.x, WithinAbs(-5.0f, 1e-6f));
}

TEST_CASE("SpringField: Set spring constant", "[ForceField][SpringField]") {
    SpringField spring(Vec3f(0.0f), 1.0f);
    spring.set_spring_constant(5.0f);
    
    Vec3f force = spring.apply(Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f);
    REQUIRE_THAT(force.x, WithinAbs(-50.0f, 1e-6f));
}

// ============================================================================
// Polymorphic Interface Tests
// ============================================================================

TEST_CASE("ForceField: Polymorphic usage of GravityField", "[ForceField][polymorphism]") {
    std::unique_ptr<ForceField> field = std::make_unique<GravityField>(Vec3f(0.0f, -10.0f, 0.0f));
    
    Vec3f force = field->apply(Vec3f(0.0f), Vec3f(0.0f), 1.0f);
    REQUIRE_THAT(force.y, WithinAbs(-10.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "GravityField");
}

TEST_CASE("ForceField: Polymorphic usage of DragField", "[ForceField][polymorphism]") {
    std::unique_ptr<ForceField> field = std::make_unique<DragField>(0.5f);
    
    Vec3f force = field->apply(Vec3f(0.0f), Vec3f(10.0f, 0.0f, 0.0f), 1.0f);
    REQUIRE_THAT(force.x, WithinAbs(-5.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "DragField");
}

TEST_CASE("ForceField: Polymorphic usage of SpringField", "[ForceField][polymorphism]") {
    std::unique_ptr<ForceField> field = std::make_unique<SpringField>(Vec3f(5.0f), 2.0f);
    
    Vec3f force = field->apply(Vec3f(10.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f);
    REQUIRE_THAT(force.x, WithinAbs(-10.0f, 1e-6f));
    REQUIRE(std::string(field->name()) == "SpringField");
}

// ============================================================================
// Combined Field Tests
// ============================================================================

TEST_CASE("ForceField: Gravity + Drag combination", "[ForceField][combination]") {
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));
    DragField drag(0.1f);
    
    Vec3f pos(0.0f);
    Vec3f vel(5.0f, 0.0f, 0.0f);
    float mass = 1.0f;
    
    Vec3f f_grav = gravity.apply(pos, vel, mass);
    Vec3f f_drag = drag.apply(pos, vel, mass);
    Vec3f f_total = f_grav + f_drag;
    
    REQUIRE_THAT(f_total.x, WithinAbs(-0.5f, 1e-6f));
    REQUIRE_THAT(f_total.y, WithinAbs(-10.0f, 1e-6f));
}

TEST_CASE("ForceField: Spring + Drag oscillation setup", "[ForceField][combination]") {
    SpringField spring(Vec3f(0.0f), 1.0f);
    DragField damping(0.2f);
    
    Vec3f pos(5.0f, 0.0f, 0.0f);
    Vec3f vel(0.0f, 0.0f, 0.0f);
    
    Vec3f f_spring = spring.apply(pos, vel, 1.0f);
    Vec3f f_damp = damping.apply(pos, vel, 1.0f);
    
    REQUIRE_THAT(f_spring.x, WithinAbs(-5.0f, 1e-6f));
    REQUIRE_THAT(f_damp.x, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_CASE("ForceField: Very small velocity (near zero)", "[ForceField][edge-cases]") {
    DragField drag(1.0f);
    Vec3f force = drag.apply(Vec3f(0.0f), Vec3f(1e-7f, 0.0f, 0.0f), 1.0f);
    
    REQUIRE_THAT(force.x, WithinAbs(-1e-7f, 1e-9f));
}

TEST_CASE("ForceField: Very large velocity", "[ForceField][edge-cases]") {
    QuadraticDragField drag(0.01f);
    Vec3f force = drag.apply(Vec3f(0.0f), Vec3f(1000.0f, 0.0f, 0.0f), 1.0f);
    
    // F = -0.01 * 1000 * 1000 = -10000
    REQUIRE_THAT(force.x, WithinAbs(-10000.0f, 1e-3f));
}

TEST_CASE("ForceField: Very large mass on gravity", "[ForceField][edge-cases]") {
    GravityField gravity(Vec3f(0.0f, -10.0f, 0.0f));
    Vec3f force = gravity.apply(Vec3f(0.0f), Vec3f(0.0f), 1000.0f);
    
    REQUIRE_THAT(force.y, WithinAbs(-10000.0f, 1e-3f));
}

TEST_CASE("ForceField: Zero spring constant", "[ForceField][edge-cases]") {
    SpringField spring(Vec3f(0.0f), 0.0f);
    Vec3f force = spring.apply(Vec3f(100.0f, 0.0f, 0.0f), Vec3f(0.0f), 1.0f);
    
    REQUIRE_THAT(force.length(), WithinAbs(0.0f, 1e-6f));
}

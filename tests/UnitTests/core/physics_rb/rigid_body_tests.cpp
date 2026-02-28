#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/macro/rigid_body.hpp>
#include <core/physics/macro/rigid_body_system.hpp>
#include <core/physics/macro/shape.hpp>
#include <core/physics/macro/inertia.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/physics/common/material.hpp>
#include <core/physics/constraints/joints/fixed_constraint_rb.hpp>
#include <cmath>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using namespace phynity::math::matrices;
using namespace phynity::math::quaternions;
using Catch::Matchers::WithinAbs;

// ============================================================================
// INERTIA TENSOR TESTS
// ============================================================================

TEST_CASE("Sphere inertia tensor", "[inertia][sphere]") {
    // Sphere: I = (2/5) * m * r²
    float mass = 1.0f;
    float radius = 0.5f;
    
    Mat3f I = inertia::compute_sphere_inertia(mass, radius);
    
    float expected = (2.0f / 5.0f) * mass * radius * radius;
    
    REQUIRE_THAT(I(0, 0), WithinAbs(expected, 1e-6f));
    REQUIRE_THAT(I(1, 1), WithinAbs(expected, 1e-6f));
    REQUIRE_THAT(I(2, 2), WithinAbs(expected, 1e-6f));
    
    // Check off-diagonal is zero
    REQUIRE_THAT(I(0, 1), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(I(1, 2), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Box inertia tensor", "[inertia][box]") {
    // Box: I_ii = (1/12) * m * (sum of squares of other two dimensions)
    float mass = 2.0f;
    Vec3f half_extents(1.0f, 2.0f, 3.0f);
    
    Mat3f I = inertia::compute_box_inertia(mass, half_extents);
    
    // Full dimensions
    float a = 2.0f * half_extents.x;  // 2.0
    float b = 2.0f * half_extents.y;  // 4.0
    float c = 2.0f * half_extents.z;  // 6.0
    
    float coeff = mass / 12.0f;
    float I_xx = coeff * (b * b + c * c);
    float I_yy = coeff * (a * a + c * c);
    float I_zz = coeff * (a * a + b * b);
    
    REQUIRE_THAT(I(0, 0), WithinAbs(I_xx, 1e-5f));
    REQUIRE_THAT(I(1, 1), WithinAbs(I_yy, 1e-5f));
    REQUIRE_THAT(I(2, 2), WithinAbs(I_zz, 1e-5f));
}

// ============================================================================
// RIGID BODY BASIC TESTS
// ============================================================================

TEST_CASE("RigidBody creation with sphere", "[rigid_body][creation]") {
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBody rb(
        Vec3f(1.0f, 2.0f, 3.0f),
        Quatf(),  // Identity
        shape,
        steel(),
        5.0f  // 5 kg
    );
    
    REQUIRE_THAT(rb.position.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(rb.position.y, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(rb.position.z, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(rb.get_mass(), WithinAbs(5.0f, 1e-6f));
    
    // Check inertia tensor was computed
    REQUIRE(rb.inertia_tensor(0, 0) > 0.0f);
    REQUIRE_THAT(rb.inertia_tensor(0, 0), WithinAbs(rb.inertia_tensor(1, 1), 1e-6f));
}

TEST_CASE("RigidBody static vs dynamic", "[rigid_body][mass]") {
    auto shape = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));
    
    // Dynamic body
    RigidBody dynamic(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);
    REQUIRE(!dynamic.is_static());
    REQUIRE_THAT(dynamic.inv_mass, WithinAbs(1.0f, 1e-6f));
    
    // Static body (infinite mass)
    RigidBody static_body(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 0.0f);
    REQUIRE(static_body.is_static());
    REQUIRE_THAT(static_body.inv_mass, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// KINETIC ENERGY TESTS
// ============================================================================

TEST_CASE("RigidBody kinetic energy - linear only", "[rigid_body][energy]") {
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBody rb(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 2.0f);
    
    rb.velocity = Vec3f(3.0f, 4.0f, 0.0f);  // |v| = 5
    
    float ke = rb.kinetic_energy();
    float expected = 0.5f * 2.0f * 25.0f;  // 0.5 * m * |v|²
    
    REQUIRE_THAT(ke, WithinAbs(expected, 1e-4f));
}

TEST_CASE("RigidBody kinetic energy - angular only", "[rigid_body][energy]") {
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBody rb(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);
    
    // Sphere: I = (2/5) * m * r² = (2/5) * 1.0 * 0.25 = 0.1
    float I_val = 0.1f;
    
    rb.angular_velocity = Vec3f(1.0f, 0.0f, 0.0f);
    
    float ke = rb.kinetic_energy();
    float expected = 0.5f * I_val * 1.0f;  // 0.5 * I * ω²
    
    REQUIRE_THAT(ke, WithinAbs(expected, 1e-4f));
}

// ============================================================================
// ANGULAR INTEGRATION TESTS
// ============================================================================

TEST_CASE("Quaternion integration", "[quaternion][integration]") {
    Quatf q;  // Identity
    Vec3f omega(1.0f, 0.0f, 0.0f);  // Rotate around X-axis at 1 rad/s
    float dt = 0.1f;
    
    Quatf q_new = inertia::integrate_quaternion(q, omega, dt);
    
    // After small dt, should be close to identity
    REQUIRE_THAT(q_new.w, WithinAbs(q.w, 0.1f));
    
    // Angular velocity component should have grown
    bool changed = (q_new.x != q.x) || (q_new.y != q.y) || (q_new.z != q.z);
    REQUIRE(changed);
}

// ============================================================================
// RIGID BODY SYSTEM TESTS
// ============================================================================

TEST_CASE("RigidBodySystem spawn and retrieve", "[system][spawn]") {
    RigidBodySystem sys;
    
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBodyID id = sys.spawn_body(
        Vec3f(5.0f, 10.0f, 15.0f),
        Quatf(),
        shape,
        3.0f
    );
    
    RigidBody* rb = sys.get_body(id);
    REQUIRE(rb != nullptr);
    REQUIRE_THAT(rb->position.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(rb->get_mass(), WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("RigidBodySystem gravity simulation", "[system][gravity]") {
    RigidBodySystem sys;
    
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBodyID id = sys.spawn_body(
        Vec3f(0.0f, 10.0f, 0.0f),
        Quatf(),
        shape,
        1.0f
    );
    
    // Add gravity
    auto gravity = std::make_shared<GravityField>(Vec3f(0.0f, -9.81f, 0.0f));
    sys.add_force_field(gravity);
    
    // Simulate 1 second
    for (int i = 0; i < 100; ++i) {
        sys.update(0.01f);
    }
    
    RigidBody* rb = sys.get_body(id);
    
    // After falling 1 second with g=9.81, should be at:
    // y = 10 - 0.5 * 9.81 * 1² ≈ 5.1 meters
    // v = -9.81 m/s
    REQUIRE(rb->position.y < 10.0f);
    REQUIRE(rb->position.y > 4.0f);  // Should have fallen significantly
    REQUIRE(rb->velocity.y < -5.0f);  // Significant downward velocity
}

TEST_CASE("RigidBodySystem rotation simulation", "[system][rotation]") {
    RigidBodySystem sys;
    
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBodyID id = sys.spawn_body(
        Vec3f(0.0f, 0.0f, 0.0f),
        Quatf(),
        shape,
        1.0f
    );
    
    RigidBody* rb = sys.get_body(id);
    
    // Apply initial angular velocity
    rb->angular_velocity = Vec3f(1.0f, 0.0f, 0.0f);
    
    // Simulate
    Quatf q_initial = rb->orientation;
    for (int i = 0; i < 10; ++i) {
        sys.update(0.01f);
    }
    
    Quatf q_final = rb->orientation;
    
    // Quaternion should have changed
    REQUIRE(!(q_final == q_initial));
    
    // Preserve unit norm (should be normalized)
    float mag = q_final.magnitude();
    REQUIRE_THAT(mag, WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("RigidBodySystem energy conservation (no gravity)", "[system][energy]") {
    RigidBodySystem sys;
    
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBodyID id = sys.spawn_body(
        Vec3f(0.0f, 0.0f, 0.0f),
        Quatf(),
        shape,
        1.0f
    );
    
    RigidBody* rb = sys.get_body(id);
    
    // Set initial velocities
    rb->velocity = Vec3f(5.0f, 0.0f, 0.0f);
    rb->angular_velocity = Vec3f(0.0f, 2.0f, 0.0f);
    
    // Simulate without damping
    rb->material.linear_damping = 0.0f;
    rb->material.angular_damping = 0.0f;
    
    float ke_initial = rb->kinetic_energy();
    
    for (int i = 0; i < 100; ++i) {
        sys.update(0.01f);
    }
    
    float ke_final = rb->kinetic_energy();
    
    // Energy should be conserved (within numerical error)
    REQUIRE_THAT(ke_final, WithinAbs(ke_initial, 0.01f * ke_initial));
}

// ============================================================================
// CONSTRAINT TESTS
// ============================================================================

TEST_CASE("FixedConstraint RB error computation", "[constraint][fixed]") {
    auto shape = std::make_shared<SphereShape>(0.5f);
    
    RigidBody body_a(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);
    RigidBody body_b(Vec3f(1, 0, 0), Quatf(), shape, Material{}, 1.0f);
    
    constraints::FixedConstraintRB constraint(
        &body_a,
        &body_b,
        Vec3f(0, 0, 0),  // Anchor at origin of A
        Vec3f(-1, 0, 0)  // Anchor at origin of B (relative to center)
    );
    
    float error = constraint.compute_error();
    
    // Initial error should be zero (anchors are at same world position)
    REQUIRE_THAT(error, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("HingeConstraint RB error computation", "[constraint][hinge]") {
    auto shape = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));
    
    RigidBody body_a(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);
    RigidBody body_b(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);
    
    constraints::HingeConstraintRB constraint(
        &body_a,
        &body_b,
        Vec3f(0, 0, 0),      // Pivot at origin of A
        Vec3f(0, 0, 0),      // Pivot at origin of B
        Vec3f(0, 0, 1.0f)    // Hinge axis along Z
    );
    
    float error = constraint.compute_error();
    
    // Initially at same position, error should be minimal
    REQUIRE(error < 0.01f);
}

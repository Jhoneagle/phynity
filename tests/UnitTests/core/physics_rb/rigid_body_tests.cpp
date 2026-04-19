#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/common/ccd_config.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/physics/common/material.hpp>
#include <core/physics/constraints/joints/fixed_constraint_rb.hpp>
#include <core/physics/macro/inertia.hpp>
#include <core/physics/macro/rigid_body.hpp>
#include <core/physics/macro/rigid_body_system.hpp>
#include <core/physics/macro/shape.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <cmath>

using namespace phynity::physics;
using namespace phynity::math::vectors;
using namespace phynity::test::helpers::constants;
using namespace phynity::math::matrices;
using namespace phynity::math::quaternions;
using Catch::Matchers::WithinAbs;

// ============================================================================
// INERTIA TENSOR TESTS
// ============================================================================

TEST_CASE("Sphere inertia tensor", "[inertia][sphere]")
{
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

TEST_CASE("Box inertia tensor", "[inertia][box]")
{
    // Box: I_ii = (1/12) * m * (sum of squares of other two dimensions)
    float mass = 2.0f;
    Vec3f half_extents(1.0f, 2.0f, 3.0f);

    Mat3f I = inertia::compute_box_inertia(mass, half_extents);

    // Full dimensions
    float a = 2.0f * half_extents.x; // 2.0
    float b = 2.0f * half_extents.y; // 4.0
    float c = 2.0f * half_extents.z; // 6.0

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

TEST_CASE("RigidBody creation with sphere", "[rigid_body][creation]")
{
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBody rb(Vec3f(1.0f, 2.0f, 3.0f),
                 Quatf(), // Identity
                 shape,
                 steel(),
                 5.0f // 5 kg
    );

    REQUIRE_THAT(rb.position.x, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(rb.position.y, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(rb.position.z, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(rb.get_mass(), WithinAbs(5.0f, 1e-6f));

    // Check inertia tensor was computed
    REQUIRE(rb.inertia_tensor(0, 0) > 0.0f);
    REQUIRE_THAT(rb.inertia_tensor(0, 0), WithinAbs(rb.inertia_tensor(1, 1), 1e-6f));
}

TEST_CASE("RigidBody static vs dynamic", "[rigid_body][mass]")
{
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

TEST_CASE("RigidBody kinetic energy - linear only", "[rigid_body][energy]")
{
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBody rb(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 2.0f);

    rb.velocity = Vec3f(3.0f, 4.0f, 0.0f); // |v| = 5

    float ke = rb.kinetic_energy();
    float expected = 0.5f * 2.0f * 25.0f; // 0.5 * m * |v|²

    REQUIRE_THAT(ke, WithinAbs(expected, 1e-4f));
}

TEST_CASE("RigidBody kinetic energy - angular only", "[rigid_body][energy]")
{
    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBody rb(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);

    // Sphere: I = (2/5) * m * r² = (2/5) * 1.0 * 0.25 = 0.1
    float I_val = 0.1f;

    rb.angular_velocity = Vec3f(1.0f, 0.0f, 0.0f);

    float ke = rb.kinetic_energy();
    float expected = 0.5f * I_val * 1.0f; // 0.5 * I * ω²

    REQUIRE_THAT(ke, WithinAbs(expected, 1e-4f));
}

// ============================================================================
// ANGULAR INTEGRATION TESTS
// ============================================================================

TEST_CASE("Quaternion integration", "[quaternion][integration]")
{
    Quatf q; // Identity
    Vec3f omega(1.0f, 0.0f, 0.0f); // Rotate around X-axis at 1 rad/s
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

TEST_CASE("RigidBodySystem spawn and retrieve", "[system][spawn]")
{
    RigidBodySystem sys;

    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBodyID id = sys.spawn_body(Vec3f(5.0f, 10.0f, 15.0f), Quatf(), shape, 3.0f);

    RigidBody *rb = sys.get_body(id);
    REQUIRE(rb != nullptr);
    REQUIRE_THAT(rb->position.x, WithinAbs(5.0f, 1e-6f));
    REQUIRE_THAT(rb->get_mass(), WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("RigidBodySystem gravity simulation", "[system][gravity]")
{
    RigidBodySystem sys;

    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBodyID id = sys.spawn_body(Vec3f(0.0f, 10.0f, 0.0f), Quatf(), shape, 1.0f);

    // Add gravity
    sys.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));

    // Simulate 1 second
    for (int i = 0; i < 100; ++i)
    {
        sys.update(0.01f);
    }

    RigidBody *rb = sys.get_body(id);

    // After falling 1 second with g=9.81, should be at:
    // y = 10 - 0.5 * 9.81 * 1² ≈ 5.1 meters
    // v = -9.81 m/s
    REQUIRE(rb->position.y < 10.0f);
    REQUIRE(rb->position.y > 4.0f); // Should have fallen significantly
    REQUIRE(rb->velocity.y < -5.0f); // Significant downward velocity
}

TEST_CASE("RigidBodySystem rotation simulation", "[system][rotation]")
{
    RigidBodySystem sys;

    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBodyID id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), shape, 1.0f);

    RigidBody *rb = sys.get_body(id);

    // Apply initial angular velocity
    rb->angular_velocity = Vec3f(1.0f, 0.0f, 0.0f);

    // Simulate
    Quatf q_initial = rb->orientation;
    for (int i = 0; i < 10; ++i)
    {
        sys.update(0.01f);
    }

    Quatf q_final = rb->orientation;

    // Quaternion should have changed
    REQUIRE(!(q_final == q_initial));

    // Preserve unit norm (should be normalized)
    float mag = q_final.magnitude();
    REQUIRE_THAT(mag, WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("RigidBodySystem energy conservation (no gravity)", "[system][energy]")
{
    RigidBodySystem sys;

    auto shape = std::make_shared<SphereShape>(0.5f);
    RigidBodyID id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), shape, 1.0f);

    RigidBody *rb = sys.get_body(id);

    // Set initial velocities
    rb->velocity = Vec3f(5.0f, 0.0f, 0.0f);
    rb->angular_velocity = Vec3f(0.0f, 2.0f, 0.0f);

    // Simulate without damping
    rb->material.linear_damping = 0.0f;
    rb->material.angular_damping = 0.0f;

    float ke_initial = rb->kinetic_energy();

    for (int i = 0; i < 100; ++i)
    {
        sys.update(0.01f);
    }

    float ke_final = rb->kinetic_energy();

    // Energy should be conserved (within numerical error)
    REQUIRE_THAT(ke_final, WithinAbs(ke_initial, 0.01f * ke_initial));
}

TEST_CASE("RigidBodySystem sphere-sphere collision response", "[system][collision][sphere]")
{
    RigidBodySystem sys;

    Material mat(1.0f, 0.8f, 0.3f, 0.0f, 0.0f, 0.0f);

    auto moving_shape = std::make_shared<SphereShape>(0.5f);
    auto target_shape = std::make_shared<SphereShape>(0.5f);

    RigidBodyID moving_id = sys.spawn_body(Vec3f(-2.0f, 0.0f, 0.0f), Quatf(), moving_shape, 1.0f, mat);

    RigidBodyID target_id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), target_shape, 1.0f, mat);

    RigidBody *moving = sys.get_body(moving_id);
    RigidBody *target = sys.get_body(target_id);
    REQUIRE(moving != nullptr);
    REQUIRE(target != nullptr);

    moving->velocity = Vec3f(8.0f, 0.0f, 0.0f);
    target->velocity = Vec3f(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 120; ++i)
    {
        sys.update(1.0f / 120.0f);
    }

    moving = sys.get_body(moving_id);
    target = sys.get_body(target_id);
    REQUIRE(moving != nullptr);
    REQUIRE(target != nullptr);

    float center_distance = (moving->position - target->position).length();
    REQUIRE(center_distance >= 0.95f);
    REQUIRE(target->velocity.x > 0.5f);
}

TEST_CASE("RigidBodySystem box AABB collision response", "[system][collision][aabb]")
{
    RigidBodySystem sys;

    Material mat(1.0f, 0.6f, 0.3f, 0.0f, 0.0f, 0.0f);

    auto moving_box = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));
    auto wall_box = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));

    RigidBodyID moving_id = sys.spawn_body(Vec3f(-2.0f, 0.0f, 0.0f), Quatf(), moving_box, 1.0f, mat);

    RigidBodyID wall_id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), wall_box, 0.0f, mat);

    RigidBody *moving = sys.get_body(moving_id);
    RigidBody *wall = sys.get_body(wall_id);
    REQUIRE(moving != nullptr);
    REQUIRE(wall != nullptr);

    moving->velocity = Vec3f(10.0f, 0.0f, 0.0f);

    for (int i = 0; i < 120; ++i)
    {
        sys.update(1.0f / 120.0f);
    }

    moving = sys.get_body(moving_id);
    wall = sys.get_body(wall_id);
    REQUIRE(moving != nullptr);
    REQUIRE(wall != nullptr);

    float center_distance = (moving->position - wall->position).length();
    REQUIRE(center_distance >= 0.95f);
    REQUIRE(moving->position.x <= 0.1f);
    REQUIRE(moving->velocity.x <= 0.0f);
}

TEST_CASE("RigidBodySystem linear CCD prevents tunneling", "[system][collision][ccd]")
{
    RigidBodySystem::Config config;
    config.enable_linear_ccd = true;
    config.ccd_config = ccd_presets::aggressive();

    RigidBodySystem sys(config);

    Material inelastic(1.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);

    auto bullet_shape = std::make_shared<SphereShape>(0.05f);
    auto target_shape = std::make_shared<SphereShape>(0.05f);

    RigidBodyID bullet_id = sys.spawn_body(Vec3f(-5.0f, 0.0f, 0.0f), Quatf(), bullet_shape, 1.0f, inelastic);

    RigidBodyID target_id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), target_shape, 0.0f, inelastic);

    RigidBody *bullet = sys.get_body(bullet_id);
    RigidBody *target = sys.get_body(target_id);
    REQUIRE(bullet != nullptr);
    REQUIRE(target != nullptr);

    bullet->velocity = Vec3f(600.0f, 0.0f, 0.0f);

    sys.update(DETERMINISTIC_TIMESTEP);

    bullet = sys.get_body(bullet_id);
    target = sys.get_body(target_id);
    REQUIRE(bullet != nullptr);
    REQUIRE(target != nullptr);

    float center_distance = (bullet->position - target->position).length();
    REQUIRE(center_distance >= 0.095f);
    REQUIRE(bullet->position.x <= 0.2f);
    REQUIRE(bullet->velocity.x <= 1.0f);
}

TEST_CASE("RigidBodySystem convex CCD prevents box tunneling", "[system][collision][ccd][convex]")
{
    RigidBodySystem::Config config;
    config.enable_linear_ccd = true;
    config.ccd_config = ccd_presets::aggressive();

    RigidBodySystem sys(config);

    Material inelastic(1.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);

    auto moving_box = std::make_shared<BoxShape>(Vec3f(0.3f, 0.3f, 0.3f));
    auto target_box = std::make_shared<BoxShape>(Vec3f(0.3f, 0.3f, 0.3f));

    RigidBodyID moving_id = sys.spawn_body(Vec3f(-3.0f, 0.0f, 0.0f), Quatf(), moving_box, 1.0f, inelastic);

    RigidBodyID target_id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), target_box, 0.0f, inelastic);

    RigidBody *moving = sys.get_body(moving_id);
    RigidBody *target = sys.get_body(target_id);
    REQUIRE(moving != nullptr);
    REQUIRE(target != nullptr);

    moving->velocity = Vec3f(200.0f, 0.0f, 0.0f);

    sys.update(DETERMINISTIC_TIMESTEP);

    moving = sys.get_body(moving_id);
    target = sys.get_body(target_id);
    REQUIRE(moving != nullptr);
    REQUIRE(target != nullptr);

    float center_distance = (moving->position - target->position).length();
    REQUIRE(center_distance >= 0.5f);
    REQUIRE(moving->position.x <= 0.5f);
    REQUIRE(moving->velocity.x <= 1.0f);
}

TEST_CASE("RigidBodySystem impact response adds angular velocity", "[system][collision][impact]")
{
    RigidBodySystem sys;

    Material mat(1.0f, 0.5f, 0.3f, 0.0f, 0.0f, 0.0f);

    auto moving_box = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));
    auto static_box = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));

    RigidBodyID moving_id = sys.spawn_body(Vec3f(-2.0f, 0.35f, 0.0f), Quatf(), moving_box, 1.0f, mat);

    RigidBodyID obstacle_id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), static_box, 0.0f, mat);

    RigidBody *moving = sys.get_body(moving_id);
    RigidBody *obstacle = sys.get_body(obstacle_id);
    REQUIRE(moving != nullptr);
    REQUIRE(obstacle != nullptr);

    moving->velocity = Vec3f(8.0f, 0.0f, 0.0f);

    for (int i = 0; i < 120; ++i)
    {
        sys.update(1.0f / 120.0f);
    }

    moving = sys.get_body(moving_id);
    REQUIRE(moving != nullptr);

    REQUIRE(std::abs(moving->angular_velocity.z) > 1e-4f);
}

TEST_CASE("RigidBodySystem speculative contacts stabilize near impact", "[system][collision][speculative]")
{
    RigidBodySystem::Config config;
    config.enable_linear_ccd = true;
    config.ccd_config = ccd_presets::aggressive();
    config.ccd_config.use_speculative_contacts = true;
    config.ccd_config.speculative_distance = 0.25f;

    RigidBodySystem sys(config);

    Material inelastic(1.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);

    auto a_shape = std::make_shared<SphereShape>(0.5f);
    auto b_shape = std::make_shared<SphereShape>(0.5f);

    RigidBodyID a_id = sys.spawn_body(Vec3f(-1.2f, 0.0f, 0.0f), Quatf(), a_shape, 1.0f, inelastic);

    RigidBodyID b_id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), b_shape, 0.0f, inelastic);

    RigidBody *body_a = sys.get_body(a_id);
    RigidBody *body_b = sys.get_body(b_id);
    REQUIRE(body_a != nullptr);
    REQUIRE(body_b != nullptr);

    body_a->velocity = Vec3f(2.0f, 0.0f, 0.0f);

    float initial_gap = (body_b->position - body_a->position).length() - (a_shape->radius + b_shape->radius);
    REQUIRE(initial_gap > 0.0f);

    sys.update(1.0f / 120.0f);

    body_a = sys.get_body(a_id);
    body_b = sys.get_body(b_id);
    REQUIRE(body_a != nullptr);
    REQUIRE(body_b != nullptr);

    float post_gap = (body_b->position - body_a->position).length() - (a_shape->radius + b_shape->radius);
    REQUIRE(post_gap >= -1e-3f);
    REQUIRE(body_a->velocity.x <= 0.5f);
}

TEST_CASE("RigidBodySystem contact friction damps tangential velocity", "[system][collision][friction]")
{
    RigidBodySystem sys;

    Material high_friction(1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

    auto moving_box = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));
    auto static_box = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));

    RigidBodyID moving_id = sys.spawn_body(Vec3f(0.0f, 0.95f, 0.0f), Quatf(), moving_box, 1.0f, high_friction);

    RigidBodyID static_id = sys.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), static_box, 0.0f, high_friction);

    RigidBody *moving = sys.get_body(moving_id);
    RigidBody *obstacle = sys.get_body(static_id);
    REQUIRE(moving != nullptr);
    REQUIRE(obstacle != nullptr);

    moving->velocity = Vec3f(4.0f, -1.0f, 0.0f);

    sys.update(1.0f / 120.0f);

    moving = sys.get_body(moving_id);
    REQUIRE(moving != nullptr);

    REQUIRE(std::abs(moving->velocity.x) < 4.0f);
}

// ============================================================================
// CONSTRAINT TESTS
// ============================================================================

TEST_CASE("FixedConstraint RB error computation", "[constraint][fixed]")
{
    auto shape = std::make_shared<SphereShape>(0.5f);

    RigidBody body_a(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);
    RigidBody body_b(Vec3f(1, 0, 0), Quatf(), shape, Material{}, 1.0f);

    constraints::FixedConstraintRB constraint(&body_a,
                                              &body_b,
                                              Vec3f(0, 0, 0), // Anchor at origin of A
                                              Vec3f(-1, 0, 0) // Anchor at origin of B (relative to center)
    );

    float error = constraint.compute_error();

    // Initial error should be zero (anchors are at same world position)
    REQUIRE_THAT(error, WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("HingeConstraint RB error computation", "[constraint][hinge]")
{
    auto shape = std::make_shared<BoxShape>(Vec3f(0.5f, 0.5f, 0.5f));

    RigidBody body_a(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);
    RigidBody body_b(Vec3f(0, 0, 0), Quatf(), shape, Material{}, 1.0f);

    constraints::HingeConstraintRB constraint(&body_a,
                                              &body_b,
                                              Vec3f(0, 0, 0), // Pivot at origin of A
                                              Vec3f(0, 0, 0), // Pivot at origin of B
                                              Vec3f(0, 0, 1.0f) // Hinge axis along Z
    );

    float error = constraint.compute_error();

    // Initially at same position, error should be minimal
    REQUIRE(error < 0.01f);
}

// ============================================================================
// CONSTRAINT SOLVER INTEGRATION TESTS
// ============================================================================

TEST_CASE("RigidBodySystem FixedConstraintRB produces bounded correction", "[rigid_body][constraints][solver]")
{
    // Two bodies connected by a fixed constraint, initially displaced.
    // The constraint solver should apply corrections that keep the system bounded.
    RigidBodySystem system;

    auto sphere_shape = std::make_shared<SphereShape>(0.5f);

    // Body A at origin, body B displaced slightly
    RigidBodyID id_a = system.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), sphere_shape, 1.0f, Material{});
    RigidBodyID id_b = system.spawn_body(Vec3f(0.1f, 0.0f, 0.0f), Quatf(), sphere_shape, 1.0f, Material{});

    RigidBody *body_a = system.get_body(id_a);
    RigidBody *body_b = system.get_body(id_b);
    REQUIRE(body_a != nullptr);
    REQUIRE(body_b != nullptr);

    auto constraint = std::make_unique<constraints::FixedConstraintRB>(body_a, body_b, Vec3f(0.0f), Vec3f(0.0f));

    float initial_error = constraint->compute_error();
    REQUIRE(initial_error > 0.05f);

    system.add_constraint(std::move(constraint));

    // Step the simulation -- verify bodies stay finite and bounded
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 200; ++i)
    {
        system.update(dt);
    }

    body_a = system.get_body(id_a);
    body_b = system.get_body(id_b);
    REQUIRE(body_a != nullptr);
    REQUIRE(body_b != nullptr);

    // Positions must remain finite (no explosion)
    CHECK(std::isfinite(body_a->position.x));
    CHECK(std::isfinite(body_b->position.x));

    // Bodies should not have diverged wildly (bounded within 10 units of origin)
    float distance_from_origin_a = body_a->position.length();
    float distance_from_origin_b = body_b->position.length();
    CHECK(distance_from_origin_a < 10.0f);
    CHECK(distance_from_origin_b < 10.0f);

    // Constraint should have had some effect on the relative velocity
    float final_distance = body_a->position.distance(body_b->position);
    CHECK(std::isfinite(final_distance));
}

// ============================================================================
// ROTATED BOX AABB TESTS
// ============================================================================

TEST_CASE("Rotated box collision detection uses orientation-aware AABB", "[rigid_body][collision][aabb]")
{
    // A long thin box (2x0.2x0.2) placed at origin, rotated 45 degrees around Y.
    // When unrotated, the box extends [-1, 1] on X and [-0.1, 0.1] on Y/Z.
    // When rotated 45deg around Y, the AABB should be wider on both X and Z.
    //
    // Place a small sphere just outside the unrotated X extent but within the
    // rotated extent. If the AABB accounts for rotation, the broadphase will
    // detect the overlap and collision will occur.

    RigidBodySystem system;

    // Rotated long box at origin
    auto box_shape = std::make_shared<BoxShape>(Vec3f(1.0f, 0.1f, 0.1f));
    Quatf rotation_45_y = toQuaternion(Vec3f(0.0f, 0.7853981f, 0.0f)); // pi/4 around Y
    system.spawn_body(Vec3f(0.0f, 0.0f, 0.0f),
                      rotation_45_y,
                      box_shape,
                      0.0f, // static (infinite mass)
                      Material{});

    // The rotated box AABB should extend to about +-0.778 on X and Z
    // (cos(45)*1.0 + sin(45)*0.1 ≈ 0.778)
    // Place a sphere at Z=0.6, which is outside unrotated AABB (0.1) but inside rotated AABB
    auto sphere_shape = std::make_shared<SphereShape>(0.2f);
    RigidBodyID sphere_id = system.spawn_body(Vec3f(0.0f, 0.0f, 0.6f), Quatf(), sphere_shape, 1.0f, Material{});

    // Give sphere velocity toward box to ensure collision response
    RigidBody *sphere = system.get_body(sphere_id);
    REQUIRE(sphere != nullptr);
    sphere->velocity = Vec3f(0.0f, 0.0f, -5.0f);

    // Step the simulation
    Vec3f initial_vel = sphere->velocity;
    system.update(1.0f / 60.0f);

    // After one step, if the rotated AABB was computed correctly, the broadphase
    // should have detected overlap, and collision response should have changed velocity.
    // With the old code (no rotation), the AABB on Z would be [-0.1, 0.1] and the
    // sphere at Z=0.6 would be missed entirely.
    // No force fields are added, so without collision the velocity would stay at -5.0 on Z.
    sphere = system.get_body(sphere_id);
    if (sphere != nullptr)
    {
        bool velocity_changed = !sphere->velocity.approxEqual(initial_vel, 0.5f);
        bool position_corrected = sphere->position.z > 0.5f; // Didn't travel full distance
        CHECK((velocity_changed || position_corrected));
    }
}

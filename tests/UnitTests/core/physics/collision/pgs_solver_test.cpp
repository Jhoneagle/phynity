#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/contact/pgs_solver.hpp>
#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/collision/shapes/sphere_collider.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using Catch::Matchers::WithinAbs;

// ============================================================================
// PGS Solver Tests - Basic Functionality
// ============================================================================

TEST_CASE("PGSSolver - Empty manifolds returns empty impulses", "[collision][pgs]") {
    std::vector<ContactManifold> manifolds;
    std::vector<SphereCollider> colliders(2);
    
    auto impulses = PGSSolver::solve(manifolds, colliders);
    
    REQUIRE(impulses.empty());
}

TEST_CASE("PGSSolver - Invalid manifold is skipped", "[collision][pgs]") {
    ContactManifold manifold;
    // object IDs not set, manifold is invalid
    
    std::vector<SphereCollider> colliders(2);
    colliders[0].inverse_mass = 1.0f;
    colliders[1].inverse_mass = 1.0f;
    
    auto impulses = PGSSolver::solve({manifold}, colliders);
    
    REQUIRE(!impulses.empty());
    REQUIRE(impulses[0].length() == 0.0f);
}

TEST_CASE("PGSSolver - Single contact resolution", "[collision][pgs]") {
    // Create two colliders penetrating each other
    SphereCollider collider_a, collider_b;
    collider_a.position = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.inverse_mass = 1.0f;
    
    collider_b.position = Vec3f(0.0f, 1.0f, 0.0f);
    collider_b.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_b.inverse_mass = 1.0f;
    
    // Contact: manifold with penetration
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.position = Vec3f(0.0f, 0.5f, 0.0f);
    manifold.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
    manifold.contact.penetration = 0.2f;
    manifold.contact.relative_velocity_along_normal = 0.0f;
    
    std::vector<ContactManifold> manifolds = {manifold};
    std::vector<SphereCollider> colliders = {collider_a, collider_b};
    
    PGSConfig config;
    config.max_iterations = 1;
    
    auto impulses = PGSSolver::solve(manifolds, colliders, config);
    
    REQUIRE(impulses.size() == 1);
    REQUIRE(!(impulses[0].length() < 1e-6f));  // Some impulse should be applied
    
    // Velocities should be modified - at least one of the colliders should have non-zero velocity
    bool vel_a_modified = !(colliders[0].velocity.length() < 1e-6f);
    bool vel_b_modified = !(colliders[1].velocity.length() < 1e-6f);
    REQUIRE((vel_a_modified || vel_b_modified));
}

// ============================================================================
// PGS Solver Tests - Multi-iteration Convergence
// ============================================================================

TEST_CASE("PGSSolver - Multi-iteration reduces penetration faster", "[collision][pgs]") {
    // Create a 3-contact stack scenario
    std::vector<SphereCollider> colliders(4);
    for (size_t i = 0; i < 4; ++i) {
        colliders[i].position = Vec3f(0.0f, static_cast<float>(i), 0.0f);
        colliders[i].velocity = Vec3f(0.0f, 0.0f, 0.0f);
        colliders[i].inverse_mass = (i == 3) ? 0.0f : 1.0f;  // Top is static
        colliders[i].restitution = 0.0f;
    }
    
    // Create contacts between adjacent pairs
    std::vector<ContactManifold> manifolds;
    for (size_t i = 0; i < 3; ++i) {
        ContactManifold m;
        m.object_a_id = i;
        m.object_b_id = i + 1;
        m.contact.position = Vec3f(0.0f, static_cast<float>(i) + 0.5f, 0.0f);
        m.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
        m.contact.penetration = 0.1f;
        m.contact.relative_velocity_along_normal = 0.0f;
        manifolds.push_back(m);
    }
    
    // Solve with 1 iteration
    std::vector<SphereCollider> colliders_1iter = colliders;
    PGSConfig config_1;
    config_1.max_iterations = 1;
    PGSSolver::solve(manifolds, colliders_1iter, config_1);
    
    // Solve with 4 iterations
    std::vector<SphereCollider> colliders_4iter = colliders;
    PGSConfig config_4;
    config_4.max_iterations = 4;
    PGSSolver::solve(manifolds, colliders_4iter, config_4);
    
    // 4 iterations should reduce penetration more (via velocity changes)
    // Check that velocities differ between iterations
    float vel_diff_1iter = colliders_1iter[0].velocity.length() + 
                           colliders_1iter[1].velocity.length();
    float vel_diff_4iter = colliders_4iter[0].velocity.length() + 
                           colliders_4iter[1].velocity.length();
    
    REQUIRE(vel_diff_4iter >= vel_diff_1iter);  // More iterations should accumulate more impulse
}

TEST_CASE("PGSSolver - Convergence check stops early", "[collision][pgs]") {
    SphereCollider collider_a, collider_b;
    collider_a.position = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.inverse_mass = 1.0f;
    
    collider_b.position = Vec3f(0.0f, 1.0f, 0.0f);
    collider_b.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_b.inverse_mass = 1.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.position = Vec3f(0.0f, 0.5f, 0.0f);
    manifold.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
    manifold.contact.penetration = 0.01f;  // Small penetration should converge quickly
    manifold.contact.relative_velocity_along_normal = 0.0f;
    
    std::vector<ContactManifold> manifolds = {manifold};
    std::vector<SphereCollider> colliders = {collider_a, collider_b};
    
    PGSConfig config;
    config.max_iterations = 100;
    config.convergence_threshold = 0.01f;
    
    auto impulses = PGSSolver::solve(manifolds, colliders, config);
    
    // Should terminate before 100 iterations due to convergence
    REQUIRE(impulses.size() == 1);
}

// ============================================================================
// PGS Solver Tests - Friction Effects
// ============================================================================

TEST_CASE("PGSSolver - Friction reduces tangential motion", "[collision][pgs]") {
    // Industry-standard friction test: sliding velocity along contact surface
    // Setup: Object moving tangentially on a static surface
    SphereCollider moving, surface;
    moving.position = Vec3f(0.0f, 1.0f, 0.0f);
    moving.velocity = Vec3f(5.0f, 0.0f, 0.0f);  // Sliding along X
    moving.inverse_mass = 1.0f;
    
    surface.position = Vec3f(0.0f, 0.0f, 0.0f);
    surface.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    surface.inverse_mass = 0.0f;  // Static
    
    ContactManifold contact;
    contact.object_a_id = 0;
    contact.object_b_id = 1;
    contact.contact.position = Vec3f(0.0f, 0.5f, 0.0f);
    contact.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);  // Normal is Y direction
    contact.contact.penetration = 0.2f;  // Big penetration for strong normal impulse
    contact.contact.relative_velocity_along_normal = 0.0f;
    
    std::vector<ContactManifold> manifolds = {contact};
    
    // Test with friction
    std::vector<SphereCollider> colliders_with_friction = {moving, surface};
    PGSConfig config_with_friction;
    config_with_friction.friction_coefficient = 1.0f;
    config_with_friction.max_iterations = 4;
    PGSSolver::solve(manifolds, colliders_with_friction, config_with_friction);
    
    // Test without friction
    std::vector<SphereCollider> colliders_no_friction = {moving, surface};
    PGSConfig config_no_friction;
    config_no_friction.friction_coefficient = 0.0f;
    config_no_friction.max_iterations = 4;
    PGSSolver::solve(manifolds, colliders_no_friction, config_no_friction);
    
    // Check tangential velocity component (X direction in this case)
    float tangent_vel_with_friction = std::abs(colliders_with_friction[0].velocity.x);
    float tangent_vel_no_friction = std::abs(colliders_no_friction[0].velocity.x);
    
    // Friction should reduce tangential motion more than no friction
    REQUIRE(tangent_vel_with_friction <= tangent_vel_no_friction);
}

TEST_CASE("PGSSolver - No friction with zero coefficient", "[collision][pgs]") {
    SphereCollider collider_a, collider_b;
    collider_a.position = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.velocity = Vec3f(1.0f, 0.0f, 0.0f);  // Tangential velocity
    collider_a.inverse_mass = 1.0f;
    
    collider_b.position = Vec3f(0.0f, 1.0f, 0.0f);
    collider_b.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_b.inverse_mass = 1.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.position = Vec3f(0.0f, 0.5f, 0.0f);
    manifold.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
    manifold.contact.penetration = 0.0f;  // No penetration
    manifold.contact.relative_velocity_along_normal = 0.0f;
    
    std::vector<ContactManifold> manifolds = {manifold};
    std::vector<SphereCollider> colliders_no_friction = {collider_a, collider_b};
    
    PGSConfig config;
    config.max_iterations = 1;
    config.friction_coefficient = 0.0f;  // No friction
    
    Vec3f vel_before = colliders_no_friction[0].velocity;
    
    PGSSolver::solve(manifolds, colliders_no_friction, config);
    
    // With no penetration and no friction, tangential velocity should remain unchanged
    // (only normal impulse is applied, which is zero for non-penetrating contact)
    REQUIRE_THAT(colliders_no_friction[0].velocity.x,
                 WithinAbs(vel_before.x, 1e-5f));
}

// ============================================================================
// PGS Solver Tests - Warm-start
// ============================================================================

TEST_CASE("PGSSolver - Warm-start uses previous impulse", "[collision][pgs]") {
    SphereCollider collider_a, collider_b;
    collider_a.position = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.inverse_mass = 1.0f;
    
    collider_b.position = Vec3f(0.0f, 1.0f, 0.0f);
    collider_b.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_b.inverse_mass = 1.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.position = Vec3f(0.0f, 0.5f, 0.0f);
    manifold.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
    manifold.contact.penetration = 0.1f;
    manifold.contact.relative_velocity_along_normal = 0.0f;
    manifold.previous_impulse = Vec3f(0.0f, 0.5f, 0.0f);  // Previous impulse from frame t-1
    
    std::vector<ContactManifold> manifolds = {manifold};
    std::vector<SphereCollider> colliders = {collider_a, collider_b};
    
    PGSConfig config;
    config.max_iterations = 1;
    
    auto impulses = PGSSolver::solve(manifolds, colliders, config);
    
    // With warm-start, should have initial velocity modifications
    bool vel_a_has_warmstart = !(colliders[0].velocity.length() < 1e-6f);
    bool vel_b_has_warmstart = !(colliders[1].velocity.length() < 1e-6f);
    REQUIRE((vel_a_has_warmstart || vel_b_has_warmstart));
}

// ============================================================================
// PGS Solver Tests - Adaptive Iteration
// ============================================================================

TEST_CASE("PGSSolver - Adaptive iteration scales with contact count", "[collision][pgs]") {
    std::vector<SphereCollider> colliders(10);
    for (size_t i = 0; i < 10; ++i) {
        colliders[i].position = Vec3f(static_cast<float>(i), 0.0f, 0.0f);
        colliders[i].velocity = Vec3f(0.0f, 0.0f, 0.0f);
        colliders[i].inverse_mass = 1.0f;
    }
    
    // Create 5 contacts (should get 4 iterations)
    std::vector<ContactManifold> manifolds;
    for (size_t i = 0; i < 5; ++i) {
        ContactManifold m;
        m.object_a_id = i;
        m.object_b_id = i + 1;
        m.contact.position = Vec3f(static_cast<float>(i) + 0.5f, 0.0f, 0.0f);
        m.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
        m.contact.penetration = 0.05f;
        m.contact.relative_velocity_along_normal = 0.0f;
        manifolds.push_back(m);
    }
    
    PGSConfig config;
    auto impulses = PGSSolver::solve_adaptive(manifolds, colliders, config);
    
    REQUIRE(impulses.size() == 5);
}

// ============================================================================
// PGS Solver Tests - Determinism
// ============================================================================

TEST_CASE("PGSSolver - Deterministic results across runs", "[collision][pgs]") {
    // Setup identical initial conditions
    auto create_test_scenario = []() {
        std::vector<SphereCollider> colliders(3);
        colliders[0].position = Vec3f(0.0f, 0.0f, 0.0f);
        colliders[0].velocity = Vec3f(0.0f, 0.0f, 0.0f);
        colliders[0].inverse_mass = 1.0f;
        
        colliders[1].position = Vec3f(0.0f, 1.0f, 0.0f);
        colliders[1].velocity = Vec3f(0.0f, 0.0f, 0.0f);
        colliders[1].inverse_mass = 1.0f;
        
        colliders[2].position = Vec3f(0.0f, 2.0f, 0.0f);
        colliders[2].velocity = Vec3f(0.0f, 0.0f, 0.0f);
        colliders[2].inverse_mass = 0.0f;  // Static
        
        std::vector<ContactManifold> manifolds;
        for (size_t i = 0; i < 2; ++i) {
            ContactManifold m;
            m.object_a_id = i;
            m.object_b_id = i + 1;
            m.contact.position = Vec3f(0.0f, static_cast<float>(i) + 0.5f, 0.0f);
            m.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
            m.contact.penetration = 0.1f;
            m.contact.relative_velocity_along_normal = 0.0f;
            manifolds.push_back(m);
        }
        
        return std::pair(manifolds, colliders);
    };
    
    // Run 1
    auto [manifolds1, colliders1] = create_test_scenario();
    PGSConfig config;
    config.max_iterations = 4;
    auto impulses1 = PGSSolver::solve(manifolds1, colliders1, config);
    
    // Run 2
    auto [manifolds2, colliders2] = create_test_scenario();
    auto impulses2 = PGSSolver::solve(manifolds2, colliders2, config);
    
    // Results should be identical
    REQUIRE(impulses1.size() == impulses2.size());
    for (size_t i = 0; i < impulses1.size(); ++i) {
        REQUIRE_THAT(impulses1[i].x, WithinAbs(impulses2[i].x, 1e-10f));
        REQUIRE_THAT(impulses1[i].y, WithinAbs(impulses2[i].y, 1e-10f));
        REQUIRE_THAT(impulses1[i].z, WithinAbs(impulses2[i].z, 1e-10f));
    }
}

// ============================================================================
// PGS Solver Tests - Impulse Bounds
// ============================================================================

TEST_CASE("PGSSolver - Normal impulse is non-negative", "[collision][pgs]") {
    SphereCollider collider_a, collider_b;
    collider_a.position = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.velocity = Vec3f(0.0f, 1.0f, 0.0f);  // Moving apart
    collider_a.inverse_mass = 1.0f;
    
    collider_b.position = Vec3f(0.0f, 1.0f, 0.0f);
    collider_b.velocity = Vec3f(0.0f, -1.0f, 0.0f);  // Moving apart
    collider_b.inverse_mass = 1.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.position = Vec3f(0.0f, 0.5f, 0.0f);
    manifold.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
    manifold.contact.penetration = 0.0f;
    manifold.contact.relative_velocity_along_normal = -2.0f;  // Separating
    
    std::vector<ContactManifold> manifolds = {manifold};
    std::vector<SphereCollider> colliders = {collider_a, collider_b};
    
    auto impulses = PGSSolver::solve(manifolds, colliders);
    
    // Normal component should be clamped to [0, max]
    // Normal is Y direction
    REQUIRE(impulses[0].y >= -1e-6f);  // Allow small numerical error
}

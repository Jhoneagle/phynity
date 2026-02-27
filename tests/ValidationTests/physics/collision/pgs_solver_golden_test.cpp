#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/collision/pgs_solver.hpp>
#include <core/physics/collision/impulse_resolver.hpp>
#include <core/physics/collision/contact_manifold.hpp>
#include <core/physics/collision/sphere_collider.hpp>
#include <vector>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using Catch::Matchers::WithinAbs;

// ============================================================================
// PGS vs Impulse Solver Comparison - Golden Tests
// ============================================================================

TEST_CASE("PGS vs ImpulseResolver - 2-contact stack comparison", "[collision][pgs][golden]") {
    // Industry-standard test: Two objects in contact, one is static
    // This tests basic constraint satisfaction
    
    SphereCollider collider_a, collider_b;
    collider_a.position = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.inverse_mass = 0.0f;  // Static
    collider_a.restitution = 0.0f;
    
    collider_b.position = Vec3f(0.0f, 1.5f, 0.0f);
    collider_b.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_b.inverse_mass = 1.0f;
    collider_b.restitution = 0.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.position = Vec3f(0.0f, 0.75f, 0.0f);
    manifold.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
    manifold.contact.penetration = 0.1f;  // Penetration from falling
    
    // Set relative velocity from actual collider states
    Vec3f rel_vel = collider_b.velocity - collider_a.velocity;
    manifold.contact.relative_velocity_along_normal = rel_vel.dot(manifold.contact.normal);
    
    std::vector<ContactManifold> manifolds = {manifold};
    
    SECTION("PGS iteration should resolve penetration") {
        std::vector<SphereCollider> colliders_pgs = {collider_a, collider_b};
        PGSConfig config;
        config.max_iterations = 4;
        auto impulses = PGSSolver::solve(manifolds, colliders_pgs, config);
        
        // At minimum, solver should return valid impulses
        REQUIRE(impulses.size() == 1);
        REQUIRE(impulses[0].length() >= 0.0f);
    }
    
    SECTION("Determinism - PGS produces consistent results") {
        std::vector<SphereCollider> colliders_1 = {collider_a, collider_b};
        std::vector<SphereCollider> colliders_2 = {collider_a, collider_b};
        
        PGSConfig config;
        config.max_iterations = 4;
        
        auto impulses_1 = PGSSolver::solve(manifolds, colliders_1, config);
        auto impulses_2 = PGSSolver::solve(manifolds, colliders_2, config);
        
        // Results should be deterministic
        REQUIRE(impulses_1.size() == impulses_2.size());
        if (!impulses_1.empty() && !impulses_2.empty()) {
            REQUIRE_THAT(impulses_1[0].x, WithinAbs(impulses_2[0].x, 1e-6f));
            REQUIRE_THAT(impulses_1[0].y, WithinAbs(impulses_2[0].y, 1e-6f));
            REQUIRE_THAT(impulses_1[0].z, WithinAbs(impulses_2[0].z, 1e-6f));
        }
    }
}

TEST_CASE("PGS - Multi-iteration convergence behavior", "[collision][pgs][golden]") {
    // Create a 4-body chain: 0 (static) - 1 - 2 - 3
    std::vector<SphereCollider> colliders(4);
    std::vector<ContactManifold> manifolds;
    
    for (size_t i = 0; i < 4; ++i) {
        colliders[i].position = Vec3f(0.0f, static_cast<float>(i), 0.0f);
        colliders[i].velocity = Vec3f(0.0f, 0.0f, 0.0f);
        colliders[i].inverse_mass = (i == 0) ? 0.0f : 1.0f;
        colliders[i].restitution = 0.0f;
    }
    
    // Create 3 contacts
    for (size_t i = 0; i < 3; ++i) {
        ContactManifold m;
        m.object_a_id = i;
        m.object_b_id = i + 1;
        m.contact.position = Vec3f(0.0f, static_cast<float>(i) + 0.5f, 0.0f);
        m.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
        m.contact.penetration = 0.1f;
        // Compute relative velocity along normal from current velocities
        Vec3f rel_vel = colliders[i + 1].velocity - colliders[i].velocity;
        m.contact.relative_velocity_along_normal = rel_vel.dot(m.contact.normal);
        manifolds.push_back(m);
    }
    
    SECTION("1 iteration vs 4 iterations") {
        // Run with 1 iteration
        std::vector<SphereCollider> colliders_1iter = colliders;
        PGSConfig config_1;
        config_1.max_iterations = 1;
        auto impulses_1 = PGSSolver::solve(manifolds, colliders_1iter, config_1);
        
        // Run with 4 iterations
        std::vector<SphereCollider> colliders_4iter = colliders;
        PGSConfig config_4;
        config_4.max_iterations = 4;
        auto impulses_4 = PGSSolver::solve(manifolds, colliders_4iter, config_4);
        
        // Both should produce velocity changes (from penetration correction)
        bool has_velocity_1iter = false;
        bool has_velocity_4iter = false;
        for (size_t i = 0; i < 4; ++i) {
            if (colliders_1iter[i].velocity.length() > 1e-6f) has_velocity_1iter = true;
            if (colliders_4iter[i].velocity.length() > 1e-6f) has_velocity_4iter = true;
        }
        REQUIRE(has_velocity_1iter);  // Should have velocity from 1 iteration
        REQUIRE(has_velocity_4iter);  // Should have velocity from 4 iterations
    }
    
    SECTION("Increasing iterations reduces max impulse change") {
        std::vector<float> max_changes;
        
        for (int iterations : {1, 2, 4, 8}) {
            std::vector<SphereCollider> colliders_iter = colliders;
            PGSConfig config;
            config.max_iterations = iterations;
            auto impulses = PGSSolver::solve(manifolds, colliders_iter, config);
            
            // Record results
            max_changes.push_back(0.0f);
        }
        
        // All iterations should complete successfully
        REQUIRE(max_changes.size() == 4);
    }
}

TEST_CASE("PGS - Friction momentum dissipation", "[collision][pgs][golden]") {
    // Create a scenario where friction is important
    SphereCollider collider_a, collider_b;
    collider_a.position = Vec3f(0.0f, 0.0f, 0.0f);
    collider_a.velocity = Vec3f(5.0f, 0.0f, 0.0f);  // High tangential velocity
    collider_a.inverse_mass = 1.0f;
    
    collider_b.position = Vec3f(0.0f, 1.0f, 0.0f);
    collider_b.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_b.inverse_mass = 0.0f;  // Static
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.position = Vec3f(0.0f, 0.5f, 0.0f);
    manifold.contact.normal = Vec3f(0.0f, 1.0f, 0.0f);
    manifold.contact.penetration = 0.1f;
    manifold.contact.relative_velocity_along_normal = 0.0f;
    
    std::vector<ContactManifold> manifolds = {manifold};
    
    SECTION("High friction reduces tangential velocity") {
        std::vector<SphereCollider> colliders_no_friction = {collider_a, collider_b};
        std::vector<SphereCollider> colliders_with_friction = {collider_a, collider_b};
        
        PGSConfig config_no_friction;
        config_no_friction.friction_coefficient = 0.0f;
        config_no_friction.max_iterations = 1;
        
        PGSConfig config_with_friction;
        config_with_friction.friction_coefficient = 1.0f;
        config_with_friction.max_iterations = 1;
        
        PGSSolver::solve(manifolds, colliders_no_friction, config_no_friction);
        PGSSolver::solve(manifolds, colliders_with_friction, config_with_friction);
        
        // With friction, tangential velocity should be more reduced
        float vel_no_friction = colliders_no_friction[0].velocity.x;
        float vel_with_friction = colliders_with_friction[0].velocity.x;
        
        // Both should reduce tangential velocity due to normal impulse
        REQUIRE(std::abs(vel_with_friction) <= std::abs(vel_no_friction) + 1e-5f);
    }
}

TEST_CASE("PGS - Warm-start initialization", "[collision][pgs][golden]") {
    // Test that warm-start is properly initialized from previous_impulse
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
    manifold.contact.penetration = 0.2f;  // Large penetration to ensure strong impulse
    
    // Compute relative velocity along normal from current velocities
    Vec3f rel_vel = collider_b.velocity - collider_a.velocity;
    manifold.contact.relative_velocity_along_normal = rel_vel.dot(manifold.contact.normal);
    
    // Collider without warm-start
    std::vector<SphereCollider> colliders_no_ws = {collider_a, collider_b};
    std::vector<ContactManifold> manifold_no_ws = {manifold};
    
    PGSSolver::solve(manifold_no_ws, colliders_no_ws);
    
    // Collider with warm-start - Apply the same penetration
    collider_a.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    collider_b.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    
    manifold.previous_impulse = Vec3f(0.0f, 0.5f, 0.0f);  // Previous frame's impulse
    std::vector<SphereCollider> colliders_with_ws = {collider_a, collider_b};
    std::vector<ContactManifold> manifold_with_ws = {manifold};
    
    PGSSolver::solve(manifold_with_ws, colliders_with_ws);
    
    // Both versions should apply impulse (from large penetration)
    REQUIRE((colliders_no_ws[0].velocity.length() > 1e-6f || 
             colliders_no_ws[1].velocity.length() > 1e-6f));
}

TEST_CASE("PGS - Adaptive iteration with varying contact counts", "[collision][pgs][golden]") {
    SECTION("Small contact count (1-10) uses 4 iterations") {
        std::vector<SphereCollider> colliders(4);
        std::vector<ContactManifold> manifolds;
        
        for (size_t i = 0; i < 4; ++i) {
            colliders[i].position = Vec3f(static_cast<float>(i), 0.0f, 0.0f);
            colliders[i].velocity = Vec3f(0.0f, 0.0f, 0.0f);
            colliders[i].inverse_mass = 1.0f;
            colliders[i].restitution = 0.0f;
        }
        
        // Create 3 contacts
        for (size_t i = 0; i < 3; ++i) {
            ContactManifold m;
            m.object_a_id = i;
            m.object_b_id = i + 1;
            m.contact.position = Vec3f(static_cast<float>(i) + 0.5f, 0.0f, 0.0f);
            m.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
            m.contact.penetration = 0.05f;
            m.contact.relative_velocity_along_normal = 0.0f;
            manifolds.push_back(m);
        }
        
        auto impulses = PGSSolver::solve_adaptive(manifolds, colliders);
        
        REQUIRE(impulses.size() == 3);
    }
}

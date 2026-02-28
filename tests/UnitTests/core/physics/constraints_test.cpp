#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/constraints/constraint.hpp>
#include <core/physics/constraints/contact_constraint.hpp>
#include <core/physics/constraints/fixed_constraint.hpp>
#include <core/physics/constraints/constraint_solver.hpp>
#include <core/physics/particle.hpp>
#include <core/physics/collision/contact_manifold.hpp>
#include <cmath>

using phynity::physics::Particle;
using phynity::physics::Material;
using phynity::physics::constraints::Constraint;
using phynity::physics::constraints::ContactConstraint;
using phynity::physics::constraints::FixedConstraint;
using phynity::physics::constraints::ConstraintSolver;
using phynity::physics::constraints::ConstraintSolverConfig;
using phynity::physics::collision::ContactManifold;
using phynity::physics::collision::ContactPoint;
using phynity::math::vectors::Vec3f;
using Catch::Matchers::WithinAbs;

// ============================================================================
// ContactConstraint Tests
// ============================================================================

TEST_CASE("ContactConstraint: Construction from manifold", "[constraint][contact]") {
    // Create two particles
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    // Create contact manifold
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.position = Vec3f(0.5f, 0.0f, 0.0f);
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.contact.penetration = 0.1f;
    manifold.contact.relative_velocity_along_normal = 2.0f;
    manifold.update_contact_id();
    
    // Create constraint
    ContactConstraint constraint(manifold, p_a, p_b);
    
    REQUIRE(constraint.is_active());
    REQUIRE(constraint.get_contact_id() == manifold.contact_id);
}

TEST_CASE("ContactConstraint: Compute error", "[constraint][contact]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.penetration = 0.05f;
    manifold.update_contact_id();
    
    ContactConstraint constraint(manifold, p_a, p_b);
    
    float error = constraint.compute_error();
    REQUIRE_THAT(error, WithinAbs(0.05f, 1e-6f));
}

TEST_CASE("ContactConstraint: Compute Jacobian", "[constraint][contact]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.update_contact_id();
    
    ContactConstraint constraint(manifold, p_a, p_b);
    
    auto jacobian = constraint.compute_jacobian();
    
    REQUIRE(jacobian.numRows() == 1);
    REQUIRE(jacobian.numCols() == 6);  // 3 linear for A, 3 linear for B
    
    // Normal should be negated for A, positive for B
    REQUIRE_THAT(jacobian(0, 0), WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(jacobian(0, 3), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("ContactConstraint: Apply impulse", "[constraint][contact]") {
    // Create particles with initial velocities
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f));
    p_b.material.mass = 2.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.update_contact_id();
    
    ContactConstraint constraint(manifold, p_a, p_b);
    
    // Apply impulse
    constraint.apply_impulse(0.5f);
    
    // Check velocities changed (direction depends on mass)
    // p_a should be pushed backward (negative x)
    // p_b should be pushed forward (positive x)
    REQUIRE(p_a.velocity.x < 1.0f);  // Reduced
    REQUIRE(p_b.velocity.x > -1.0f);  // Increased (less negative)
    
    // Check accumulated impulse
    REQUIRE_THAT(constraint.get_accumulated_impulse(), WithinAbs(0.5f, 1e-6f));
}

TEST_CASE("ContactConstraint: Warm-start", "[constraint][contact]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.update_contact_id();
    
    ContactConstraint constraint(manifold, p_a, p_b);
    constraint.set_warm_start_impulse(0.3f);
    
    REQUIRE(constraint.get_accumulated_impulse() == 0.0f);  // Not applied yet
}

// ============================================================================
// FixedConstraint Tests
// ============================================================================

TEST_CASE("FixedConstraint: Construction", "[constraint][fixed]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    
    Particle p_b(Vec3f(2.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    FixedConstraint constraint(p_a, p_b);
    
    REQUIRE_THAT(constraint.get_rest_distance(), WithinAbs(2.0f, 1e-6f));
    REQUIRE(constraint.is_active());
}

TEST_CASE("FixedConstraint: Compute error - no violation", "[constraint][fixed]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    
    Particle p_b(Vec3f(2.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    FixedConstraint constraint(p_a, p_b);
    
    float error = constraint.compute_error();
    REQUIRE_THAT(error, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("FixedConstraint: Compute error - with violation", "[constraint][fixed]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    
    Particle p_b(Vec3f(2.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    FixedConstraint constraint(p_a, p_b);  // Stores rest distance = 2.0
    
    // Move p_b further away
    p_b.position = Vec3f(3.0f, 0.0f, 0.0f);
    
    float error = constraint.compute_error();
    REQUIRE_THAT(error, WithinAbs(1.0f, 1e-6f));  // 3.0 - 2.0 = 1.0
}

TEST_CASE("FixedConstraint: Apply impulse", "[constraint][fixed]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    
    Particle p_b(Vec3f(3.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    // Move p_b further away so we have a constraint violation
    FixedConstraint constraint(p_a, p_b);  // Rest distance = 3.0
    p_b.position = Vec3f(5.0f, 0.0f, 0.0f);  // Now distance = 5.0, error = 2.0
    
    // Apply impulse to correct the distance
    constraint.apply_impulse(1.0f);
    
    // p_a should be pushed in -x direction (toward p_b)
    // p_b should be pushed in +x direction (away from p_a)
    REQUIRE(p_a.velocity.x < 0.0f);
    REQUIRE(p_b.velocity.x > 0.0f);
}

TEST_CASE("FixedConstraint: Stays active with live particles", "[constraint][fixed]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    p_a.active = true;
    
    Particle p_b(Vec3f(2.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    p_b.active = true;
    
    FixedConstraint constraint(p_a, p_b);
    
    REQUIRE(constraint.is_active());
}

TEST_CASE("FixedConstraint: Becomes inactive with dead particle", "[constraint][fixed]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    p_a.lifetime = 0.0f;  // Dead particle (lifetime = 0)
    
    Particle p_b(Vec3f(2.0f, 0.0f, 0.0f));
    p_b.material.mass = 1.0f;
    
    FixedConstraint constraint(p_a, p_b);
    
    REQUIRE_FALSE(constraint.is_active());
}

// ============================================================================
// ConstraintSolver Tests
// ============================================================================

TEST_CASE("ConstraintSolver: Single contact constraint", "[solver][constraint]") {
    // Two particles approaching each other
    std::vector<Particle> particles;
    particles.push_back(Particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f)));
    particles.push_back(Particle(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f)));
    
    particles[0].material.mass = 1.0f;
    particles[1].material.mass = 1.0f;
    
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.contact.penetration = 0.1f;
    manifold.update_contact_id();
    
    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.push_back(std::make_unique<ContactConstraint>(manifold, particles[0], particles[1]));
    
    ConstraintSolver solver;
    solver.solve(constraints, particles);
    
    // After solving, the relative velocity along normal should be reduced
    // (particles should stop approaching)
    Vec3f relative_vel = particles[1].velocity - particles[0].velocity;
    float vel_along_normal = relative_vel.dot(Vec3f(1.0f, 0.0f, 0.0f));
    
    REQUIRE(vel_along_normal < 2.0f);  // Was 2.0 before solving
}

TEST_CASE("ConstraintSolver: Fixed constraint convergence", "[solver][constraint]") {
    // Two particles separated too far (constraint violation)
    std::vector<Particle> particles;
    particles.push_back(Particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f)));
    particles.push_back(Particle(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f)));
    
    particles[0].material.mass = 1.0f;
    particles[1].material.mass = 1.0f;
    
    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.push_back(std::make_unique<FixedConstraint>(particles[0], particles[1]));  // Rest distance = 2.0
    
    // Now move particle B further away
    particles[1].position = Vec3f(4.0f, 0.0f, 0.0f);
    
    ConstraintSolver solver;
    ConstraintSolverConfig config;
    config.iterations = 10;
    solver.set_config(config);
    
    solver.solve(constraints, particles);
    
    // After solving, particles should have velocities that correct the separation
    // Both should move toward their rest distance (p_a moves right, p_b moves left)
    REQUIRE(particles[0].velocity.x > 0.0f);  // p_a moves right (toward p_b)
    REQUIRE(particles[1].velocity.x < 0.0f);  // p_b moves left (toward p_a)
}

TEST_CASE("ConstraintSolver: Empty constraint list", "[solver][constraint]") {
    std::vector<std::unique_ptr<Constraint>> constraints;
    std::vector<Particle> particles;
    
    ConstraintSolver solver;
    
    // Should not crash with empty lists
    solver.solve(constraints, particles);
    
    REQUIRE(constraints.empty());
    REQUIRE(particles.empty());
}

TEST_CASE("ConstraintSolver: Configuration", "[solver][constraint]") {
    ConstraintSolver solver;
    ConstraintSolverConfig config;
    config.iterations = 8;
    config.convergence_threshold = 1e-4f;
    config.use_warm_start = false;
    
    solver.set_config(config);
    
    const auto& retrieved_config = solver.config();
    REQUIRE(retrieved_config.iterations == 8);
    REQUIRE_THAT(retrieved_config.convergence_threshold, WithinAbs(1e-4f, 1e-9f));
    REQUIRE_FALSE(retrieved_config.use_warm_start);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_CASE("Constraint: Contact + Fixed constraint together", "[constraint][integration]") {
    // Three particles: A and B in contact, C fixed to B
    std::vector<Particle> particles;
    particles.push_back(Particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f)));
    particles.push_back(Particle(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f)));
    particles.push_back(Particle(Vec3f(2.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f)));
    
    particles[0].material.mass = 1.0f;
    particles[1].material.mass = 1.0f;
    particles[2].material.mass = 1.0f;
    
    // Contact between A and B
    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.contact.penetration = 0.1f;
    manifold.update_contact_id();
    
    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.push_back(std::make_unique<ContactConstraint>(manifold, particles[0], particles[1]));
    constraints.push_back(std::make_unique<FixedConstraint>(particles[1], particles[2]));
    
    ConstraintSolver solver;
    solver.solve(constraints, particles);
    
    // All particles should have changed their velocities
    REQUIRE(particles[0].velocity != Vec3f(1.0f, 0.0f, 0.0f));
    REQUIRE(particles[1].velocity != Vec3f(-1.0f, 0.0f, 0.0f));
    REQUIRE(particles[2].velocity != Vec3f(0.0f, 0.0f, 0.0f));
}

TEST_CASE("Constraint: Determinism - same input same result", "[constraint][determinism]") {
    // Test that the same scenario produces the same result
    
    auto run_scenario = []() {
        std::vector<Particle> particles;
        particles.push_back(Particle(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f)));
        particles.push_back(Particle(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f)));
        
        particles[0].material.mass = 1.0f;
        particles[1].material.mass = 1.0f;
        
        ContactManifold manifold;
        manifold.object_a_id = 0;
        manifold.object_b_id = 1;
        manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
        manifold.contact.penetration = 0.1f;
        manifold.update_contact_id();
        
        std::vector<std::unique_ptr<Constraint>> constraints;
        constraints.push_back(std::make_unique<ContactConstraint>(manifold, particles[0], particles[1]));
        
        ConstraintSolver solver;
        solver.solve(constraints, particles);
        
        return std::make_pair(particles[0].velocity, particles[1].velocity);
    };
    
    auto result1 = run_scenario();
    auto result2 = run_scenario();
    
    REQUIRE_THAT(result1.first.x, WithinAbs(result2.first.x, 1e-9f));
    REQUIRE_THAT(result1.second.x, WithinAbs(result2.second.x, 1e-9f));
}

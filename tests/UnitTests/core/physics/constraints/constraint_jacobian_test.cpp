#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/collision/contact/contact_manifold.hpp>
#include <core/physics/constraints/contact_constraint.hpp>
#include <core/physics/constraints/fixed_joint.hpp>
#include <core/physics/particles/particle.hpp>

using Catch::Matchers::WithinAbs;
using phynity::math::vectors::Vec3f;
using phynity::physics::Particle;
using phynity::physics::collision::ContactManifold;
using phynity::physics::constraints::ContactConstraint;
using phynity::physics::constraints::DistanceJoint;

TEST_CASE("Constraint Jv: ContactConstraint computes relative velocity along normal", "[constraint][jv]")
{
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f));
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f));

    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.contact.penetration = 0.05f;
    manifold.update_contact_id();

    ContactConstraint constraint(manifold, p_a, p_b);

    // Jv = normal . (v_b - v_a) = (1,0,0) . (2-(-1), 0, 0) = 3.0
    REQUIRE_THAT(constraint.compute_jv(), WithinAbs(3.0f, 1e-6f));
}

TEST_CASE("Constraint effective mass: ContactConstraint sums inverse masses", "[constraint][effective_mass]")
{
    Particle p_a(Vec3f(0.0f));
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f));
    p_a.material.mass = 2.0f;
    p_b.material.mass = 4.0f;

    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.contact.penetration = 0.05f;
    manifold.update_contact_id();

    ContactConstraint constraint(manifold, p_a, p_b);

    // effective_mass = 1/2 + 1/4 = 0.75
    REQUIRE_THAT(constraint.compute_effective_mass(), WithinAbs(0.75f, 1e-6f));
}

TEST_CASE("Constraint Jv: DistanceJoint computes relative velocity along direction", "[constraint][jv]")
{
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f));
    Particle p_b(Vec3f(3.0f, 4.0f, 0.0f), Vec3f(3.0f, 4.0f, 0.0f));

    DistanceJoint constraint(p_a, p_b);

    // Direction from A to B is (0.6, 0.8, 0.0)
    // Jv = dir . (v_b - v_a) = (0.6, 0.8, 0) . (3, 4, 0) = 1.8 + 3.2 = 5.0
    REQUIRE_THAT(constraint.compute_jv(), WithinAbs(5.0f, 1e-5f));
}

TEST_CASE("Constraint Jv: DistanceJoint degenerate case returns zero", "[constraint][jv]")
{
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f));
    Particle p_b(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f));

    DistanceJoint constraint(p_a, p_b);

    // Same position = degenerate, should return 0
    REQUIRE_THAT(constraint.compute_jv(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Constraint effective mass: DistanceJoint sums inverse masses", "[constraint][effective_mass]")
{
    Particle p_a(Vec3f(0.0f));
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f));
    p_a.material.mass = 1.0f;
    p_b.material.mass = 1.0f;

    DistanceJoint constraint(p_a, p_b);

    // effective_mass = 1/1 + 1/1 = 2.0
    REQUIRE_THAT(constraint.compute_effective_mass(), WithinAbs(2.0f, 1e-6f));
}

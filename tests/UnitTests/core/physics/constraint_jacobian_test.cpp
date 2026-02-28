#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/physics/constraints/contact_constraint.hpp>
#include <core/physics/constraints/fixed_constraint.hpp>
#include <core/physics/collision/contact_manifold.hpp>
#include <core/physics/particle.hpp>
#include <core/math/vectors/vec3.hpp>

using phynity::physics::Particle;
using phynity::physics::constraints::ContactConstraint;
using phynity::physics::constraints::FixedConstraint;
using phynity::physics::collision::ContactManifold;
using phynity::math::vectors::Vec3f;
using Catch::Matchers::WithinAbs;

TEST_CASE("Constraint Jacobian: ContactConstraint matches normal", "[constraint][jacobian]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    Particle p_b(Vec3f(1.0f, 0.0f, 0.0f));

    ContactManifold manifold;
    manifold.object_a_id = 0;
    manifold.object_b_id = 1;
    manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
    manifold.contact.penetration = 0.05f;
    manifold.update_contact_id();

    ContactConstraint constraint(manifold, p_a, p_b);
    auto jacobian = constraint.compute_jacobian();

    REQUIRE(jacobian.numRows() == 1);
    REQUIRE(jacobian.numCols() == 6);
    REQUIRE_THAT(jacobian(0, 0), WithinAbs(-1.0f, 1e-6f));
    REQUIRE_THAT(jacobian(0, 3), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Constraint Jacobian: FixedConstraint direction normalized", "[constraint][jacobian]") {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    Particle p_b(Vec3f(3.0f, 4.0f, 0.0f));

    FixedConstraint constraint(p_a, p_b);
    auto jacobian = constraint.compute_jacobian();

    REQUIRE(jacobian.numRows() == 1);
    REQUIRE(jacobian.numCols() == 6);

    // Direction from A to B is (0.6, 0.8, 0.0)
    REQUIRE_THAT(jacobian(0, 0), WithinAbs(-0.6f, 1e-5f));
    REQUIRE_THAT(jacobian(0, 1), WithinAbs(-0.8f, 1e-5f));
    REQUIRE_THAT(jacobian(0, 3), WithinAbs(0.6f, 1e-5f));
    REQUIRE_THAT(jacobian(0, 4), WithinAbs(0.8f, 1e-5f));
}

TEST_CASE("Constraint Jacobian: FixedConstraint degenerate case", "[constraint][jacobian]" ) {
    Particle p_a(Vec3f(0.0f, 0.0f, 0.0f));
    Particle p_b(Vec3f(0.0f, 0.0f, 0.0f));

    FixedConstraint constraint(p_a, p_b);
    auto jacobian = constraint.compute_jacobian();

    REQUIRE(jacobian.numRows() == 1);
    REQUIRE(jacobian.numCols() == 6);

    // Degenerate case should leave row at zeros.
    REQUIRE_THAT(jacobian(0, 0), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(jacobian(0, 1), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(jacobian(0, 2), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(jacobian(0, 3), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(jacobian(0, 4), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(jacobian(0, 5), WithinAbs(0.0f, 1e-6f));
}

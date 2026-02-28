#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/common/material.hpp>

using phynity::physics::Material;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Material Constructors
// ============================================================================

TEST_CASE("Material: Default constructor", "[Material][constructor]") {
    Material m;
    REQUIRE_THAT(m.mass, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(m.restitution, WithinAbs(0.8f, 1e-6f));
    REQUIRE_THAT(m.friction, WithinAbs(0.3f, 1e-6f));
    REQUIRE_THAT(m.linear_damping, WithinAbs(0.01f, 1e-6f));
    REQUIRE_THAT(m.angular_damping, WithinAbs(0.01f, 1e-6f));
    REQUIRE_THAT(m.drag_coefficient, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("Material: Explicit constructor", "[Material][constructor]") {
    Material m(
        /*mass=*/2.5f,
        /*restitution=*/0.5f,
        /*friction=*/0.6f,
        /*linear_damping=*/0.05f,
        /*angular_damping=*/0.03f,
        /*drag_coefficient=*/0.2f
    );
    REQUIRE_THAT(m.mass, WithinAbs(2.5f, 1e-6f));
    REQUIRE_THAT(m.restitution, WithinAbs(0.5f, 1e-6f));
    REQUIRE_THAT(m.friction, WithinAbs(0.6f, 1e-6f));
    REQUIRE_THAT(m.linear_damping, WithinAbs(0.05f, 1e-6f));
    REQUIRE_THAT(m.angular_damping, WithinAbs(0.03f, 1e-6f));
    REQUIRE_THAT(m.drag_coefficient, WithinAbs(0.2f, 1e-6f));
}

// ============================================================================
// Material Presets
// ============================================================================

TEST_CASE("Material: Steel preset", "[Material][presets]") {
    auto m = phynity::physics::steel();
    
    REQUIRE_THAT(m.mass, WithinAbs(7850.0f, 1e-3f));
    REQUIRE_THAT(m.restitution, WithinAbs(0.3f, 1e-6f));
    REQUIRE(m.restitution < 0.4f);  // Verify low bounce
    REQUIRE_THAT(m.friction, WithinAbs(0.6f, 1e-6f));
    REQUIRE(m.friction > 0.5f);  // Verify high friction
    REQUIRE_THAT(m.linear_damping, WithinAbs(0.02f, 1e-6f));
    REQUIRE_THAT(m.angular_damping, WithinAbs(0.02f, 1e-6f));
    REQUIRE_THAT(m.drag_coefficient, WithinAbs(0.01f, 1e-6f));
}

TEST_CASE("Material: Rubber preset", "[Material][presets]") {
    auto m = phynity::physics::rubber();
    
    REQUIRE_THAT(m.mass, WithinAbs(1200.0f, 1e-3f));
    REQUIRE_THAT(m.restitution, WithinAbs(0.8f, 1e-6f));
    REQUIRE(m.restitution > 0.7f);  // Verify high bounce
    REQUIRE_THAT(m.friction, WithinAbs(0.4f, 1e-6f));
    REQUIRE_THAT(m.linear_damping, WithinAbs(0.01f, 1e-6f));
    REQUIRE_THAT(m.angular_damping, WithinAbs(0.01f, 1e-6f));
    REQUIRE_THAT(m.drag_coefficient, WithinAbs(0.05f, 1e-6f));
}

TEST_CASE("Material: Wood preset", "[Material][presets]") {
    auto m = phynity::physics::wood();
    
    REQUIRE_THAT(m.mass, WithinAbs(600.0f, 1e-3f));
    REQUIRE_THAT(m.restitution, WithinAbs(0.4f, 1e-6f));
    REQUIRE(m.restitution < 0.5f);  // Verify moderate bounce
    REQUIRE_THAT(m.friction, WithinAbs(0.5f, 1e-6f));
    REQUIRE_THAT(m.linear_damping, WithinAbs(0.02f, 1e-6f));
    REQUIRE_THAT(m.angular_damping, WithinAbs(0.02f, 1e-6f));
    REQUIRE_THAT(m.drag_coefficient, WithinAbs(0.03f, 1e-6f));
}

TEST_CASE("Material: Fluid particle preset", "[Material][presets]") {
    auto m = phynity::physics::fluid_particle();
    
    REQUIRE_THAT(m.mass, WithinAbs(0.1f, 1e-6f));
    REQUIRE(m.mass < 0.2f);  // Verify low mass
    REQUIRE_THAT(m.restitution, WithinAbs(0.1f, 1e-6f));
    REQUIRE(m.restitution < 0.2f);  // Verify very low bounce
    REQUIRE_THAT(m.friction, WithinAbs(0.2f, 1e-6f));
    REQUIRE_THAT(m.linear_damping, WithinAbs(0.05f, 1e-6f));
    REQUIRE_THAT(m.angular_damping, WithinAbs(0.05f, 1e-6f));
    REQUIRE_THAT(m.drag_coefficient, WithinAbs(0.5f, 1e-6f));
    REQUIRE(m.drag_coefficient > 0.4f);  // Verify high drag
}

TEST_CASE("Material: Dust preset", "[Material][presets]") {
    auto m = phynity::physics::dust();
    
    REQUIRE_THAT(m.mass, WithinAbs(0.01f, 1e-6f));
    REQUIRE(m.mass < 0.02f);  // Verify extremely low mass
    REQUIRE_THAT(m.restitution, WithinAbs(0.0f, 1e-6f));
    REQUIRE(m.restitution == 0.0f);  // Verify no bounce
    REQUIRE_THAT(m.friction, WithinAbs(0.1f, 1e-6f));
    REQUIRE_THAT(m.linear_damping, WithinAbs(0.1f, 1e-6f));
    REQUIRE_THAT(m.angular_damping, WithinAbs(0.1f, 1e-6f));
    REQUIRE(m.linear_damping > 0.08f);  // Verify high damping
    REQUIRE_THAT(m.drag_coefficient, WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("Material: Stone preset", "[Material][presets]") {
    auto m = phynity::physics::stone();
    
    REQUIRE_THAT(m.mass, WithinAbs(2500.0f, 1e-3f));
    REQUIRE_THAT(m.restitution, WithinAbs(0.15f, 1e-6f));
    REQUIRE(m.restitution < 0.2f);  // Verify very low bounce
    REQUIRE_THAT(m.friction, WithinAbs(0.7f, 1e-6f));
    REQUIRE(m.friction > 0.6f);  // Verify high friction
    REQUIRE_THAT(m.linear_damping, WithinAbs(0.01f, 1e-6f));
    REQUIRE_THAT(m.angular_damping, WithinAbs(0.01f, 1e-6f));
    REQUIRE_THAT(m.drag_coefficient, WithinAbs(0.02f, 1e-6f));
}

// ============================================================================
// Preset Comparisons
// ============================================================================

TEST_CASE("Material: Steel vs Rubber comparison", "[Material][presets][compare]") {
    auto steel = phynity::physics::steel();
    auto rubber = phynity::physics::rubber();
    
    // Steel should be denser (higher mass)
    REQUIRE(steel.mass > rubber.mass);
    
    // Rubber should have higher restitution (bouncier)
    REQUIRE(rubber.restitution > steel.restitution);
    
    // Steel should have higher friction
    REQUIRE(steel.friction > rubber.friction);
}

TEST_CASE("Material: Fluid vs Dust comparison", "[Material][presets][compare]") {
    auto fluid = phynity::physics::fluid_particle();
    auto dust = phynity::physics::dust();
    
    // Both should be light, but dust lighter
    REQUIRE(dust.mass < fluid.mass);
    
    // Dust should have higher damping (more viscous)
    REQUIRE(dust.linear_damping > fluid.linear_damping);
    
    // Dust should have higher drag
    REQUIRE(dust.drag_coefficient > fluid.drag_coefficient);
}

TEST_CASE("Material: All presets have positive mass", "[Material][presets][sanity]") {
    REQUIRE(phynity::physics::steel().mass > 0.0f);
    REQUIRE(phynity::physics::rubber().mass > 0.0f);
    REQUIRE(phynity::physics::wood().mass > 0.0f);
    REQUIRE(phynity::physics::fluid_particle().mass > 0.0f);
    REQUIRE(phynity::physics::dust().mass > 0.0f);
    REQUIRE(phynity::physics::stone().mass > 0.0f);
}

TEST_CASE("Material: All presets have restitution in [0, 1]", "[Material][presets][sanity]") {
    auto check_restitution = [](const Material& m) {
        REQUIRE(m.restitution >= 0.0f);
        REQUIRE(m.restitution <= 1.0f);
    };
    
    check_restitution(phynity::physics::steel());
    check_restitution(phynity::physics::rubber());
    check_restitution(phynity::physics::wood());
    check_restitution(phynity::physics::fluid_particle());
    check_restitution(phynity::physics::dust());
    check_restitution(phynity::physics::stone());
}

TEST_CASE("Material: All presets have friction in [0, 1]", "[Material][presets][sanity]") {
    auto check_friction = [](const Material& m) {
        REQUIRE(m.friction >= 0.0f);
        REQUIRE(m.friction <= 1.0f);
    };
    
    check_friction(phynity::physics::steel());
    check_friction(phynity::physics::rubber());
    check_friction(phynity::physics::wood());
    check_friction(phynity::physics::fluid_particle());
    check_friction(phynity::physics::dust());
    check_friction(phynity::physics::stone());
}

TEST_CASE("Material: All presets have damping in [0, 1]", "[Material][presets][sanity]") {
    auto check_damping = [](const Material& m) {
        REQUIRE(m.linear_damping >= 0.0f);
        REQUIRE(m.linear_damping <= 1.0f);
        REQUIRE(m.angular_damping >= 0.0f);
        REQUIRE(m.angular_damping <= 1.0f);
    };
    
    check_damping(phynity::physics::steel());
    check_damping(phynity::physics::rubber());
    check_damping(phynity::physics::wood());
    check_damping(phynity::physics::fluid_particle());
    check_damping(phynity::physics::dust());
    check_damping(phynity::physics::stone());
}

TEST_CASE("Material: All presets have non-negative drag", "[Material][presets][sanity]") {
    REQUIRE(phynity::physics::steel().drag_coefficient >= 0.0f);
    REQUIRE(phynity::physics::rubber().drag_coefficient >= 0.0f);
    REQUIRE(phynity::physics::wood().drag_coefficient >= 0.0f);
    REQUIRE(phynity::physics::fluid_particle().drag_coefficient >= 0.0f);
    REQUIRE(phynity::physics::dust().drag_coefficient >= 0.0f);
    REQUIRE(phynity::physics::stone().drag_coefficient >= 0.0f);
}

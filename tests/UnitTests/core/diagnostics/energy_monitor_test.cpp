#include <catch2/catch_test_macros.hpp>
#include <core/diagnostics/energy_monitor.hpp>

using namespace phynity::diagnostics;

TEST_CASE("EnergyMonitor: Basic energy tracking", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    
    // First measurement - should not trigger violations
    monitor.update(1000.0);  // 1000 J
    REQUIRE(monitor.get_current_energy() == 1000.0);
    REQUIRE(monitor.get_frame_count() == 1);
    
    // Second measurement - stable energy
    monitor.update(1000.0);  // Still 1000 J
    REQUIRE(monitor.get_frame_count() == 2);
}

TEST_CASE("EnergyMonitor: Energy loss detection", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    monitor.set_max_loss_percent(1.0);  // Allow max 1% loss
    
    int violation_count = 0;
    std::string_view violation_type;
    
    monitor.set_violation_callback([&](const EnergyViolation& v) {
        ++violation_count;
        violation_type = v.violation_type;
    });
    
    // Start with 1000 J
    monitor.update(1000.0);
    
    // Lose 5% (50 J) - should trigger violation
    monitor.update(950.0);
    
    REQUIRE(violation_count == 1);
    REQUIRE(violation_type == "loss");
}

TEST_CASE("EnergyMonitor: Energy loss within tolerance", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    monitor.set_max_loss_percent(5.0);  // Allow up to 5% loss
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&](const EnergyViolation& v) {
        ++violation_count;
    });
    
    // Start with 1000 J
    monitor.update(1000.0);
    
    // Lose 3% (30 J) - within tolerance
    monitor.update(970.0);
    
    REQUIRE(violation_count == 0);
}

TEST_CASE("EnergyMonitor: Energy gain detection", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    monitor.set_max_gain_percent(0.5);  // Allow max 0.5% gain
    
    int violation_count = 0;
    std::string_view violation_type;
    
    monitor.set_violation_callback([&](const EnergyViolation& v) {
        ++violation_count;
        violation_type = v.violation_type;
    });
    
    // Start with 1000 J
    monitor.update(1000.0);
    
    // Gain 2% (20 J) - should trigger violation
    monitor.update(1020.0);
    
    REQUIRE(violation_count == 1);
    REQUIRE(violation_type == "gain");
}

TEST_CASE("EnergyMonitor: Enable/disable monitoring", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    monitor.set_max_loss_percent(1.0);
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&](const EnergyViolation& v) {
        ++violation_count;
    });
    
    REQUIRE(monitor.is_enabled());
    
    // Disable monitoring
    monitor.set_enabled(false);
    REQUIRE_FALSE(monitor.is_enabled());
    
    monitor.update(1000.0);
    monitor.update(950.0);  // 5% loss - should not trigger since disabled
    
    REQUIRE(violation_count == 0);
    
    // Re-enable
    monitor.set_enabled(true);
    monitor.update(900.0);  // Another 5% loss - should trigger
    
    REQUIRE(violation_count == 1);
}

TEST_CASE("EnergyMonitor: Reset functionality", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    
    monitor.update(1000.0);
    monitor.update(1000.0);
    REQUIRE(monitor.get_frame_count() == 2);
    
    monitor.reset();
    REQUIRE(monitor.get_frame_count() == 0);
    REQUIRE(monitor.get_current_energy() == 0.0);
    
    // After reset, first update should not trigger violations
    monitor.update(500.0);  // New initial value
    REQUIRE(monitor.get_frame_count() == 1);
}

TEST_CASE("EnergyMonitor: Violation details", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    monitor.set_max_loss_percent(1.0);
    
    EnergyViolation captured_violation{};
    bool captured = false;
    
    monitor.set_violation_callback([&](const EnergyViolation& v) {
        captured_violation = v;
        captured = true;
    });
    
    monitor.update(1000.0);
    monitor.update(900.0);  // 10% loss
    
    REQUIRE(captured);
    REQUIRE(captured_violation.frame_number == 1);
    REQUIRE(captured_violation.previous_energy == 1000.0);
    REQUIRE(captured_violation.current_energy == 900.0);
    REQUIRE(captured_violation.energy_loss == 100.0);
    REQUIRE(captured_violation.loss_percentage == 10.0);
}

TEST_CASE("EnergyMonitor: Tolerance check method", "[diagnostics][energy_monitor]") {
    EnergyViolation violation;
    violation.loss_percentage = 2.5;
    
    REQUIRE(violation.is_within_tolerance(3.0));   // 2.5% < 3.0%
    REQUIRE_FALSE(violation.is_within_tolerance(2.0));  // 2.5% > 2.0%
    REQUIRE(violation.is_within_tolerance(2.5));   // 2.5% == 2.5%
}

TEST_CASE("EnergyMonitor: Multiple violations", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    monitor.set_max_loss_percent(2.0);
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&](const EnergyViolation& v) {
        ++violation_count;
    });
    
    monitor.update(1000.0);
    monitor.update(920.0);  // 8% loss - violates
    monitor.update(900.0);  // 2.2% loss - violates
    monitor.update(885.0);  // 1.7% loss - OK
    
    REQUIRE(violation_count == 2);
}

TEST_CASE("EnergyMonitor: Zero energy baseline", "[diagnostics][energy_monitor]") {
    EnergyMonitor monitor;
    
    // Starting from zero energy edge case
    monitor.update(0.0);
    monitor.update(0.0);
    // Should not crash on division by zero
    REQUIRE(monitor.get_current_energy() == 0.0);
}

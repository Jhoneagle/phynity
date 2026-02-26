#include <catch2/catch_test_macros.hpp>
#include <core/diagnostics/momentum_monitor.hpp>

using namespace phynity::diagnostics;

TEST_CASE("MomentumMonitor: Basic momentum tracking", "[diagnostics][momentum_monitor]") {
    MomentumMonitor monitor;
    
    // First measurement
    monitor.update(Vec3(10.0, 20.0, 30.0));
    REQUIRE(monitor.get_frame_count() == 1);
    
    // Second measurement - same momentum (conserved)
    monitor.update(Vec3(10.0, 20.0, 30.0));
    REQUIRE(monitor.get_frame_count() == 2);
}

TEST_CASE("MomentumMonitor: Momentum conservation check", "[diagnostics][momentum_monitor]") {
    MomentumMonitor monitor;
    monitor.set_max_change_magnitude(1.0);  // Very strict tolerance
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&]([[maybe_unused]] const MomentumViolation& v) {
        ++violation_count;
    });
    
    monitor.update(Vec3(100.0, 100.0, 100.0));
    
    // Small change within tolerance
    monitor.update(Vec3(100.5, 100.5, 100.5));
    REQUIRE(violation_count == 0);
    
    // Large change exceeds tolerance
    monitor.update(Vec3(130.0, 130.0, 130.0));
    REQUIRE(violation_count == 1);
}

TEST_CASE("MomentumMonitor: Violation details", "[diagnostics][momentum_monitor]") {
    MomentumMonitor monitor;
    monitor.set_max_change_magnitude(10.0);
    
    MomentumViolation captured_violation{};
    bool captured = false;
    
    monitor.set_violation_callback([&](const MomentumViolation& v) {
        captured_violation = v;
        captured = true;
    });
    
    Vec3 initial(50.0, 50.0, 50.0);
    Vec3 changed(60.0, 60.0, 60.0);
    
    monitor.update(initial);
    monitor.update(changed);
    
    REQUIRE(captured);
    REQUIRE(captured_violation.frame_number == 1);
    REQUIRE(captured_violation.previous_momentum.x == 50.0);
    REQUIRE(captured_violation.current_momentum.x == 60.0);
}

TEST_CASE("MomentumMonitor: Enable/disable", "[diagnostics][momentum_monitor]") {
    MomentumMonitor monitor;
    monitor.set_max_change_magnitude(1.0);
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&]([[maybe_unused]] const MomentumViolation& v) {
        ++violation_count;
    });
    
    REQUIRE(monitor.is_enabled());
    
    monitor.set_enabled(false);
    REQUIRE_FALSE(monitor.is_enabled());
    
    monitor.update(Vec3(100.0, 100.0, 100.0));
    monitor.update(Vec3(200.0, 200.0, 200.0));  // Large change - should not trigger
    
    REQUIRE(violation_count == 0);
    
    // Re-enable
    monitor.set_enabled(true);
    monitor.update(Vec3(300.0, 300.0, 300.0));
    
    REQUIRE(violation_count == 1);
}

TEST_CASE("MomentumMonitor: Reset functionality", "[diagnostics][momentum_monitor]") {
    MomentumMonitor monitor;
    
    monitor.update(Vec3(100.0, 100.0, 100.0));
    monitor.update(Vec3(100.0, 100.0, 100.0));
    REQUIRE(monitor.get_frame_count() == 2);
    
    monitor.reset();
    REQUIRE(monitor.get_frame_count() == 0);
    
    Vec3 p = monitor.get_current_momentum();
    REQUIRE(p.x == 0.0);
    REQUIRE(p.y == 0.0);
    REQUIRE(p.z == 0.0);
}

TEST_CASE("MomentumMonitor: 3D vector operations", "[diagnostics][momentum_monitor]") {
    Vec3 v1(1.0, 2.0, 3.0);
    Vec3 v2(4.0, 5.0, 6.0);
    
    // Addition
    Vec3 sum = v1 + v2;
    REQUIRE(sum.x == 5.0);
    REQUIRE(sum.y == 7.0);
    REQUIRE(sum.z == 9.0);
    
    // Subtraction
    Vec3 diff = v2 - v1;
    REQUIRE(diff.x == 3.0);
    REQUIRE(diff.y == 3.0);
    REQUIRE(diff.z == 3.0);
    
    // Magnitude (3D, 4, 5 might be ~7.07)
    double mag = diff.magnitude();
    REQUIRE(mag > 5.0);  // sqrt(27) ≈ 5.19
}

TEST_CASE("MomentumMonitor: Multiple violations", "[diagnostics][momentum_monitor]") {
    MomentumMonitor monitor;
    monitor.set_max_change_magnitude(5.0);
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&]([[maybe_unused]] const MomentumViolation& v) {
        ++violation_count;
    });
    
    monitor.update(Vec3(0.0, 0.0, 0.0));
    monitor.update(Vec3(10.0, 0.0, 0.0));   // Change of 10 - violates
    monitor.update(Vec3(20.0, 0.0, 0.0));   // Change of 10 - violates
    monitor.update(Vec3(23.0, 0.0, 0.0));   // Change of 3 - OK
    
    REQUIRE(violation_count == 2);
}

TEST_CASE("MomentumMonitor: Tolerance check method", "[diagnostics][momentum_monitor]") {
    MomentumViolation violation;
    violation.change_magnitude = 7.5;
    
    REQUIRE(violation.is_within_tolerance(10.0));   // 7.5 < 10.0
    REQUIRE_FALSE(violation.is_within_tolerance(5.0));  // 7.5 > 5.0
    REQUIRE(violation.is_within_tolerance(7.5));    // 7.5 == 7.5
}

TEST_CASE("MomentumMonitor: Negative momentum values", "[diagnostics][momentum_monitor]") {
    MomentumMonitor monitor;
    monitor.set_max_change_magnitude(10.0);
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&]([[maybe_unused]] const MomentumViolation& v) {
        ++violation_count;
    });
    
    monitor.update(Vec3(-100.0, -50.0, 25.0));
    monitor.update(Vec3(-100.0, -50.0, 25.0));  // No change
    
    REQUIRE(violation_count == 0);
    
    // Change momentum while preserving magnitude
    monitor.update(Vec3(-80.0, -50.0, 25.0));  // Change of 20 - violates
    
    REQUIRE(violation_count == 1);
}

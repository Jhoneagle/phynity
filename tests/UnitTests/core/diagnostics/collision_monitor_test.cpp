#include <catch2/catch_test_macros.hpp>
#include "../../../../src/core/diagnostics/collision_monitor.hpp"

using namespace phynity::diagnostics;

TEST_CASE("CollisionMonitor: Basic tracking", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    
    REQUIRE(monitor.get_frame_count() == 0);
    REQUIRE(monitor.is_enabled());
    
    // Set collision data for frame
    monitor.set_broadphase_candidates(100);
    monitor.set_narrowphase_tests(100);
    monitor.set_actual_collisions(10);
    
    REQUIRE(monitor.get_current_broadphase_candidates() == 100);
    REQUIRE(monitor.get_current_narrowphase_tests() == 100);
    REQUIRE(monitor.get_current_actual_collisions() == 10);
    
    // End frame
    monitor.end_frame();
    REQUIRE(monitor.get_frame_count() == 1);
    
    // Counters should be reset
    REQUIRE(monitor.get_current_broadphase_candidates() == 0);
    REQUIRE(monitor.get_current_narrowphase_tests() == 0);
    REQUIRE(monitor.get_current_actual_collisions() == 0);
}

TEST_CASE("CollisionMonitor: Efficiency calculation", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(10);
    monitor.end_frame();
    
    CollisionStats stats = monitor.get_last_frame_stats();
    
    REQUIRE(stats.frame_number == 1);
    REQUIRE(stats.broadphase_candidates == 100);
    REQUIRE(stats.actual_collisions == 10);
    REQUIRE(stats.efficiency == 0.1);  // 10/100 = 10%
    REQUIRE(stats.false_positive_rate == 0.9);  // 90/100 = 90%
}

TEST_CASE("CollisionMonitor: Efficiency violation detection", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    monitor.set_min_efficiency(0.1);  // Expect at least 10%
    
    int violation_count = 0;
    CollisionEfficiencyViolation last_violation;
    
    monitor.set_violation_callback([&]([[maybe_unused]] const CollisionEfficiencyViolation& v) {
        ++violation_count;
        last_violation = v;
    });
    
    // Good efficiency - no violation
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(15);  // 15% efficiency
    monitor.end_frame();
    
    REQUIRE(violation_count == 0);
    
    // Poor efficiency - violation
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(5);  // 5% efficiency (below 10% threshold)
    monitor.end_frame();
    
    REQUIRE(violation_count == 1);
    REQUIRE(last_violation.frame_number == 2);
    REQUIRE(last_violation.broadphase_candidates == 100);
    REQUIRE(last_violation.actual_collisions == 5);
    REQUIRE(last_violation.efficiency == 0.05);
    REQUIRE(last_violation.min_efficiency == 0.1);
}

TEST_CASE("CollisionMonitor: Frame boundary handling", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    
    // Frame 1
    monitor.set_broadphase_candidates(50);
    monitor.set_narrowphase_tests(50);
    monitor.set_actual_collisions(5);
    monitor.end_frame();
    
    CollisionStats stats1 = monitor.get_last_frame_stats();
    REQUIRE(stats1.frame_number == 1);
    REQUIRE(stats1.broadphase_candidates == 50);
    REQUIRE(stats1.actual_collisions == 5);
    
    // Frame 2 - different values
    monitor.set_broadphase_candidates(80);
    monitor.set_narrowphase_tests(80);
    monitor.set_actual_collisions(12);
    monitor.end_frame();
    
    CollisionStats stats2 = monitor.get_last_frame_stats();
    REQUIRE(stats2.frame_number == 2);
    REQUIRE(stats2.broadphase_candidates == 80);
    REQUIRE(stats2.actual_collisions == 12);
    REQUIRE(stats2.efficiency == 0.15);  // 12/80 = 15%
}

TEST_CASE("CollisionMonitor: Enable/disable monitoring", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    monitor.set_min_efficiency(0.5);  // Very high threshold
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&]([[maybe_unused]] const CollisionEfficiencyViolation& v) {
        ++violation_count;
    });
    
    REQUIRE(monitor.is_enabled());
    
    // Disable monitoring
    monitor.set_enabled(false);
    REQUIRE_FALSE(monitor.is_enabled());
    
    // Set poor efficiency - should not trigger violation
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(10);  // 10% efficiency (well below 50% threshold)
    monitor.end_frame();
    
    REQUIRE(violation_count == 0);
    
    // Re-enable monitoring
    monitor.set_enabled(true);
    
    // Now it should trigger
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(10);
    monitor.end_frame();
    
    REQUIRE(violation_count == 1);
}

TEST_CASE("CollisionMonitor: Reset functionality", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(10);
    monitor.end_frame();
    
    REQUIRE(monitor.get_frame_count() == 1);
    
    CollisionStats stats_before = monitor.get_last_frame_stats();
    REQUIRE(stats_before.broadphase_candidates == 100);
    
    // Reset
    monitor.reset();
    
    REQUIRE(monitor.get_frame_count() == 0);
    REQUIRE(monitor.get_current_broadphase_candidates() == 0);
    REQUIRE(monitor.get_current_narrowphase_tests() == 0);
    REQUIRE(monitor.get_current_actual_collisions() == 0);
    
    CollisionStats stats_after = monitor.get_last_frame_stats();
    REQUIRE(stats_after.broadphase_candidates == 0);
    REQUIRE(stats_after.efficiency == 0.0);
}

TEST_CASE("CollisionMonitor: Zero candidates edge case", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    monitor.set_min_efficiency(0.1);
    
    int violation_count = 0;
    monitor.set_violation_callback([&]([[maybe_unused]] const CollisionEfficiencyViolation& v) {
        ++violation_count;
    });
    
    // No candidates at all
    monitor.set_broadphase_candidates(0);
    monitor.set_actual_collisions(0);
    monitor.end_frame();
    
    // Should not crash or trigger violation (no division by zero)
    REQUIRE(violation_count == 0);
    
    CollisionStats stats = monitor.get_last_frame_stats();
    REQUIRE(stats.efficiency == 0.0);
    REQUIRE(stats.false_positive_rate == 0.0);
}

TEST_CASE("CollisionMonitor: Multiple violations", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    monitor.set_min_efficiency(0.2);  // Expect at least 20%
    
    int violation_count = 0;
    
    monitor.set_violation_callback([&]([[maybe_unused]] const CollisionEfficiencyViolation& v) {
        ++violation_count;
    });
    
    // Frame 1 - violation
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(10);  // 10% < 20%
    monitor.end_frame();
    
    // Frame 2 - violation again
    monitor.set_broadphase_candidates(200);
    monitor.set_actual_collisions(20);  // 10% < 20%
    monitor.end_frame();
    
    // Frame 3 - no violation
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(30);  // 30% >= 20%
    monitor.end_frame();
    
    REQUIRE(violation_count == 2);
}

TEST_CASE("CollisionMonitor: Tolerance configuration", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    
    REQUIRE(monitor.get_min_efficiency() == 0.05);  // Default 5%
    
    monitor.set_min_efficiency(0.15);
    REQUIRE(monitor.get_min_efficiency() == 0.15);
    
    int violation_count = 0;
    monitor.set_violation_callback([&]([[maybe_unused]] const CollisionEfficiencyViolation& v) {
        ++violation_count;
    });
    
    // 10% efficiency - below new 15% threshold
    monitor.set_broadphase_candidates(100);
    monitor.set_actual_collisions(10);
    monitor.end_frame();
    
    REQUIRE(violation_count == 1);
}

TEST_CASE("CollisionMonitor: Narrowphase tracking", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    
    // Track all three metrics
    monitor.set_broadphase_candidates(100);
    monitor.set_narrowphase_tests(80);  // Some candidates filtered before narrowphase
    monitor.set_actual_collisions(10);
    monitor.end_frame();
    
    CollisionStats stats = monitor.get_last_frame_stats();
    
    REQUIRE(stats.broadphase_candidates == 100);
    REQUIRE(stats.narrowphase_tests == 80);
    REQUIRE(stats.actual_collisions == 10);
    
    // Can verify narrowphase hit rate: 10/80 = 12.5%
    double narrowphase_hit_rate = static_cast<double>(stats.actual_collisions) / 
                                   static_cast<double>(stats.narrowphase_tests);
    REQUIRE(narrowphase_hit_rate == 0.125);
}

TEST_CASE("CollisionMonitor: Perfect efficiency", "[diagnostics][collision_monitor]") {
    CollisionMonitor monitor;
    monitor.set_min_efficiency(0.99);  // Expect near-perfect
    
    int violation_count = 0;
    monitor.set_violation_callback([&]([[maybe_unused]] const CollisionEfficiencyViolation& v) {
        ++violation_count;
    });
    
    // Perfect efficiency - all candidates actually collide
    monitor.set_broadphase_candidates(50);
    monitor.set_actual_collisions(50);
    monitor.end_frame();
    
    REQUIRE(violation_count == 0);  // No violation
    
    CollisionStats stats = monitor.get_last_frame_stats();
    REQUIRE(stats.efficiency == 1.0);  // 100% efficiency
    REQUIRE(stats.false_positive_rate == 0.0);  // 0% false positives
}

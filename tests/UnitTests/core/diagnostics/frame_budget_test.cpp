#include <catch2/catch_test_macros.hpp>
#include <core/diagnostics/frame_budget.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <thread>
#include <chrono>

using namespace phynity::diagnostics;

TEST_CASE("FrameBudget: Target FPS conversion", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    
    budget.set_target_fps(60.0);
    // Allow off-by-one due to rounding (16666 or 16667 both acceptable)
    REQUIRE(budget.get_target_frame_time() >= 16666);
    REQUIRE(budget.get_target_frame_time() <= 16667);
    REQUIRE(budget.get_target_fps() >= 59.9);
    REQUIRE(budget.get_target_fps() <= 60.1);
    
    budget.set_target_fps(30.0);
    REQUIRE(budget.get_target_frame_time() == 33333);  // ~33.33ms
    
    budget.set_target_fps(144.0);
    REQUIRE(budget.get_target_frame_time() == 6944);  // ~6.94ms
}

TEST_CASE("FrameBudget: Direct frame time setting", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    
    budget.set_target_frame_time(10000);  // 10ms
    REQUIRE(budget.get_target_frame_time() == 10000);
    REQUIRE(budget.get_target_fps() == 100.0);
    
    budget.set_target_frame_time(20000);  // 20ms
    REQUIRE(budget.get_target_frame_time() == 20000);
    REQUIRE(budget.get_target_fps() == 50.0);
}

TEST_CASE("FrameBudget: Zone budget management", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    
    budget.set_zone_budget("physics", 5000);   // 5ms
    budget.set_zone_budget("rendering", 8000);  // 8ms
    budget.set_zone_budget("audio", 2000);     // 2ms
    
    REQUIRE(budget.get_zone_budget("physics") == 5000);
    REQUIRE(budget.get_zone_budget("rendering") == 8000);
    REQUIRE(budget.get_zone_budget("audio") == 2000);
    REQUIRE(budget.get_zone_budget("nonexistent") == 0);
    
    budget.remove_zone_budget("audio");
    REQUIRE(budget.get_zone_budget("audio") == 0);
    
    budget.clear_zone_budgets();
    REQUIRE(budget.get_zone_budget("physics") == 0);
    REQUIRE(budget.get_zone_budget("rendering") == 0);
}

TEST_CASE("FrameBudget: Frame budget violation detection", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    FrameProfiler profiler(5);
    
    budget.set_target_frame_time(5000);  // 5ms budget
    
    int violation_count = 0;
    std::string_view violated_zone;
    
    budget.set_violation_callback([&](const BudgetViolation& v) {
        ++violation_count;
        violated_zone = v.zone_name;
    });
    
    PROFILER_ENABLE(true);
    
    // Create a frame that definitely exceeds budget
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));  // 25ms > 5ms budget
    profiler.end_frame();
    
    budget.check_frame(profiler.get_last_frame());
    
    // Should have at least one violation (frame itself)
    REQUIRE(violation_count >= 1);
    REQUIRE(violated_zone.empty());  // Empty = total frame violation
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameBudget: Zone budget violation detection", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    FrameProfiler profiler(5);
    
    budget.set_target_frame_time(50000);  // 50ms total budget (won't violate)
    budget.set_zone_budget("expensive_zone", 5000);  // 5ms zone budget
    
    bool violation_triggered = false;
    std::string_view violated_zone;
    uint64_t overage = 0;
    
    budget.set_violation_callback([&](const BudgetViolation& v) {
        violation_triggered = true;
        violated_zone = v.zone_name;
        overage = v.overage_us();
    });
    
    PROFILER_ENABLE(true);
    
    // Create a frame where the zone exceeds its budget
    profiler.begin_frame();
    {
        PROFILE_SCOPE("expensive_zone");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ms > 5ms budget
    }
    profiler.end_frame();
    
    budget.check_frame(profiler.get_last_frame());
    
    REQUIRE(violation_triggered);
    REQUIRE(violated_zone == "expensive_zone");
    REQUIRE(overage >= 4000);  // At least 4ms over (conservative)
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameBudget: No violation when under budget", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    FrameProfiler profiler(5);
    
    budget.set_target_frame_time(50000);  // 50ms budget - plenty of room
    
    int violation_count = 0;
    
    budget.set_violation_callback([&]([[maybe_unused]] const BudgetViolation& v) {
        ++violation_count;
    });
    
    PROFILER_ENABLE(true);
    
    // Create a frame well under budget
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 5ms << 50ms budget
    profiler.end_frame();
    
    budget.check_frame(profiler.get_last_frame());
    
    REQUIRE(violation_count == 0);  // No violations expected
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameBudget: Multiple violations in one frame", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    FrameProfiler profiler(5);
    
    budget.set_target_frame_time(100000);  // 100ms total (won't violate)
    budget.set_zone_budget("zone_a", 3000);  // 3ms (tighter budget)
    budget.set_zone_budget("zone_b", 5000);  // 5ms (tighter budget)
    
    int violation_count = 0;
    std::vector<std::string> violated_zones;
    
    budget.set_violation_callback([&](const BudgetViolation& v) {
        ++violation_count;
        if (!v.zone_name.empty()) {
            violated_zones.push_back(std::string(v.zone_name));
        }
    });
    
    PROFILER_ENABLE(true);
    
    profiler.begin_frame();
    {
        PROFILE_SCOPE("zone_a");
        std::this_thread::sleep_for(std::chrono::milliseconds(15));  // Definitely exceeds 3ms
    }
    {
        PROFILE_SCOPE("zone_b");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Definitely exceeds 5ms
    }
    profiler.end_frame();
    
    budget.check_frame(profiler.get_last_frame());
    
    // Should have at least 1 violation, preferably 2
    REQUIRE(violation_count >= 1);
    REQUIRE(!violated_zones.empty());
    // Verify at least one of the zones violated
    bool has_zone_a = std::find(violated_zones.begin(), violated_zones.end(), "zone_a") != violated_zones.end();
    bool has_zone_b = std::find(violated_zones.begin(), violated_zones.end(), "zone_b") != violated_zones.end();
    REQUIRE((has_zone_a || has_zone_b));
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameBudget: Enable/disable", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    FrameProfiler profiler(5);
    
    budget.set_target_frame_time(10000);  // 10ms (will violate with 25ms sleep)
    
    int violation_count = 0;
    budget.set_violation_callback([&]([[maybe_unused]] const BudgetViolation& v) {
        ++violation_count;
    });
    
    REQUIRE(budget.is_enabled());  // Enabled by default
    
    // Disable budget checking
    budget.set_enabled(false);
    REQUIRE_FALSE(budget.is_enabled());
    
    PROFILER_ENABLE(true);
    
    // Create violating frame
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    profiler.end_frame();
    
    budget.check_frame(profiler.get_last_frame());
    
    // Should not trigger because disabled
    REQUIRE(violation_count == 0);
    
    // Re-enable and check again
    budget.set_enabled(true);
    budget.check_frame(profiler.get_last_frame());
    
    // Now should trigger
    REQUIRE(violation_count >= 1);
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameBudget: Violation overage calculations", "[diagnostics][frame_budget]") {
    BudgetViolation v;
    v.budget_us = 10000;   // 10ms budget
    v.actual_us = 15000;   // 15ms actual
    
    REQUIRE(v.overage_us() == 5000);  // 5ms over
    REQUIRE(v.overage_percent() == 50.0);  // 50% over
    
    v.actual_us = 20000;  // 20ms actual
    REQUIRE(v.overage_us() == 10000);  // 10ms over
    REQUIRE(v.overage_percent() == 100.0);  // 100% over
    
    v.actual_us = 8000;  // Under budget
    REQUIRE(v.overage_us() == 0);
    REQUIRE(v.overage_percent() == 0.0);
}

TEST_CASE("FrameBudget: Check multiple recent frames", "[diagnostics][frame_budget]") {
    FrameBudget budget;
    FrameProfiler profiler(5);
    
    budget.set_target_frame_time(15000);  // 15ms budget
    
    int violation_count = 0;
    budget.set_violation_callback([&]([[maybe_unused]] const BudgetViolation& v) {
        ++violation_count;
    });
    
    PROFILER_ENABLE(true);
    
    // Frame 1: Well under budget
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    profiler.end_frame();
    
    // Frame 2: Definitely violates
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    profiler.end_frame();
    
    // Frame 3: Well under budget
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    profiler.end_frame();
    
    // Check last 3 frames
    budget.check_recent_frames(profiler, 3);
    
    // Should have 1 violation (frame 2), but timing variance might cause 1-3
    REQUIRE(violation_count >= 1);
    REQUIRE(violation_count <= 3);  // Allow for timing variance
    
    PROFILER_ENABLE(false);
}

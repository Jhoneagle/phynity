#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/common/timestep_controller.hpp>
#include <vector>

using phynity::physics::TimestepController;
using Catch::Matchers::WithinAbs;

// ============================================================================
// Basic Accumulation Tests
// ============================================================================

TEST_CASE("TimestepController: Default constructor", "[TimestepController][constructor]") {
    TimestepController controller;
    
    REQUIRE_THAT(controller.target_timestep(), WithinAbs(1.0f / 60.0f, 1e-6f));
    REQUIRE_THAT(controller.max_timestep(), WithinAbs(1.0f / 30.0f, 1e-6f));
    REQUIRE(controller.overflow_mode() == TimestepController::OverflowMode::CLAMP);
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("TimestepController: Custom constructor", "[TimestepController][constructor]") {
    TimestepController controller(0.01f, 0.05f, TimestepController::OverflowMode::SUBDIVIDE);
    
    REQUIRE_THAT(controller.target_timestep(), WithinAbs(0.01f, 1e-6f));
    REQUIRE_THAT(controller.max_timestep(), WithinAbs(0.05f, 1e-6f));
    REQUIRE(controller.overflow_mode() == TimestepController::OverflowMode::SUBDIVIDE);
}

TEST_CASE("TimestepController: Accumulate increases accumulated_time", "[TimestepController][accumulate]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.008f);
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.008f, 1e-6f));
    
    controller.accumulate(0.008f);
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.016f, 1e-6f));
}

TEST_CASE("TimestepController: Step returns zero before accumulated_time reaches target", 
          "[TimestepController][step]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.008f);
    REQUIRE_THAT(controller.step(), WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.008f, 1e-6f));
}

TEST_CASE("TimestepController: Step returns target_timestep when enough accumulated", 
          "[TimestepController][step]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.016f);
    float dt = controller.step();
    
    REQUIRE_THAT(dt, WithinAbs(0.016f, 1e-6f));
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.0f, 1e-6f));
    REQUIRE(controller.statistics().total_steps == 1);
}

TEST_CASE("TimestepController: Multiple steps with exact timing", "[TimestepController][step]") {
    TimestepController controller(0.016f);
    
    // Accumulate exactly 3 timesteps
    controller.accumulate(0.048f);
    
    float dt1 = controller.step();
    REQUIRE_THAT(dt1, WithinAbs(0.016f, 1e-6f));
    REQUIRE(controller.statistics().total_steps == 1);
    
    float dt2 = controller.step();
    REQUIRE_THAT(dt2, WithinAbs(0.016f, 1e-6f));
    REQUIRE(controller.statistics().total_steps == 2);
    
    float dt3 = controller.step();
    REQUIRE_THAT(dt3, WithinAbs(0.016f, 1e-6f));
    REQUIRE(controller.statistics().total_steps == 3);
    
    float dt4 = controller.step();
    REQUIRE_THAT(dt4, WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// CLAMP Overflow Mode Tests
// ============================================================================

TEST_CASE("TimestepController: CLAMP mode discards overflow", "[TimestepController][overflow][clamp]") {
    TimestepController controller(0.016f, 0.032f, TimestepController::OverflowMode::CLAMP);
    
    // Accumulate more than one timestep
    controller.accumulate(0.020f);
    
    float dt = controller.step();
    REQUIRE_THAT(dt, WithinAbs(0.016f, 1e-6f));
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.004f, 1e-6f));
    REQUIRE(controller.statistics().overflow_count == 0);  // No overflow yet
}

TEST_CASE("TimestepController: CLAMP mode detects overflow after step", "[TimestepController][overflow][clamp]") {
    TimestepController controller(0.016f, 0.032f, TimestepController::OverflowMode::CLAMP);
    
    // Accumulate more than 2 timesteps (causes overflow in second step)
    controller.accumulate(0.040f);
    
    float dt1 = controller.step();
    REQUIRE_THAT(dt1, WithinAbs(0.016f, 1e-6f));
    
    float dt2 = controller.step();
    REQUIRE_THAT(dt2, WithinAbs(0.016f, 1e-6f));
    
    // Overflow is now detected and clamped
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.0f, 1e-6f));
    REQUIRE(controller.statistics().overflow_count == 1);
}

// ============================================================================
// SUBDIVIDE Overflow Mode Tests
// ============================================================================

TEST_CASE("TimestepController: SUBDIVIDE mode clamps and tracks overflow", 
          "[TimestepController][overflow][subdivide]") {
    TimestepController controller(0.016f, 0.032f, TimestepController::OverflowMode::SUBDIVIDE);
    
    // Accumulate more than 2 timesteps
    controller.accumulate(0.040f);
    
    float dt1 = controller.step();
    REQUIRE_THAT(dt1, WithinAbs(0.016f, 1e-6f));
    
    float dt2 = controller.step();
    REQUIRE_THAT(dt2, WithinAbs(0.016f, 1e-6f));
    
    REQUIRE(controller.statistics().overflow_count == 1);
    REQUIRE(controller.statistics().subdivision_count == 1);
}

// ============================================================================
// UNCONSTRAINED Overflow Mode Tests
// ============================================================================

TEST_CASE("TimestepController: UNCONSTRAINED mode allows unlimited accumulation", 
          "[TimestepController][overflow][unconstrained]") {
    TimestepController controller(0.016f, 0.032f, TimestepController::OverflowMode::UNCONSTRAINED);
    
    controller.accumulate(0.100f);  // Large accumulation
    
    // Should be able to step multiple times
    int steps = 0;
    while (controller.step() > 0.0f) {
        steps++;
    }
    
    REQUIRE(steps == 6);  // 0.100 / 0.016 ≈ 6.25
}

// ============================================================================
// Statistics Tracking Tests
// ============================================================================

TEST_CASE("TimestepController: Statistics track total steps", "[TimestepController][statistics]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.048f);
    controller.step();
    controller.step();
    controller.step();
    
    REQUIRE(controller.statistics().total_steps == 3);
}

TEST_CASE("TimestepController: Statistics track max accumulated time", "[TimestepController][statistics]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.010f);
    controller.accumulate(0.015f);
    controller.accumulate(0.020f);  // total = 0.045f
    
    REQUIRE_THAT(controller.statistics().max_accumulated_time, WithinAbs(0.045f, 1e-6f));
}

TEST_CASE("TimestepController: Reset statistics", "[TimestepController][statistics]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.032f);
    controller.step();
    controller.step();
    
    REQUIRE(controller.statistics().total_steps == 2);
    
    controller.reset_statistics();
    
    REQUIRE(controller.statistics().total_steps == 0);
    REQUIRE(controller.statistics().overflow_count == 0);
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// Setter/Getter Tests
// ============================================================================

TEST_CASE("TimestepController: Set and get target timestep", "[TimestepController][setters]") {
    TimestepController controller;
    
    controller.set_target_timestep(0.01f);
    REQUIRE_THAT(controller.target_timestep(), WithinAbs(0.01f, 1e-6f));
}

TEST_CASE("TimestepController: Set and get max timestep", "[TimestepController][setters]") {
    TimestepController controller;
    
    controller.set_max_timestep(0.05f);
    REQUIRE_THAT(controller.max_timestep(), WithinAbs(0.05f, 1e-6f));
}

TEST_CASE("TimestepController: Set and get overflow mode", "[TimestepController][setters]") {
    TimestepController controller;
    
    controller.set_overflow_mode(TimestepController::OverflowMode::SUBDIVIDE);
    REQUIRE(controller.overflow_mode() == TimestepController::OverflowMode::SUBDIVIDE);
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_CASE("TimestepController: Reset accumulator clears accumulated time", "[TimestepController][reset]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.050f);
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.050f, 1e-6f));
    
    controller.reset_accumulator();
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.0f, 1e-6f));
}

// ============================================================================
// step_all Callback Tests
// ============================================================================

TEST_CASE("TimestepController: step_all executes callback for each step", "[TimestepController][step_all]") {
    TimestepController controller(0.016f);
    controller.accumulate(0.048f);
    
    std::vector<float> timesteps;
    int steps = controller.step_all([&timesteps](float dt) {
        timesteps.push_back(dt);
    });
    
    REQUIRE(steps == 3);
    REQUIRE(timesteps.size() == 3);
    REQUIRE_THAT(timesteps[0], WithinAbs(0.016f, 1e-6f));
    REQUIRE_THAT(timesteps[1], WithinAbs(0.016f, 1e-6f));
    REQUIRE_THAT(timesteps[2], WithinAbs(0.016f, 1e-6f));
}

TEST_CASE("TimestepController: step_all returns 0 when no steps", "[TimestepController][step_all]") {
    TimestepController controller(0.016f);
    controller.accumulate(0.008f);
    
    int steps = controller.step_all([](float) {});
    REQUIRE(steps == 0);
}

// ============================================================================
// Determinism Tests
// ============================================================================

TEST_CASE("TimestepController: Fixed timestep produces deterministic steps", 
          "[TimestepController][determinism]") {
    // Simulate two controllers with same inputs but different frame patterns
    TimestepController ctrl1(0.016f);
    TimestepController ctrl2(0.016f);
    
    // Controller 1: regular 16ms frames
    ctrl1.accumulate(0.048f);
    while (ctrl1.step() > 0.0f);
    
    // Controller 2: irregular frames that sum to same total
    ctrl2.accumulate(0.010f);
    ctrl2.step();
    ctrl2.accumulate(0.020f);
    ctrl2.step();
    ctrl2.accumulate(0.018f);
    while (ctrl2.step() > 0.0f);
    
    // Both controllers should have performed same total number of steps
    // regardless of how the accumulated time was split across frames
    REQUIRE(ctrl1.statistics().total_steps == ctrl2.statistics().total_steps);
    REQUIRE(ctrl1.statistics().total_steps == 3);  // 0.048 / 0.016 = 3
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_CASE("TimestepController: Zero accumulated time", "[TimestepController][edge-cases]") {
    TimestepController controller(0.016f);
    
    float dt = controller.step();
    REQUIRE_THAT(dt, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("TimestepController: Very small accumulation (below epsilon)", "[TimestepController][edge-cases]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(1e-9f);
    float dt = controller.step();
    REQUIRE_THAT(dt, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("TimestepController: Exactly one timestep", "[TimestepController][edge-cases]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.016f);
    float dt = controller.step();
    
    REQUIRE_THAT(dt, WithinAbs(0.016f, 1e-6f));
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("TimestepController: Just below one timestep", "[TimestepController][edge-cases]") {
    TimestepController controller(0.016f);
    
    controller.accumulate(0.0159f);
    float dt = controller.step();
    
    REQUIRE_THAT(dt, WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(controller.accumulated_time(), WithinAbs(0.0159f, 1e-6f));
}

TEST_CASE("TimestepController: Large number of small frames", "[TimestepController][edge-cases]") {
    TimestepController controller(0.016f);
    
    // Simulate 100 tiny frames that sum to 1 second
    for (int i = 0; i < 100; i++) {
        controller.accumulate(0.01f);
    }
    
    int steps = 0;
    while (controller.step() > 0.0f) {
        steps++;
    }
    
    // 100 * 0.01 = 1.0 second / (1/60) ≈ 60 steps
    REQUIRE(steps >= 60);
    REQUIRE(steps <= 62);  // Allow some rounding
}

// ============================================================================
// Real-world Scenario Tests
// ============================================================================

TEST_CASE("TimestepController: 60FPS with variable frame times", "[TimestepController][scenarios]") {
    TimestepController controller(1.0f / 60.0f);  // 60 FPS target
    
    // Simulate variable frame times
    std::vector<float> frame_times = {0.010f, 0.020f, 0.015f, 0.025f, 0.012f};
    
    int total_steps = 0;
    for (float frame_time : frame_times) {
        controller.accumulate(frame_time);
        while (controller.step() > 0.0f) {
            total_steps++;
        }
    }
    
    // 0.010 + 0.020 + 0.015 + 0.025 + 0.012 = 0.082 sec
    // 0.082 / (1/60) ≈ 4.92 steps, so expect 4 or 5
    REQUIRE(total_steps >= 4);
    REQUIRE(total_steps <= 6);
}

TEST_CASE("TimestepController: Frame rate spike handling (CLAMP mode)", "[TimestepController][scenarios]") {
    TimestepController controller(0.016f, 0.032f, TimestepController::OverflowMode::CLAMP);
    
    // Normal frame
    controller.accumulate(0.016f);
    controller.step();
    
    // Frame spike (single 100ms frame, maybe from stutter)
    controller.accumulate(0.100f);
    
    int steps = 0;
    while (controller.step() > 0.0f) steps++;
    
    // Should handle spike by clamping
    REQUIRE(controller.statistics().overflow_count >= 1);
}

TEST_CASE("TimestepController: Slow motion (very large frame time)", "[TimestepController][scenarios]") {
    TimestepController controller(0.016f);
    
    // Simulate slow-motion recording (single 1 second frame)
    controller.accumulate(1.0f);
    
    int steps = 0;
    while (controller.step() > 0.0f) {
        steps++;
    }
    
    // 1.0 / 0.016 = 62.5, so expect ~62 steps
    REQUIRE(steps == 62);
}

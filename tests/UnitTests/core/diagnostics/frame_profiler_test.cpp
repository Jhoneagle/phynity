#include <catch2/catch_test_macros.hpp>
#include <core/diagnostics/frame_profiler.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <thread>
#include <chrono>

using namespace phynity::diagnostics;

TEST_CASE("FrameProfiler: Basic frame tracking", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(10);  // Keep 10 frames
    
    PROFILER_ENABLE(true);
    
    // Simulate one frame
    profiler.begin_frame();
    {
        PROFILE_SCOPE("test_zone");
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    profiler.end_frame();
    
    REQUIRE(profiler.get_frame_count() == 1);
    
    const auto& last_frame = profiler.get_last_frame();
    REQUIRE(last_frame.frame_number == 0);
    REQUIRE(last_frame.total_frame_time_us >= 15000);  // At least 15ms
    REQUIRE(last_frame.zones.size() == 1);
    REQUIRE(last_frame.zones[0].name == "test_zone");
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Multiple frames", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(5);  // Keep 5 frames
    
    PROFILER_ENABLE(true);
    
    // Simulate 3 frames
    for (int i = 0; i < 3; ++i) {
        profiler.begin_frame();
        {
            PROFILE_SCOPE("frame_work");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        profiler.end_frame();
    }
    
    REQUIRE(profiler.get_frame_count() == 3);
    
    // Check we can access recent frames
    const auto& frame0 = profiler.get_frame(0);  // Most recent
    const auto& frame1 = profiler.get_frame(1);
    const auto& frame2 = profiler.get_frame(2);
    
    REQUIRE(frame0.frame_number == 2);  // Most recent = frame 2
    REQUIRE(frame1.frame_number == 1);
    REQUIRE(frame2.frame_number == 0);  // Oldest
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Ring buffer wraparound", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(3);  // Small buffer (3 frames)
    
    PROFILER_ENABLE(true);
    
    // Fill buffer and overflow it
    for (int i = 0; i < 5; ++i) {
        profiler.begin_frame();
        profiler.end_frame();
    }
    
    REQUIRE(profiler.get_frame_count() == 5);
    
    // Should only have access to last 3 frames
    const auto& frame0 = profiler.get_frame(0);
    const auto& frame1 = profiler.get_frame(1);
    const auto& frame2 = profiler.get_frame(2);
    
    REQUIRE(frame0.frame_number == 4);  // Most recent
    REQUIRE(frame1.frame_number == 3);
    REQUIRE(frame2.frame_number == 2);
    
    // Accessing beyond history should return empty frame
    const auto& frame3 = profiler.get_frame(3);
    REQUIRE(frame3.frame_number == 0);  // Empty/default frame
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Average frame time", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(10);
    
    PROFILER_ENABLE(true);
    
    // Create frames with known durations
    for (int i = 0; i < 3; ++i) {
        profiler.begin_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        profiler.end_frame();
    }
    
    const auto avg = profiler.get_average_frame_time(3);
    REQUIRE(avg >= 8000);  // At least 8ms average (conservative)
    REQUIRE(avg <= 30000); // Less than 30ms average (generous upper bound)
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Min/max frame time", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(10);
    
    PROFILER_ENABLE(true);
    
    // Frame 1: Short
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    profiler.end_frame();
    
    // Frame 2: Long
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    profiler.end_frame();
    
    // Frame 3: Medium
    profiler.begin_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(8));
    profiler.end_frame();
    
    const auto min_time = profiler.get_min_frame_time(3);
    const auto max_time = profiler.get_max_frame_time(3);
    
    REQUIRE(min_time < max_time);  // Min should be less than max
    REQUIRE(min_time >= 1000);     // At least 1ms (should be ~2ms)
    REQUIRE(max_time >= 12000);    // At least 12ms (should be ~15ms)
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Zone statistics", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(10);
    
    PROFILER_ENABLE(true);
    
    // Frame 1
    profiler.begin_frame();
    {
        PROFILE_SCOPE("update");
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    profiler.end_frame();
    
    // Frame 2
    profiler.begin_frame();
    {
        PROFILE_SCOPE("update");
        std::this_thread::sleep_for(std::chrono::milliseconds(7));
    }
    profiler.end_frame();
    
    // Frame 3
    profiler.begin_frame();
    {
        PROFILE_SCOPE("update");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    profiler.end_frame();
    
    const auto stats = profiler.get_zone_stats("update", 3);
    
    REQUIRE(stats.name == "update");
    REQUIRE(stats.call_count == 3);
    REQUIRE(stats.min_duration_us >= 2000);   // At least 2ms
    REQUIRE(stats.max_duration_us >= 6000);   // At least 6ms
    REQUIRE(stats.min_duration_us < stats.max_duration_us);
    REQUIRE(stats.average_duration_us() >= 2000);  // Average should be reasonable
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Zone statistics for non-existent zone", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(10);
    
    PROFILER_ENABLE(true);
    
    profiler.begin_frame();
    {
        PROFILE_SCOPE("existing_zone");
    }
    profiler.end_frame();
    
    const auto stats = profiler.get_zone_stats("non_existent_zone");
    
    REQUIRE(stats.name == "non_existent_zone");
    REQUIRE(stats.call_count == 0);
    REQUIRE(stats.min_duration_us == 0);
    REQUIRE(stats.max_duration_us == 0);
    REQUIRE(stats.average_duration_us() == 0.0);
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Multiple zones per frame", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(10);
    
    PROFILER_ENABLE(true);
    
    profiler.begin_frame();
    {
        PROFILE_SCOPE("physics");
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    {
        PROFILE_SCOPE("rendering");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    {
        PROFILE_SCOPE("audio");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    profiler.end_frame();
    
    const auto& frame = profiler.get_last_frame();
    REQUIRE(frame.zones.size() == 3);
    REQUIRE(frame.zones[0].name == "physics");
    REQUIRE(frame.zones[1].name == "rendering");
    REQUIRE(frame.zones[2].name == "audio");
    
    // Total frame time should be at least sum of zones
    const uint64_t zone_sum = frame.zones[0].duration_us + 
                              frame.zones[1].duration_us + 
                              frame.zones[2].duration_us;
    REQUIRE(frame.total_frame_time_us >= zone_sum);
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Clear history", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(5);
    
    PROFILER_ENABLE(true);
    
    // Add some frames
    for (int i = 0; i < 3; ++i) {
        profiler.begin_frame();
        profiler.end_frame();
    }
    
    REQUIRE(profiler.get_frame_count() == 3);
    
    // Clear history
    profiler.clear();
    
    REQUIRE(profiler.get_frame_count() == 0);
    const auto& frame = profiler.get_last_frame();
    REQUIRE(frame.frame_number == 0);
    REQUIRE(frame.total_frame_time_us == 0);
    
    PROFILER_ENABLE(false);
}

TEST_CASE("FrameProfiler: Nested zones preserved", "[diagnostics][frame_profiler]") {
    FrameProfiler profiler(5);
    
    PROFILER_ENABLE(true);
    
    profiler.begin_frame();
    {
        PROFILE_SCOPE("outer");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        {
            PROFILE_SCOPE("inner");
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    }
    profiler.end_frame();
    
    const auto& frame = profiler.get_last_frame();
    REQUIRE(frame.zones.size() == 2);
    
    // Check hierarchy is preserved
    REQUIRE(frame.zones[0].name == "outer");
    REQUIRE(frame.zones[0].depth == 0);
    REQUIRE(frame.zones[1].name == "inner");
    REQUIRE(frame.zones[1].depth == 1);
    REQUIRE(frame.zones[1].parent_index == 0);
    
    PROFILER_ENABLE(false);
}

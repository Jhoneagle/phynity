#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/diagnostics/timer.hpp>
#include <thread>
#include <chrono>

using namespace phynity::diagnostics;

TEST_CASE("Timer: Basic timing", "[diagnostics][timer]") {
    Timer timer;
    
    SECTION("Initial state") {
        REQUIRE(timer.elapsed_microseconds() == 0);
        REQUIRE_FALSE(timer.is_running());
    }
    
    SECTION("Start and stop") {
        timer.start();
        REQUIRE(timer.is_running());
        
        // Sleep for a known duration (approximate)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        timer.stop();
        REQUIRE_FALSE(timer.is_running());
        
        // Should have measured at least 30ms - much longer sleep to overcome scheduler variance
        REQUIRE(timer.elapsed_microseconds() >= 30000);
        REQUIRE(timer.elapsed_milliseconds() >= 30.0);
    }
    
    SECTION("Multiple start/stop cycles") {
        // First measurement
        timer.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        timer.stop();
        const auto first_duration = timer.elapsed_microseconds();
        REQUIRE(first_duration >= 15000);  // At least 15ms
        
        // Second measurement (restart)
        timer.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        timer.stop();
        const auto second_duration = timer.elapsed_microseconds();
        REQUIRE(second_duration >= 30000);  // At least 30ms
        
        // Second duration should be different from first
        REQUIRE(second_duration != first_duration);
    }
}

TEST_CASE("Timer: Elapsed time while running", "[diagnostics][timer]") {
    Timer timer;
    timer.start();
    
    // Sleep and check elapsed time without stopping
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto elapsed1 = timer.elapsed_microseconds();
    REQUIRE(elapsed1 >= 30000);  // At least 30ms
    
    // Continue running and check again
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto elapsed2 = timer.elapsed_microseconds();
    REQUIRE(elapsed2 >= 60000);  // At least 60ms total
    REQUIRE(elapsed2 > elapsed1);  // Time should have increased
}

TEST_CASE("Timer: Lap timing", "[diagnostics][timer]") {
    Timer timer;
    timer.start();
    
    // First lap
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    const auto lap1 = timer.lap();
    REQUIRE(lap1 >= 15000);  // At least 15ms
    
    // Second lap (cumulative)
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    const auto lap2 = timer.lap();
    REQUIRE(lap2 >= 30000);  // At least 30ms total
    REQUIRE(lap2 > lap1);   // Second lap should be later
    
    // Timer should still be running
    REQUIRE(timer.is_running());
}

TEST_CASE("Timer: Reset", "[diagnostics][timer]") {
    Timer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    timer.stop();
    
    REQUIRE(timer.elapsed_microseconds() > 0);
    
    // Reset should clear everything
    timer.reset();
    REQUIRE(timer.elapsed_microseconds() == 0);
    REQUIRE_FALSE(timer.is_running());
}

TEST_CASE("Timer: Time unit conversions", "[diagnostics][timer]") {
    Timer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    timer.stop();
    
    const auto us = timer.elapsed_microseconds();
    const auto ms = timer.elapsed_milliseconds();
    const auto s = timer.elapsed_seconds();
    
    // Check conversions are consistent
    REQUIRE_THAT(ms, Catch::Matchers::WithinRel(static_cast<double>(us) / 1000.0, 0.01));
    REQUIRE_THAT(s, Catch::Matchers::WithinRel(static_cast<double>(us) / 1000000.0, 0.01));
    
    // Should be approximately 100ms
    REQUIRE(ms >= 90.0);  // At least 90ms
    REQUIRE(ms <= 200.0); // At most 200ms (generous upper bound)
}

TEST_CASE("ScopedTimer: RAII timing", "[diagnostics][timer]") {
    uint64_t duration_us = 0;
    
    {
        ScopedTimer scoped_timer(duration_us);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }  // Timer stops here
    
    // Duration should be set after scope exit
    REQUIRE(duration_us >= 30000);  // At least 30ms
}

TEST_CASE("ScopedTimer: Multiple measurements", "[diagnostics][timer]") {
    uint64_t duration1 = 0;
    uint64_t duration2 = 0;
    
    // First measurement
    {
        ScopedTimer timer(duration1);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    
    // Second measurement (longer sleep time)
    {
        ScopedTimer timer(duration2);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Significantly longer
    }
    
    REQUIRE(duration1 >= 15000);   // At least 15ms
    REQUIRE(duration2 >= 30000);   // At least 30ms
    // Second should generally be longer (but timing variance means we can't guarantee strict ordering)
    // Just verify both measurements are reasonable
    REQUIRE(duration1 < 100000);   // Less than 100ms
    REQUIRE(duration2 < 100000);   // Less than 100ms
}

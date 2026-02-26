#include <catch2/catch_test_macros.hpp>
#include <core/diagnostics/profiler.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <core/diagnostics/timer.hpp>
#include <chrono>
#include <vector>

using namespace phynity::diagnostics;

// Baseline: do minimal work to compare against
void baseline_empty_function() {
    // Empty - measures function call overhead only
}

// Profiled version
void profiled_empty_function() {
    PROFILE_FUNCTION();
    // Empty - measures function call + profiling overhead
}

// Nested profiling test
void nested_profiling(int depth) {
    PROFILE_FUNCTION();
    if (depth > 0) {
        nested_profiling(depth - 1);
    }
}

// Baseline nested (no profiling)
void baseline_nested(int depth) {
    if (depth > 0) {
        baseline_nested(depth - 1);
    }
}

TEST_CASE("Profiler Overhead: Empty scope timing", "[validation][profiler][benchmark]") {
    const int iterations = 10000;
    
    // Warm up
    for (int i = 0; i < 100; ++i) {
        baseline_empty_function();
        profiled_empty_function();
    }
    
    // Measure baseline (no profiling)
    Profiler::enable(false);
    auto baseline_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        profiled_empty_function();  // Should be same as baseline when disabled
    }
    auto baseline_end = std::chrono::high_resolution_clock::now();
    const auto baseline_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(baseline_end - baseline_start);
    
    // Measure with profiling enabled
    Profiler::enable(true);
    auto profiled_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        profiled_empty_function();
    }
    auto profiled_end = std::chrono::high_resolution_clock::now();
    const auto profiled_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(profiled_end - profiled_start);
    
    // Calculate overhead per call
    const double baseline_ns_per_call = static_cast<double>(baseline_duration.count()) / iterations;
    const double profiled_ns_per_call = static_cast<double>(profiled_duration.count()) / iterations;
    const double overhead_ns = profiled_ns_per_call - baseline_ns_per_call;
    
    // Report results
    INFO("Baseline (disabled): " << baseline_ns_per_call << " ns/call");
    INFO("Profiled (enabled):  " << profiled_ns_per_call << " ns/call");
    INFO("Overhead:            " << overhead_ns << " ns/call");
    
    // Verify overhead is reasonable (less than 500ns per call)
    // Note: This is conservative for debug builds; optimized builds should be <50ns
    REQUIRE(overhead_ns < 500.0);
    REQUIRE(overhead_ns >= 0.0);  // Sanity check
}

TEST_CASE("Profiler Overhead: Disabled profiling cost", "[validation][profiler][benchmark]") {
    const int iterations = 100000;
    
    // Warm up
    for (int i = 0; i < 100; ++i) {
        baseline_empty_function();
    }
    
    // Measure baseline (truly empty)
    auto baseline_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        baseline_empty_function();
    }
    auto baseline_end = std::chrono::high_resolution_clock::now();
    const auto baseline_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(baseline_end - baseline_start);
    
    // Measure with profiling disabled (should add minimal overhead)
    Profiler::enable(false);
    auto disabled_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        profiled_empty_function();
    }
    auto disabled_end = std::chrono::high_resolution_clock::now();
    const auto disabled_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(disabled_end - disabled_start);
    
    // Calculate overhead
    const double baseline_ns_per_call = static_cast<double>(baseline_duration.count()) / iterations;
    const double disabled_ns_per_call = static_cast<double>(disabled_duration.count()) / iterations;
    const double overhead_ns = disabled_ns_per_call - baseline_ns_per_call;
    
    INFO("Baseline (no profiling code): " << baseline_ns_per_call << " ns/call");
    INFO("Disabled (profiling exists):  " << disabled_ns_per_call << " ns/call");
    INFO("Overhead:                     " << overhead_ns << " ns/call");
    
    // Verify disabled overhead is minimal (<50ns, ideally <5ns with branch prediction)
    REQUIRE(disabled_ns_per_call < baseline_ns_per_call + 100.0);  // Very conservative
}

TEST_CASE("Profiler Overhead: Nested scope scaling", "[validation][profiler][benchmark]") {
    const std::vector<int> depths = {1, 5, 10, 20};
    const int iterations_per_depth = 1000;
    
    // Verify overhead scales linearly with depth (not exponentially)
    std::vector<double> overhead_per_depth;
    
    for (int depth : depths) {
        // Warm up
        for (int i = 0; i < 10; ++i) {
            baseline_nested(depth);
            nested_profiling(depth);
        }
        
        // Measure baseline
        Profiler::enable(false);
        auto baseline_start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations_per_depth; ++i) {
            nested_profiling(depth);  // Should be same as baseline when disabled
        }
        auto baseline_end = std::chrono::high_resolution_clock::now();
        const auto baseline_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(baseline_end - baseline_start);
        
        // Measure with profiling
        Profiler::enable(true);
        Profiler::clear_frame();  // Clear previous data
        auto profiled_start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations_per_depth; ++i) {
            nested_profiling(depth);
            Profiler::clear_frame();  // Clear each iteration
        }
        auto profiled_end = std::chrono::high_resolution_clock::now();
        const auto profiled_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(profiled_end - profiled_start);
        
        const double overhead_ns = static_cast<double>(profiled_duration.count() - baseline_duration.count()) / iterations_per_depth;
        overhead_per_depth.push_back(overhead_ns);
        
        INFO("Depth " << depth << ": " << overhead_ns << " ns overhead");
    }
    
    // Verify scaling is roughly linear
    // Overhead should grow proportionally with depth
    // Check: overhead(depth=20) / overhead(depth=5) should be roughly 20/5 = 4
    if (overhead_per_depth.size() >= 4) {
        const double ratio_5_to_1 = overhead_per_depth[1] / overhead_per_depth[0];  // depth 5 / depth 1
        const double ratio_20_to_5 = overhead_per_depth[3] / overhead_per_depth[1];  // depth 20 / depth 5
        
        INFO("Ratio (depth 5 / depth 1): " << ratio_5_to_1 << " (expect ~5)");
        INFO("Ratio (depth 20 / depth 5): " << ratio_20_to_5 << " (expect ~4)");
        
        // Very conservative check: ratio shouldn't be exponential (e.g., 2^depth would be 16x minimum)
        // Linear scaling means ratio should be roughly proportional to depth ratio
        REQUIRE(ratio_20_to_5 < 10.0);  // Not exponential (would be 16+ for 2^depth)
        REQUIRE(ratio_20_to_5 > 1.5);    // Does scale with depth (not constant overhead)
    }
}

TEST_CASE("Profiler Overhead: Scope creation cost", "[validation][profiler][benchmark]") {
    const int iterations = 10000;
    
    // Enable profiling
    Profiler::enable(true);
    
    // Measure cost of creating many scopes
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        PROFILE_SCOPE("test_scope");
        // Minimal work
        [[maybe_unused]] volatile int x = i;
    }
    auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    const double ns_per_scope = static_cast<double>(duration.count()) / iterations;
    
    INFO("Scope creation + destruction: " << ns_per_scope << " ns");
    
    // Verify scope overhead is reasonable (conservative for debug builds)
    REQUIRE(ns_per_scope < 1000.0);  // Less than 1 microsecond per scope
}

TEST_CASE("Profiler Overhead: Multiple concurrent scopes", "[validation][profiler][benchmark]") {
    const int iterations = 1000;
    
    Profiler::enable(true);
    
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        PROFILE_SCOPE("outer");
        {
            PROFILE_SCOPE("inner1");
            [[maybe_unused]] volatile int x = i;
        }
        {
            PROFILE_SCOPE("inner2");
            [[maybe_unused]] volatile int y = i * 2;
        }
        {
            PROFILE_SCOPE("inner3");
            [[maybe_unused]] volatile int z = i * 3;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    const double ns_per_iteration = static_cast<double>(duration.count()) / iterations;
    
    INFO("4 scopes (1 outer + 3 inner): " << ns_per_iteration << " ns/iteration");
    
    // With 4 scopes, expect roughly 4x single scope overhead
    // Conservative check: less than 4 microseconds for 4 scopes
    REQUIRE(ns_per_iteration < 4000.0);
}

TEST_CASE("Profiler Overhead: Frame data collection", "[validation][profiler][benchmark]") {
    const int iterations = 1000;
    
    Profiler::enable(true);
    
    // Create some profiling data
    for (int i = 0; i < 10; ++i) {
        PROFILE_SCOPE("test_zone");
        [[maybe_unused]] volatile int x = i;
    }
    
    // Measure cost of retrieving frame data
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto data = Profiler::get_zones();
        [[maybe_unused]] volatile size_t size = data.size();
    }
    auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    const double ns_per_retrieval = static_cast<double>(duration.count()) / iterations;
    
    INFO("Frame data retrieval: " << ns_per_retrieval << " ns");
    
    // Data retrieval should be fast (it's just returning a reference)
    REQUIRE(ns_per_retrieval < 20000.0);
}

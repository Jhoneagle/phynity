#include <catch2/catch_test_macros.hpp>
#include <core/diagnostics/profiler.hpp>
#include <core/diagnostics/profiling_macros.hpp>
#include <thread>
#include <chrono>

using namespace phynity::diagnostics;

TEST_CASE("Profiler: Enable/disable", "[diagnostics][profiler]") {
    // Initially disabled
    REQUIRE_FALSE(Profiler::is_enabled());
    
    // Enable profiling
    Profiler::enable(true);
    REQUIRE(Profiler::is_enabled());
    
    // Disable profiling
    Profiler::enable(false);
    REQUIRE_FALSE(Profiler::is_enabled());
}

TEST_CASE("Profiler: Basic zone recording", "[diagnostics][profiler]") {
    Profiler::clear_frame();
    Profiler::enable(true);
    
    {
        ProfileScope scope("test_zone");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    const auto& zones = Profiler::get_zones();
    REQUIRE(zones.size() == 1);
    REQUIRE(zones[0].name == "test_zone");
    REQUIRE(zones[0].duration_us >= 4000);  // At least 4ms
    REQUIRE(zones[0].depth == 0);           // Root level
    
    Profiler::enable(false);
}

TEST_CASE("Profiler: Nested zones", "[diagnostics][profiler]") {
    Profiler::clear_frame();
    Profiler::enable(true);
    
    {
        ProfileScope outer("outer");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        
        {
            ProfileScope inner("inner");
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    const auto& zones = Profiler::get_zones();
    REQUIRE(zones.size() == 2);
    
    // Outer zone
    REQUIRE(zones[0].name == "outer");
    REQUIRE(zones[0].depth == 0);
    REQUIRE(zones[0].parent_index == static_cast<uint32_t>(-1));  // No parent
    REQUIRE(zones[0].duration_us >= 6000);  // At least 6ms total
    
    // Inner zone
    REQUIRE(zones[1].name == "inner");
    REQUIRE(zones[1].depth == 1);  // Nested inside outer
    REQUIRE(zones[1].parent_index == 0);  // Parent is zones[0]
    REQUIRE(zones[1].duration_us >= 2000);  // At least 2ms
    
    // Inner duration should be less than outer
    REQUIRE(zones[1].duration_us < zones[0].duration_us);
    
    Profiler::enable(false);
}

TEST_CASE("Profiler: Multiple sequential zones", "[diagnostics][profiler]") {
    Profiler::clear_frame();
    Profiler::enable(true);
    
    {
        ProfileScope zone1("zone1");
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    
    {
        ProfileScope zone2("zone2");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    {
        ProfileScope zone3("zone3");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    const auto& zones = Profiler::get_zones();
    REQUIRE(zones.size() == 3);
    REQUIRE(zones[0].name == "zone1");
    REQUIRE(zones[1].name == "zone2");
    REQUIRE(zones[2].name == "zone3");
    
    // All should be at root level
    REQUIRE(zones[0].depth == 0);
    REQUIRE(zones[1].depth == 0);
    REQUIRE(zones[2].depth == 0);
    
    Profiler::enable(false);
}

TEST_CASE("Profiler: Deep nesting", "[diagnostics][profiler]") {
    Profiler::clear_frame();
    Profiler::enable(true);
    
    {
        ProfileScope level0("level0");
        {
            ProfileScope level1("level1");
            {
                ProfileScope level2("level2");
                {
                    ProfileScope level3("level3");
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            }
        }
    }
    
    const auto& zones = Profiler::get_zones();
    REQUIRE(zones.size() == 4);
    
    // Check depth hierarchy
    REQUIRE(zones[0].depth == 0);
    REQUIRE(zones[1].depth == 1);
    REQUIRE(zones[2].depth == 2);
    REQUIRE(zones[3].depth == 3);
    
    // Check parent relationships
    REQUIRE(zones[0].parent_index == static_cast<uint32_t>(-1));
    REQUIRE(zones[1].parent_index == 0);
    REQUIRE(zones[2].parent_index == 1);
    REQUIRE(zones[3].parent_index == 2);
    
    Profiler::enable(false);
}

TEST_CASE("Profiler: Disabled profiling has no overhead", "[diagnostics][profiler]") {
    Profiler::clear_frame();
    Profiler::enable(false);  // Explicitly disabled
    
    {
        ProfileScope scope("disabled_zone");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    const auto& zones = Profiler::get_zones();
    REQUIRE(zones.empty());  // No zones recorded when disabled
}

TEST_CASE("Profiler: Clear frame resets zones", "[diagnostics][profiler]") {
    Profiler::clear_frame();
    Profiler::enable(true);
    
    // Add some zones
    {
        ProfileScope scope1("zone1");
    }
    {
        ProfileScope scope2("zone2");
    }
    
    REQUIRE(Profiler::get_zones().size() == 2);
    
    // Clear frame
    Profiler::clear_frame();
    REQUIRE(Profiler::get_zones().empty());
    
    Profiler::enable(false);
}

TEST_CASE("Profiling macros: PROFILE_SCOPE", "[diagnostics][profiler][macros]") {
    PROFILER_CLEAR_FRAME();
    PROFILER_ENABLE(true);
    
    {
        PROFILE_SCOPE("macro_test");
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    
    const auto& zones = PROFILER_GET_ZONES();
    REQUIRE(zones.size() == 1);
    REQUIRE(zones[0].name == "macro_test");
    REQUIRE(zones[0].duration_us >= 15000);
    
    PROFILER_ENABLE(false);
}

TEST_CASE("Profiling macros: PROFILE_FUNCTION", "[diagnostics][profiler][macros]") {
    auto test_function = []() {
        PROFILE_FUNCTION();
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    };
    
    PROFILER_CLEAR_FRAME();
    PROFILER_ENABLE(true);
    
    test_function();
    
    const auto& zones = PROFILER_GET_ZONES();
    REQUIRE(zones.size() == 1);
    // Note: Lambda function name is compiler-specific, but should be non-empty
    REQUIRE(!zones[0].name.empty());
    REQUIRE(zones[0].duration_us >= 1000);  // At least 1ms (timing can vary)
    
    PROFILER_ENABLE(false);
}

TEST_CASE("Profiling macros: Nested PROFILE_SCOPE", "[diagnostics][profiler][macros]") {
    PROFILER_CLEAR_FRAME();
    PROFILER_ENABLE(true);
    
    {
        PROFILE_SCOPE("outer_macro");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        
        {
            PROFILE_SCOPE("inner_macro");
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    }
    
    const auto& zones = PROFILER_GET_ZONES();
    REQUIRE(zones.size() == 2);
    REQUIRE(zones[0].name == "outer_macro");
    REQUIRE(zones[1].name == "inner_macro");
    REQUIRE(zones[0].depth == 0);
    REQUIRE(zones[1].depth == 1);
    
    PROFILER_ENABLE(false);
}

TEST_CASE("Profiler: Multiple zones at same level", "[diagnostics][profiler]") {
    PROFILER_CLEAR_FRAME();
    PROFILER_ENABLE(true);
    
    {
        PROFILE_SCOPE("outer");
        
        {
            PROFILE_SCOPE("child1");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        
        {
            PROFILE_SCOPE("child2");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    
    const auto& zones = PROFILER_GET_ZONES();
    REQUIRE(zones.size() == 3);
    
    // Outer zone
    REQUIRE(zones[0].name == "outer");
    REQUIRE(zones[0].depth == 0);
    
    // Two children at same depth
    REQUIRE(zones[1].name == "child1");
    REQUIRE(zones[1].depth == 1);
    REQUIRE(zones[1].parent_index == 0);
    
    REQUIRE(zones[2].name == "child2");
    REQUIRE(zones[2].depth == 1);
    REQUIRE(zones[2].parent_index == 0);  // Same parent as child1
    
    PROFILER_ENABLE(false);
}

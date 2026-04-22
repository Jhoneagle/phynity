#include <catch2/catch_test_macros.hpp>
#include <platform/thread_affinity.hpp>

using namespace phynity::platform;

TEST_CASE("physical_core_count returns at least 1", "[platform][affinity]")
{
    REQUIRE(physical_core_count() >= 1);
}

TEST_CASE("set_thread_affinity to core 0", "[platform][affinity]")
{
    // Core 0 should always exist. On macOS this sets an affinity tag (best-effort).
    bool result = set_thread_affinity(0);

    // We don't hard-require success because CI runners may restrict affinity,
    // but on most systems this should work.
    (void) result;

    // Just verify it doesn't crash
    SUCCEED();
}

TEST_CASE("set_thread_affinity to multiple cores", "[platform][affinity]")
{
    uint32_t cores = physical_core_count();

    // Try pinning to each core in sequence
    for (uint32_t i = 0; i < cores && i < 4; ++i)
    {
        set_thread_affinity(i);
    }

    SUCCEED();
}

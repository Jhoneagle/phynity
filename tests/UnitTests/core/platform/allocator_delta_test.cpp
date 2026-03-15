#include <catch2/catch_test_macros.hpp>
#include <platform/allocation_tracker.hpp>
#include <platform/memory_usage.hpp>

#include <algorithm>
#include <vector>

using namespace phynity::platform;

TEST_CASE("get_allocator_delta_bytes: is non-negative", "[platform][memory]")
{
    REQUIRE(get_allocator_delta_bytes() >= 0);
}

TEST_CASE("AllocatorDeltaScope: captures live allocation growth", "[platform][memory]")
{
    AllocatorDeltaScope scope;
    TrackedVector<int64_t> data(4096);
    std::fill(data.begin(), data.end(), 0LL);
    REQUIRE(!data.empty());

    REQUIRE(scope.delta_bytes() >= static_cast<int64_t>(data.size() * sizeof(int64_t)));
}

TEST_CASE("AllocatorDeltaScope: keeps benchmark delta after cleanup", "[platform][memory]")
{
    {
        AllocatorDeltaScope scope;
        constexpr int64_t alloc_size = 131072;
        TrackedVector<unsigned char> chunk(static_cast<size_t>(alloc_size), static_cast<unsigned char>(0xBE));
        REQUIRE(chunk.size() == static_cast<size_t>(alloc_size));

        REQUIRE(scope.delta_bytes() >= alloc_size);
    }

    REQUIRE(get_allocator_delta_bytes() >= 0);
}

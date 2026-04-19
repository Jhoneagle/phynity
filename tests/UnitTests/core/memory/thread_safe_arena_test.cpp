#include <catch2/catch_test_macros.hpp>
#include <core/memory/thread_safe_arena.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

namespace
{
constexpr std::size_t kArenaCapacity = 1024;
}

TEST_CASE("ThreadSafeArena allocates aligned blocks until full", "[memory]")
{
    phynity::memory::ThreadSafeArena arena(kArenaCapacity);

    constexpr std::size_t kAlign = alignof(std::max_align_t);

    void *first = arena.allocate(16, kAlign);
    REQUIRE(first != nullptr);
    REQUIRE(reinterpret_cast<std::uintptr_t>(first) % kAlign == 0);

    void *second = arena.allocate(32, kAlign);
    REQUIRE(second != nullptr);
    REQUIRE(reinterpret_cast<std::uintptr_t>(second) % kAlign == 0);

    REQUIRE(arena.size() >= 48);
    REQUIRE(arena.size() <= arena.capacity());
}

TEST_CASE("ThreadSafeArena returns nullptr when capacity is exhausted", "[memory]")
{
    phynity::memory::ThreadSafeArena arena(64);

    REQUIRE(arena.allocate(48, 16) != nullptr);
    REQUIRE(arena.allocate(32, 16) == nullptr);
    REQUIRE(arena.size() <= arena.capacity());
}

TEST_CASE("ThreadSafeArena reset rewinds allocation cursor", "[memory]")
{
    phynity::memory::ThreadSafeArena arena(128);

    void *first = arena.allocate(24, 8);
    REQUIRE(first != nullptr);
    REQUIRE(arena.size() > 0);

    arena.reset();
    REQUIRE(arena.size() == 0);

    void *after_reset = arena.allocate(24, 8);
    REQUIRE(after_reset != nullptr);
    REQUIRE(after_reset == first);
}

TEST_CASE("ThreadSafeArena serializes concurrent allocations", "[memory]")
{
    phynity::memory::ThreadSafeArena arena(kArenaCapacity);

    constexpr std::size_t kThreadCount = 8;
    constexpr std::size_t kAllocationSize = 32;
    std::array<void *, kThreadCount> allocations{};
    std::vector<std::thread> threads;
    threads.reserve(kThreadCount);

    for (std::size_t index = 0; index < kThreadCount; ++index)
    {
        threads.emplace_back([&arena, &allocations, index]()
                             { allocations[index] = arena.allocate(kAllocationSize, alignof(std::max_align_t)); });
    }

    for (auto &thread : threads)
    {
        thread.join();
    }

    REQUIRE(std::all_of(allocations.begin(), allocations.end(), [](void *ptr) { return ptr != nullptr; }));

    auto sorted = allocations;
    std::sort(sorted.begin(), sorted.end());
    REQUIRE(std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end());
    REQUIRE(arena.size() <= arena.capacity());
}

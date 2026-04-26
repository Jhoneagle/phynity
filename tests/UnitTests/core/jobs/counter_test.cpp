#include <catch2/catch_test_macros.hpp>
#include <core/jobs/counter.hpp>

#include <atomic>
#include <thread>
#include <vector>

using namespace phynity::jobs;

TEST_CASE("CounterPool acquire returns valid handle", "[jobs][counter]")
{
    CounterPool pool(8);
    auto handle = pool.acquire(5);

    REQUIRE(handle.valid());
    REQUIRE(pool.peek(handle) == 5);
}

TEST_CASE("CounterPool acquire exhaustion returns invalid", "[jobs][counter]")
{
    CounterPool pool(2);

    auto h1 = pool.acquire(1);
    auto h2 = pool.acquire(1);
    auto h3 = pool.acquire(1); // should fail

    REQUIRE(h1.valid());
    REQUIRE(h2.valid());
    REQUIRE_FALSE(h3.valid());
}

TEST_CASE("CounterPool decrement reaches zero", "[jobs][counter]")
{
    CounterPool pool(8);
    EventCount ec;

    auto handle = pool.acquire(3);

    REQUIRE_FALSE(pool.decrement(handle, ec)); // 3 -> 2
    REQUIRE(pool.peek(handle) == 2);
    REQUIRE_FALSE(pool.decrement(handle, ec)); // 2 -> 1
    REQUIRE(pool.peek(handle) == 1);
    REQUIRE(pool.decrement(handle, ec));       // 1 -> 0, returns true
    REQUIRE(pool.peek(handle) == 0);
}

TEST_CASE("CounterPool wait returns immediately when counter is zero", "[jobs][counter]")
{
    CounterPool pool(8);
    EventCount ec;

    auto handle = pool.acquire(1);
    pool.decrement(handle, ec); // 1 -> 0

    pool.wait(handle, ec); // should not block
}

TEST_CASE("CounterPool wait blocks until counter reaches zero", "[jobs][counter]")
{
    CounterPool pool(8);
    EventCount ec;

    auto handle = pool.acquire(3);
    std::atomic<bool> wait_done{false};

    std::thread waiter(
        [&]
        {
            pool.wait(handle, ec);
            wait_done.store(true, std::memory_order_release);
        });

    // Decrement from other threads
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    REQUIRE_FALSE(wait_done.load(std::memory_order_acquire));

    pool.decrement(handle, ec); // 3 -> 2
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    REQUIRE_FALSE(wait_done.load(std::memory_order_acquire));

    pool.decrement(handle, ec); // 2 -> 1
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    REQUIRE_FALSE(wait_done.load(std::memory_order_acquire));

    pool.decrement(handle, ec); // 1 -> 0 — triggers notify
    waiter.join();

    REQUIRE(wait_done.load(std::memory_order_acquire));
}

TEST_CASE("CounterPool concurrent decrements", "[jobs][counter]")
{
    CounterPool pool(8);
    EventCount ec;

    constexpr int count = 100;
    auto handle = pool.acquire(count);

    std::vector<std::thread> threads;
    std::atomic<int> zero_count{0};

    for (int i = 0; i < count; ++i)
    {
        threads.emplace_back(
            [&]
            {
                if (pool.decrement(handle, ec))
                {
                    zero_count.fetch_add(1, std::memory_order_relaxed);
                }
            });
    }

    for (auto &t : threads)
    {
        t.join();
    }

    // Exactly one thread should have seen the counter reach zero
    REQUIRE(zero_count.load() == 1);
    REQUIRE(pool.peek(handle) == 0);
}

TEST_CASE("CounterPool stale handle decrement is safe", "[jobs][counter]")
{
    CounterPool pool(8);
    EventCount ec;

    auto h1 = pool.acquire(1);
    pool.decrement(h1, ec); // 1 -> 0
    pool.wait(h1, ec);      // releases the slot

    auto h2 = pool.acquire(5); // reuses the slot with new generation

    // Decrementing the stale handle should be a no-op (generation mismatch)
    REQUIRE_FALSE(pool.decrement(h1, ec));
    REQUIRE(pool.peek(h2) == 5); // h2 unaffected
}

TEST_CASE("CounterPool decrement with invalid handle is safe", "[jobs][counter]")
{
    CounterPool pool(8);
    EventCount ec;

    CounterHandle invalid;
    REQUIRE_FALSE(pool.decrement(invalid, ec));
}

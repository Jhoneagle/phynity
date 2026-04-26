#include <catch2/catch_test_macros.hpp>
#include <core/jobs/eventcount.hpp>

#include <atomic>
#include <thread>
#include <vector>

using namespace phynity::jobs;

TEST_CASE("EventCount notify before wait does not block", "[jobs][eventcount]")
{
    EventCount ec;

    // Notify first
    ec.notify_one();

    // prepare_wait captures the post-notify epoch
    auto token = ec.prepare_wait();
    // Notify again so epoch advances past token
    ec.notify_one();

    // wait should return immediately because epoch advanced
    ec.wait(token);
}

TEST_CASE("EventCount cancel_wait after prepare_wait", "[jobs][eventcount]")
{
    EventCount ec;

    auto token = ec.prepare_wait();
    ec.cancel_wait(token);
    // Should not deadlock or crash — just decrements waiter count
}

TEST_CASE("EventCount single waiter woken by notify_one", "[jobs][eventcount]")
{
    EventCount ec;
    std::atomic<bool> started{false};
    std::atomic<bool> woken{false};

    std::thread waiter(
        [&]
        {
            auto token = ec.prepare_wait();
            started.store(true, std::memory_order_release);
            ec.wait(token);
            woken.store(true, std::memory_order_release);
        });

    // Wait for the waiter to be ready
    while (!started.load(std::memory_order_acquire))
    {
        std::this_thread::yield();
    }

    // Small delay to let waiter enter wait()
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    REQUIRE_FALSE(woken.load(std::memory_order_acquire));
    ec.notify_one();

    waiter.join();
    REQUIRE(woken.load(std::memory_order_acquire));
}

TEST_CASE("EventCount multiple waiters woken by notify_all", "[jobs][eventcount]")
{
    EventCount ec;
    constexpr int num_waiters = 4;
    std::atomic<int> ready_count{0};
    std::atomic<int> woken_count{0};

    std::vector<std::thread> waiters;
    for (int i = 0; i < num_waiters; ++i)
    {
        waiters.emplace_back(
            [&]
            {
                auto token = ec.prepare_wait();
                ready_count.fetch_add(1, std::memory_order_release);
                ec.wait(token);
                woken_count.fetch_add(1, std::memory_order_release);
            });
    }

    // Wait for all waiters to be ready
    while (ready_count.load(std::memory_order_acquire) < num_waiters)
    {
        std::this_thread::yield();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    REQUIRE(woken_count.load(std::memory_order_acquire) == 0);

    ec.notify_all();

    for (auto &t : waiters)
    {
        t.join();
    }

    REQUIRE(woken_count.load(std::memory_order_acquire) == num_waiters);
}

TEST_CASE("EventCount condition-met before wait skips blocking", "[jobs][eventcount]")
{
    EventCount ec;
    std::atomic<bool> condition{false};

    // Set condition before the waiter checks
    condition.store(true, std::memory_order_release);

    auto token = ec.prepare_wait();
    if (condition.load(std::memory_order_acquire))
    {
        ec.cancel_wait(token);
        // Should not block
        REQUIRE(true);
    }
    else
    {
        ec.wait(token);
        REQUIRE(false); // Should not reach here
    }
}

#include <catch2/catch_test_macros.hpp>
#include <core/jobs/work_stealing_deque.hpp>

#include <algorithm>
#include <atomic>
#include <set>
#include <thread>
#include <vector>

using namespace phynity::jobs;

TEST_CASE("WorkStealingDeque single-threaded push/pop LIFO", "[jobs][deque]")
{
    WorkStealingDeque<int> deque(4); // capacity 16

    REQUIRE(deque.push(1));
    REQUIRE(deque.push(2));
    REQUIRE(deque.push(3));

    int val = 0;
    REQUIRE(deque.pop(val));
    REQUIRE(val == 3); // LIFO
    REQUIRE(deque.pop(val));
    REQUIRE(val == 2);
    REQUIRE(deque.pop(val));
    REQUIRE(val == 1);
    REQUIRE_FALSE(deque.pop(val)); // empty
}

TEST_CASE("WorkStealingDeque single-threaded push/steal FIFO", "[jobs][deque]")
{
    WorkStealingDeque<int> deque(4);

    REQUIRE(deque.push(1));
    REQUIRE(deque.push(2));
    REQUIRE(deque.push(3));

    int val = 0;
    REQUIRE(deque.steal(val));
    REQUIRE(val == 1); // FIFO
    REQUIRE(deque.steal(val));
    REQUIRE(val == 2);
    REQUIRE(deque.steal(val));
    REQUIRE(val == 3);
    REQUIRE_FALSE(deque.steal(val)); // empty
}

TEST_CASE("WorkStealingDeque empty pop and steal", "[jobs][deque]")
{
    WorkStealingDeque<int> deque(4);

    int val = 0;
    REQUIRE_FALSE(deque.pop(val));
    REQUIRE_FALSE(deque.steal(val));
}

TEST_CASE("WorkStealingDeque full push returns false", "[jobs][deque]")
{
    WorkStealingDeque<int> deque(2); // capacity 4

    REQUIRE(deque.push(1));
    REQUIRE(deque.push(2));
    REQUIRE(deque.push(3));
    REQUIRE(deque.push(4));
    REQUIRE_FALSE(deque.push(5)); // full
}

TEST_CASE("WorkStealingDeque size_approx", "[jobs][deque]")
{
    WorkStealingDeque<int> deque(4);

    REQUIRE(deque.size_approx() == 0);

    deque.push(1);
    deque.push(2);
    REQUIRE(deque.size_approx() == 2);

    int val = 0;
    deque.pop(val);
    REQUIRE(deque.size_approx() == 1);
}

TEST_CASE("WorkStealingDeque multi-threaded producer + stealers", "[jobs][deque]")
{
    constexpr int item_count = 1000;
    constexpr std::size_t stealer_count = 3;

    WorkStealingDeque<int> deque(12); // capacity 4096

    std::atomic<int> total_consumed{0};
    std::vector<std::vector<int>> stolen_items(stealer_count);

    // Producer pushes all items, then pops what remains
    std::vector<int> owner_items;

    // Start stealers
    std::vector<std::thread> stealers;
    std::atomic<bool> done{false};

    for (std::size_t s = 0; s < stealer_count; ++s)
    {
        stealers.emplace_back(
            [&deque, &done, &stolen_items, s]
            {
                while (!done.load(std::memory_order_acquire))
                {
                    int val = 0;
                    if (deque.steal(val))
                    {
                        stolen_items[s].push_back(val);
                    }
                }
                // Drain remaining
                int val = 0;
                while (deque.steal(val))
                {
                    stolen_items[s].push_back(val);
                }
            });
    }

    // Producer
    for (int i = 0; i < item_count; ++i)
    {
        while (!deque.push(i))
        {
            // If full, pop from our end
            int val = 0;
            if (deque.pop(val))
            {
                owner_items.push_back(val);
            }
        }
    }

    // Pop remaining from owner side
    {
        int val = 0;
        while (deque.pop(val))
        {
            owner_items.push_back(val);
        }
    }

    done.store(true, std::memory_order_release);

    for (auto &t : stealers)
    {
        t.join();
    }

    // Collect all consumed items
    std::set<int> all_items(owner_items.begin(), owner_items.end());
    for (const auto &sv : stolen_items)
    {
        for (int v : sv)
        {
            all_items.insert(v);
        }
    }

    // Every item consumed exactly once
    REQUIRE(all_items.size() == item_count);
    size_t total_size = owner_items.size();
    for (const auto &sv : stolen_items)
    {
        total_size += sv.size();
    }
    REQUIRE(total_size == item_count);
}

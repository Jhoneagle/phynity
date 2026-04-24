#include <catch2/catch_test_macros.hpp>
#include <core/serialization/simulation_timeline.hpp>

using phynity::serialization::PhysicsSnapshot;
using phynity::serialization::SimulationTimeline;

namespace
{

PhysicsSnapshot make_snapshot(uint64_t frame, double time)
{
    PhysicsSnapshot s;
    s.frame_number = frame;
    s.simulated_time = time;
    return s;
}

} // namespace

TEST_CASE("SimulationTimeline starts empty", "[serialization][timeline]")
{
    SimulationTimeline timeline(10);
    REQUIRE(timeline.size() == 0);
    REQUIRE(timeline.capacity() == 10);
    REQUIRE(timeline.at(0) == nullptr);
}

TEST_CASE("SimulationTimeline push and access", "[serialization][timeline]")
{
    SimulationTimeline timeline(5);

    timeline.push(make_snapshot(0, 0.0));
    timeline.push(make_snapshot(1, 0.016));
    timeline.push(make_snapshot(2, 0.032));

    REQUIRE(timeline.size() == 3);
    REQUIRE(timeline.at(0)->frame_number == 0);
    REQUIRE(timeline.at(1)->frame_number == 1);
    REQUIRE(timeline.at(2)->frame_number == 2);
    REQUIRE(timeline.at(3) == nullptr);
}

TEST_CASE("SimulationTimeline ring buffer wraps around", "[serialization][timeline]")
{
    SimulationTimeline timeline(3);

    timeline.push(make_snapshot(0, 0.0));
    timeline.push(make_snapshot(1, 0.016));
    timeline.push(make_snapshot(2, 0.032));
    // Buffer is full, next push overwrites oldest
    timeline.push(make_snapshot(3, 0.048));

    REQUIRE(timeline.size() == 3);
    // Oldest is now frame 1
    REQUIRE(timeline.at(0)->frame_number == 1);
    REQUIRE(timeline.at(1)->frame_number == 2);
    REQUIRE(timeline.at(2)->frame_number == 3);

    // Push again
    timeline.push(make_snapshot(4, 0.064));
    REQUIRE(timeline.at(0)->frame_number == 2);
    REQUIRE(timeline.at(2)->frame_number == 4);
}

TEST_CASE("SimulationTimeline clear", "[serialization][timeline]")
{
    SimulationTimeline timeline(5);
    timeline.push(make_snapshot(0, 0.0));
    timeline.push(make_snapshot(1, 0.016));

    timeline.clear();
    REQUIRE(timeline.size() == 0);
    REQUIRE(timeline.at(0) == nullptr);

    // Can push after clear
    timeline.push(make_snapshot(10, 1.0));
    REQUIRE(timeline.size() == 1);
    REQUIRE(timeline.at(0)->frame_number == 10);
}

TEST_CASE("SimulationTimeline truncate", "[serialization][timeline]")
{
    SimulationTimeline timeline(10);

    for (uint64_t i = 0; i < 5; ++i)
    {
        timeline.push(make_snapshot(i, static_cast<double>(i) * 0.016));
    }

    REQUIRE(timeline.size() == 5);

    // Truncate to 3 (keep frames 0, 1, 2)
    timeline.truncate(3);
    REQUIRE(timeline.size() == 3);
    REQUIRE(timeline.at(0)->frame_number == 0);
    REQUIRE(timeline.at(2)->frame_number == 2);
    REQUIRE(timeline.at(3) == nullptr);

    // Can push new frames after truncate
    timeline.push(make_snapshot(100, 10.0));
    REQUIRE(timeline.size() == 4);
    REQUIRE(timeline.at(3)->frame_number == 100);
}

TEST_CASE("SimulationTimeline truncate is no-op when new_size >= size", "[serialization][timeline]")
{
    SimulationTimeline timeline(5);
    timeline.push(make_snapshot(0, 0.0));
    timeline.push(make_snapshot(1, 0.016));

    timeline.truncate(5);
    REQUIRE(timeline.size() == 2);

    timeline.truncate(2);
    REQUIRE(timeline.size() == 2);
}

TEST_CASE("SimulationTimeline single capacity", "[serialization][timeline]")
{
    SimulationTimeline timeline(1);

    timeline.push(make_snapshot(0, 0.0));
    REQUIRE(timeline.size() == 1);
    REQUIRE(timeline.at(0)->frame_number == 0);

    timeline.push(make_snapshot(1, 0.016));
    REQUIRE(timeline.size() == 1);
    REQUIRE(timeline.at(0)->frame_number == 1);
}

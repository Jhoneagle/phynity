#include <catch2/catch_test_macros.hpp>
#include <core/jobs/job_id.hpp>

#include <unordered_set>

using namespace phynity::jobs;

TEST_CASE("JobId default is invalid", "[jobs][job_id]")
{
    JobId id;
    REQUIRE_FALSE(id.valid());
    REQUIRE(id.index == JobId::invalid_index);
}

TEST_CASE("JobId with valid index", "[jobs][job_id]")
{
    JobId id{42, 1};
    REQUIRE(id.valid());
    REQUIRE(id.index == 42);
    REQUIRE(id.generation == 1);
}

TEST_CASE("JobId equality", "[jobs][job_id]")
{
    JobId a{1, 2};
    JobId b{1, 2};
    JobId c{1, 3};
    JobId d{2, 2};

    REQUIRE(a == b);
    REQUIRE_FALSE(a == c);
    REQUIRE_FALSE(a == d);
}

TEST_CASE("JobId ordering by index", "[jobs][job_id]")
{
    JobId a{1, 0};
    JobId b{2, 0};

    REQUIRE(a < b);
    REQUIRE_FALSE(b < a);
}

TEST_CASE("JobId hashable", "[jobs][job_id]")
{
    std::unordered_set<JobId> set;
    set.insert(JobId{0, 1});
    set.insert(JobId{1, 1});
    set.insert(JobId{0, 1}); // duplicate

    REQUIRE(set.size() == 2);
}

TEST_CASE("CounterHandle default is invalid", "[jobs][counter_handle]")
{
    CounterHandle h;
    REQUIRE_FALSE(h.valid());
}

TEST_CASE("CounterHandle with valid index", "[jobs][counter_handle]")
{
    CounterHandle h{5, 1};
    REQUIRE(h.valid());
}

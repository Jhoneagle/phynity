#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "field_line_tracer.hpp"

using Catch::Matchers::WithinAbs;
using phynity::app::viz::FieldLine;
using phynity::app::viz::trace_field_line;
using phynity::app::viz::trace_field_lines;
using phynity::math::vectors::Vec3f;

TEST_CASE("FieldLineTracer: Uniform field traces a straight line", "[field_line_tracer]")
{
    // Uniform field along +z; field lines are straight and evenly spaced.
    auto sampler = [](const Vec3f &) { return Vec3f(0.0f, 0.0f, 3.0f); };

    FieldLine line = trace_field_line(sampler, Vec3f(0.0f), 10, 0.1f);

    REQUIRE(line.points.size() == 11); // steps + 1
    for (size_t i = 0; i < line.points.size(); ++i)
    {
        REQUIRE_THAT(line.points[i].x, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(line.points[i].y, WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(line.points[i].z, WithinAbs(static_cast<float>(i) * 0.1f, 1e-5f));
    }
}

TEST_CASE("FieldLineTracer: Step length is independent of field magnitude", "[field_line_tracer]")
{
    // Doubling |B| must not change the geometry — the direction is normalized.
    auto weak = [](const Vec3f &) { return Vec3f(1.0f, 0.0f, 0.0f); };
    auto strong = [](const Vec3f &) { return Vec3f(100.0f, 0.0f, 0.0f); };

    FieldLine a = trace_field_line(weak, Vec3f(0.0f), 5, 0.2f);
    FieldLine b = trace_field_line(strong, Vec3f(0.0f), 5, 0.2f);

    REQUIRE(a.points.size() == b.points.size());
    for (size_t i = 0; i < a.points.size(); ++i)
    {
        REQUIRE_THAT(a.points[i].x, WithinAbs(b.points[i].x, 1e-6f));
    }
    // Last point advanced 5 * 0.2 = 1.0 along +x.
    REQUIRE_THAT(a.points.back().x, WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("FieldLineTracer: Parallel seeds preserve their separation", "[field_line_tracer]")
{
    auto sampler = [](const Vec3f &) { return Vec3f(0.0f, 0.0f, 1.0f); };

    std::vector<Vec3f> seeds = {Vec3f(0.0f, 0.0f, 0.0f), Vec3f(1.0f, 0.0f, 0.0f)};
    std::vector<FieldLine> lines = trace_field_lines(sampler, seeds, 4, 0.25f);

    REQUIRE(lines.size() == 2);
    REQUIRE(lines[0].points.size() == lines[1].points.size());
    for (size_t i = 0; i < lines[0].points.size(); ++i)
    {
        const float separation = (lines[1].points[i] - lines[0].points[i]).length();
        REQUIRE_THAT(separation, WithinAbs(1.0f, 1e-5f));
    }
}

TEST_CASE("FieldLineTracer: A vanishing field ends the line", "[field_line_tracer]")
{
    // Field is non-zero only near the origin; the line stops once it leaves.
    auto sampler = [](const Vec3f &p)
    { return p.z < 0.25f ? Vec3f(0.0f, 0.0f, 1.0f) : Vec3f(0.0f, 0.0f, 0.0f); };

    FieldLine line = trace_field_line(sampler, Vec3f(0.0f), 100, 0.1f);

    // Starts at z=0, advances while z < 0.25, then the sampler returns zero.
    REQUIRE(line.points.size() >= 2);
    REQUIRE(line.points.size() < 100);
    REQUIRE(line.points.back().z >= 0.25f);
}

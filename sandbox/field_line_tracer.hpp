#pragma once

#include <core/math/vectors/vec3.hpp>

#include <functional>
#include <vector>

namespace phynity::app::viz
{

using phynity::math::vectors::Vec3f;

/// A traced magnetic field line: an ordered polyline of sample points.
struct FieldLine
{
    std::vector<Vec3f> points;
};

/// Samples a vector field B at a world position. Returning a zero-length vector
/// signals "no field here" and terminates the streamline.
using FieldSampler = std::function<Vec3f(const Vec3f &)>;

/// Trace a single magnetic field line by integrating along the normalized field
/// direction (a simple forward-Euler streamline). Visualization only — this does
/// not participate in the physics solve.
///
/// @param sampler Field sampler B(x).
/// @param seed Starting point of the line.
/// @param steps Number of segments to advance (points = steps + 1 at most).
/// @param step_length World-space length of each segment (> 0).
/// @return The traced polyline; stops early where the field vanishes.
inline FieldLine trace_field_line(const FieldSampler &sampler, const Vec3f &seed, int steps, float step_length)
{
    FieldLine line;
    if (steps < 0 || step_length <= 0.0f)
    {
        return line;
    }

    line.points.reserve(static_cast<size_t>(steps) + 1);
    Vec3f position = seed;
    line.points.push_back(position);

    for (int i = 0; i < steps; ++i)
    {
        const Vec3f b = sampler(position);
        // Field too weak to define a direction: end the line here.
        if (b.squaredLength() < 1e-12f)
        {
            break;
        }
        position += b.normalized() * step_length;
        line.points.push_back(position);
    }

    return line;
}

/// Trace field lines from several seed points using the same parameters.
inline std::vector<FieldLine>
trace_field_lines(const FieldSampler &sampler, const std::vector<Vec3f> &seeds, int steps, float step_length)
{
    std::vector<FieldLine> lines;
    lines.reserve(seeds.size());
    for (const Vec3f &seed : seeds)
    {
        lines.push_back(trace_field_line(sampler, seed, steps, step_length));
    }
    return lines;
}

} // namespace phynity::app::viz

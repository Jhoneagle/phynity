#include <core/physics/collision/ccd/swept_sphere.hpp>

#include <algorithm>
#include <cmath>

namespace phynity::physics::collision::ccd
{

TimeOfImpactResult SweptSphereSolver::solve() const
{
    return solve_static(sphere_a_, sphere_b_, dt_);
}

TimeOfImpactResult SweptSphereSolver::solve_static(const Sphere &sphere_a, const Sphere &sphere_b, float dt)
{
    auto finite_vec = [](const Vec3f &v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); };

    if (!std::isfinite(dt) || dt <= 0.0f)
    {
        return TimeOfImpactResult();
    }

    if (!std::isfinite(sphere_a.radius) || !std::isfinite(sphere_b.radius) || sphere_a.radius <= 0.0f ||
        sphere_b.radius <= 0.0f)
    {
        return TimeOfImpactResult();
    }

    if (!finite_vec(sphere_a.position) || !finite_vec(sphere_b.position) || !finite_vec(sphere_a.velocity) ||
        !finite_vec(sphere_b.velocity))
    {
        return TimeOfImpactResult();
    }

    float safe_dt = (dt > 1e-10f) ? dt : 1.0f;

    // Compute relative position and velocity
    Vec3f relative_pos = sphere_a.position - sphere_b.position;
    // Use normalized time t in [0,1], so scale motion by dt
    Vec3f relative_vel = (sphere_a.velocity - sphere_b.velocity) * safe_dt;

    float min_distance = sphere_a.radius + sphere_b.radius;

    // Early exit: already overlapping at start
    float current_distance = relative_pos.length();
    if (current_distance < min_distance)
    {
        // Collision at t=0 (already overlapping)
        Vec3f normal = (current_distance > 1e-6f) ? relative_pos.normalized() : Vec3f(1.0f, 0.0f, 0.0f);

        Vec3f contact_pt = sphere_b.position + normal * sphere_b.radius;
        Vec3f rel_vel = compute_relative_velocity(sphere_a, sphere_b);

        return TimeOfImpactResult(0.0f, contact_pt, normal, rel_vel);
    }

    // Check if spheres are approaching
    if (!LinearCast::are_spheres_approaching(
            sphere_a.position, sphere_a.velocity, sphere_b.position, sphere_b.velocity))
    {
        return TimeOfImpactResult(); // No collision
    }

    // Set up quadratic equation: |p + v*t|² = d²
    // where p = relative_pos, v = relative_vel, d = min_distance
    // Expands to: a*t² + b*t + c = 0

    float a = relative_vel.dot(relative_vel);
    float b = 2.0f * relative_pos.dot(relative_vel);
    float c = relative_pos.dot(relative_pos) - min_distance * min_distance;

    // Solve quadratic (normalized to [0, 1] timestep)
    float toi = solve_quadratic(a, b, c, 1.0f);

    if (toi < 0.0f)
    {
        return TimeOfImpactResult(); // No collision within timestep
    }

    // Compute contact point and normal at collision time
    float physical_time = toi * safe_dt;
    Vec3f contact_point = compute_contact_point(physical_time, sphere_a, sphere_b);
    Vec3f contact_normal = compute_contact_normal(physical_time, sphere_a, sphere_b);
    Vec3f rel_velocity = compute_relative_velocity(sphere_a, sphere_b);

    return TimeOfImpactResult(toi, contact_point, contact_normal, rel_velocity);
}

float SweptSphereSolver::solve_quadratic(float a, float b, float c, float max_t)
{
    // Handle degenerate case (relative_vel ≈ 0, parallel motion)
    if (std::abs(a) < 1e-10f)
    {
        // Linear equation: b*t + c = 0
        if (std::abs(b) < 1e-10f)
        {
            return -1.0f; // No collision (both coefficients near zero)
        }
        float t = -c / b;
        return (t >= 0.0f && t <= max_t) ? t : -1.0f;
    }

    // Standard quadratic formula
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
    {
        return -1.0f; // No real roots (no collision)
    }

    float sqrt_disc = std::sqrt(discriminant);
    float t1 = (-b - sqrt_disc) / (2.0f * a);
    float t2 = (-b + sqrt_disc) / (2.0f * a);

    // We want the smallest non-negative root within [0, max_t]
    // t1 is typically the "entering" time, t2 is the "exiting" time
    if (t1 >= 0.0f && t1 <= max_t)
    {
        return t1;
    }
    if (t2 >= 0.0f && t2 <= max_t)
    {
        return t2;
    }

    return -1.0f; // No valid collision time
}

Vec3f SweptSphereSolver::compute_contact_point(float time, const Sphere &sphere_a, const Sphere &sphere_b)
{
    // Position of each sphere at collision time
    Vec3f pos_a = sphere_a.position + sphere_a.velocity * time;
    Vec3f pos_b = sphere_b.position + sphere_b.velocity * time;

    // Contact point is on the surface, linearly interpolated
    // along the line connecting centers
    Vec3f direction = (pos_b - pos_a);
    float distance = direction.length();

    if (distance > 1e-6f)
    {
        Vec3f normal = direction / distance;
        // Point on surface of sphere A
        return pos_a + normal * sphere_a.radius;
    }

    // Degenerate case: centers overlap, return A's center
    return pos_a;
}

Vec3f SweptSphereSolver::compute_contact_normal(float time, const Sphere &sphere_a, const Sphere &sphere_b)
{
    // Position of each sphere at collision time
    Vec3f pos_a = sphere_a.position + sphere_a.velocity * time;
    Vec3f pos_b = sphere_b.position + sphere_b.velocity * time;

    // Normal points from A to B
    Vec3f direction = pos_b - pos_a;
    float distance = direction.length();

    if (distance > 1e-6f)
    {
        return direction / distance; // Normalized
    }

    // Degenerate case: return arbitrary normal
    return Vec3f(1.0f, 0.0f, 0.0f);
}

Vec3f SweptSphereSolver::compute_relative_velocity(const Sphere &sphere_a, const Sphere &sphere_b)
{
    return sphere_b.velocity - sphere_a.velocity;
}

} // namespace phynity::physics::collision::ccd

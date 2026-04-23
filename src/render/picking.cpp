#include "picking.hpp"

#include <cmath>
#include <limits>

namespace phynity::render
{

namespace
{

/// Ray-sphere intersection test.
/// Returns distance along ray, or negative if no hit.
float ray_sphere_intersect(const Vec3f &ray_origin, const Vec3f &ray_dir, const Vec3f &center, float radius)
{
    Vec3f oc(ray_origin.x - center.x, ray_origin.y - center.y, ray_origin.z - center.z);
    float b = oc.x * ray_dir.x + oc.y * ray_dir.y + oc.z * ray_dir.z;
    float c = oc.x * oc.x + oc.y * oc.y + oc.z * oc.z - radius * radius;
    float discriminant = b * b - c;

    if (discriminant < 0.0f)
    {
        return -1.0f;
    }

    float sqrt_disc = std::sqrt(discriminant);
    float t0 = -b - sqrt_disc;
    float t1 = -b + sqrt_disc;

    if (t0 > 0.0f)
        return t0;
    if (t1 > 0.0f)
        return t1;
    return -1.0f;
}

float bounding_radius_for_body(const SceneRenderer::BodyVisual &body)
{
    switch (body.type)
    {
        case SceneRenderer::ShapeType::Sphere:
            return body.dimensions.x;
        case SceneRenderer::ShapeType::Box:
        {
            // Use half-diagonal of the box
            float dx = body.dimensions.x;
            float dy = body.dimensions.y;
            float dz = body.dimensions.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        case SceneRenderer::ShapeType::Capsule:
            return body.dimensions.x + body.dimensions.y; // radius + half_height
        case SceneRenderer::ShapeType::Unknown:
            return 0.5f;
    }
    return 0.5f;
}

} // namespace

std::optional<PickResult> pick_body(const Camera &camera,
                                     float screen_x,
                                     float screen_y,
                                     int viewport_width,
                                     int viewport_height,
                                     const SceneRenderer::State &state)
{
    Vec3f ray_origin = camera.get_position();
    Vec3f ray_dir = camera.screen_to_ray(screen_x, screen_y, viewport_width, viewport_height);

    float best_dist = std::numeric_limits<float>::max();
    int best_id = -1;

    for (const auto &body : state.bodies)
    {
        float radius = bounding_radius_for_body(body);
        float t = ray_sphere_intersect(ray_origin, ray_dir, body.position, radius);
        if (t > 0.0f && t < best_dist)
        {
            best_dist = t;
            best_id = body.id;
        }
    }

    if (best_id >= 0)
    {
        return PickResult{best_id, best_dist};
    }
    return std::nullopt;
}

} // namespace phynity::render

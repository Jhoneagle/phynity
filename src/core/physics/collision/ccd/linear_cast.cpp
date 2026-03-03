#include <core/physics/collision/ccd/linear_cast.hpp>
#include <cmath>
#include <algorithm>

namespace phynity::physics::collision::ccd {

RaySpherIntersection LinearCast::raycast_sphere(
    const Vec3f& ray_origin,
    const Vec3f& ray_direction,
    float ray_max_t,
    const Vec3f& sphere_center,
    float sphere_radius
) {
    RaySpherIntersection result;
    
    // Vector from ray origin to sphere center
    Vec3f oc = ray_origin - sphere_center;
    
    // Ray equation: P(t) = ray_origin + t * ray_direction
    // Sphere equation: |P(t) - sphere_center|² = radius²
    // Substituting: |oc + t * ray_direction|² = radius²
    
    // Expand to quadratic: a*t² + b*t + c = 0
    float a = ray_direction.dot(ray_direction);
    float b = 2.0f * oc.dot(ray_direction);
    float c = oc.dot(oc) - sphere_radius * sphere_radius;
    
    float discriminant = b * b - 4.0f * a * c;
    
    if (discriminant < 0.0f) {
        return result;  // No intersection
    }
    
    float sqrt_disc = std::sqrt(discriminant);
    float t1 = (-b - sqrt_disc) / (2.0f * a);
    float t2 = (-b + sqrt_disc) / (2.0f * a);
    
    // Clamp to ray bounds [0, ray_max_t]
    if (t1 < 0.0f && t2 < 0.0f) {
        return result;  // Both intersections behind ray origin
    }
    
    // Use the first intersection (near intersection) if valid
    float t_near = std::max(0.0f, t1);
    if (t_near > ray_max_t) {
        return result;  // Beyond ray endpoint
    }
    
    float t_far = std::max(0.0f, t2);
    t_far = (t_far > ray_max_t) ? ray_max_t : t_far;
    
    result.hit = true;
    result.t_near = t_near;
    result.t_far = t_far;
    result.contact_point = ray_origin + ray_direction * t_near;
    
    // Normal at near intersection (pointing outward from sphere)
    Vec3f to_contact = result.contact_point - sphere_center;
    float dist = to_contact.length();
    result.contact_normal = (dist > 1e-6f) ? (to_contact / dist) : Vec3f(1.0f, 0.0f, 0.0f);
    
    return result;
}

RaySpherIntersection LinearCast::sweep_sphere_vs_sphere_static(
    const Vec3f& moving_sphere_center,
    const Vec3f& moving_sphere_velocity,
    float moving_sphere_radius,
    const Vec3f& static_sphere_center,
    float static_sphere_radius,
    float max_time
) {
    RaySpherIntersection result;
    
    // Treat as ray cast: the moving sphere's center traces a ray
    // Check intersection with enlarged static sphere (sum of radii)
    float combined_radius = moving_sphere_radius + static_sphere_radius;
    
    // Normalize velocity to unit direction and get speed
    float speed = moving_sphere_velocity.length();
    
    if (speed < 1e-10f) {
        // No movement - check if already colliding
        Vec3f diff = moving_sphere_center - static_sphere_center;
        float distance = diff.length();
        
        if (distance < combined_radius) {
            result.hit = true;
            result.t_near = 0.0f;
            result.t_far = 0.0f;
            result.contact_point = static_sphere_center + 
                (distance > 1e-6f ? diff.normalized() * static_sphere_radius : Vec3f(1.0f, 0.0f, 0.0f));
            result.contact_normal = distance > 1e-6f ? -diff.normalized() : Vec3f(1.0f, 0.0f, 0.0f);
        }
        return result;
    }
    
    Vec3f ray_direction = moving_sphere_velocity / speed;
    float ray_max_t = speed * max_time;
    
    // Raycast against enlarged sphere
    return raycast_sphere(
        moving_sphere_center,
        ray_direction,
        ray_max_t,
        static_sphere_center,
        combined_radius
    );
}

RaySpherIntersection LinearCast::sweep_sphere_vs_aabb(
    const Vec3f& sphere_center,
    const Vec3f& sphere_velocity,
    float sphere_radius,
    const Vec3f& aabb_min,
    const Vec3f& aabb_max,
    float max_time
) {
    RaySpherIntersection result;
    
    // Conservative approach: sweep sphere against AABB by
    // expanding AABB by sphere radius and raycasting center point
    Vec3f expanded_min = aabb_min - Vec3f(sphere_radius);
    Vec3f expanded_max = aabb_max + Vec3f(sphere_radius);
    
    // Simple ray-AABB intersection
    // For each axis, compute intersection with slab [expanded_min, expanded_max]
    
    float speed = sphere_velocity.length();
    if (speed < 1e-10f) {
        // Check if sphere overlaps AABB
        bool overlaps = (sphere_center.x >= expanded_min.x && sphere_center.x <= expanded_max.x) &&
                       (sphere_center.y >= expanded_min.y && sphere_center.y <= expanded_max.y) &&
                       (sphere_center.z >= expanded_min.z && sphere_center.z <= expanded_max.z);
        
        if (overlaps) {
            result.hit = true;
            result.t_near = 0.0f;
            result.t_far = 0.0f;
            // Compute contact point (closest point on AABB to sphere center)
            Vec3f closest = sphere_center;
            closest.x = std::max(expanded_min.x, std::min(sphere_center.x, expanded_max.x));
            closest.y = std::max(expanded_min.y, std::min(sphere_center.y, expanded_max.y));
            closest.z = std::max(expanded_min.z, std::min(sphere_center.z, expanded_max.z));
            result.contact_point = closest;
            
            Vec3f normal = sphere_center - closest;
            float dist = normal.length();
            result.contact_normal = (dist > 1e-6f) ? normal / dist : Vec3f(1.0f, 0.0f, 0.0f);
        }
        return result;
    }
    
    Vec3f ray_direction = sphere_velocity / speed;
    float ray_length = speed * max_time;
    
    // Ray-AABB slab intersection (optimized DDA-like algorithm)
    float t_enter = 0.0f;
    float t_exit = ray_length;
    
    for (int axis = 0; axis < 3; ++axis) {
        float ray_start = sphere_center[axis];
        float ray_dir = ray_direction[axis];
        
        if (std::abs(ray_dir) < 1e-10f) {
            // Ray parallel to slab
            if (ray_start < expanded_min[axis] || ray_start > expanded_max[axis]) {
                return result;  // No intersection
            }
        } else {
            float t1 = (expanded_min[axis] - ray_start) / ray_dir;
            float t2 = (expanded_max[axis] - ray_start) / ray_dir;
            
            if (t1 > t2) std::swap(t1, t2);
            
            t_enter = std::max(t_enter, t1);
            t_exit = std::min(t_exit, t2);
            
            if (t_enter > t_exit) {
                return result;  // No intersection
            }
        }
    }
    
    // Intersection found
    if (t_enter >= ray_length) {
        return result;  // Beyond max_time
    }
    
    result.hit = true;
    result.t_near = std::max(0.0f, t_enter) / ray_length;  // Normalize to [0, 1]
    result.t_far = std::min(ray_length, t_exit) / ray_length;
    result.contact_point = sphere_center + ray_direction * t_enter;
    
    // Compute normal (perpendicular to closest AABB face)
    Vec3f closest = result.contact_point;
    closest.x = std::max(aabb_min.x, std::min(closest.x, aabb_max.x));
    closest.y = std::max(aabb_min.y, std::min(closest.y, aabb_max.y));
    closest.z = std::max(aabb_min.z, std::min(closest.z, aabb_max.z));
    
    Vec3f normal = result.contact_point - closest;
    float dist = normal.length();
    result.contact_normal = (dist > 1e-6f) ? normal / dist : Vec3f(1.0f, 0.0f, 0.0f);
    
    return result;
}

float LinearCast::compute_closest_approach_time(
    const Vec3f& pos1,
    const Vec3f& vel1,
    const Vec3f& pos2,
    const Vec3f& vel2
) {
    // Relative position and velocity
    Vec3f rel_pos = pos1 - pos2;
    Vec3f rel_vel = vel1 - vel2;
    
    // Time of closest approach: d/dt(|rel_pos + rel_vel*t|²) = 0
    // This gives: t = -(rel_pos · rel_vel) / (rel_vel · rel_vel)
    
    float vel_sq = rel_vel.dot(rel_vel);
    if (vel_sq < 1e-10f) {
        return 0.0f;  // Parallel motion or stationary
    }
    
    float t = -(rel_pos.dot(rel_vel)) / vel_sq;
    return clamp_time(t, 1.0f);
}

bool LinearCast::are_spheres_approaching(
    const Vec3f& pos1,
    const Vec3f& vel1,
    const Vec3f& pos2,
    const Vec3f& vel2
) {
    // Check if relative velocity points toward contact
    Vec3f rel_pos = pos2 - pos1;  // Direction from 1 to 2
    Vec3f rel_vel = vel2 - vel1;   // Relative velocity
    
    // If dot product is negative, they're approaching
    float approaching = rel_pos.dot(rel_vel);
    return approaching < -1e-6f;
}

}  // namespace phynity::physics::collision::ccd

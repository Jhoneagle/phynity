#pragma once

#include "camera.hpp"

#include <core/math/quaternions/quat.hpp>
#include <core/math/vectors/vec3.hpp>

#include <vector>

namespace phynity::render
{

using phynity::math::quaternions::Quatf;
using phynity::math::vectors::Vec3f;

/// Debug wireframe scene renderer using OpenGL lines.
class SceneRenderer
{
public:
    enum class ShapeType
    {
        Sphere,
        Box,
        Capsule,
        Unknown
    };

    struct BodyVisual
    {
        Vec3f position;
        Quatf orientation;
        ShapeType type = ShapeType::Unknown;
        Vec3f dimensions; // radius for sphere, half_extents for box, (radius, half_height, 0) for capsule
        bool selected = false;
        int id = -1;
    };

    struct ParticleVisual
    {
        Vec3f position;
        float radius = 0.2f;
        bool selected = false;
    };

    struct ConstraintVisual
    {
        Vec3f pos_a;
        Vec3f pos_b;
        int type = 0;
    };

    struct State
    {
        std::vector<BodyVisual> bodies;
        std::vector<ParticleVisual> particles;
        std::vector<ConstraintVisual> constraints;
    };

    /// Draw the scene using the provided camera and state
    void draw(const Camera &camera, const State &state, int viewport_width, int viewport_height);

private:
    void draw_ground_grid();
    void draw_body(const BodyVisual &body);
    void draw_particle(const ParticleVisual &particle);
    void draw_constraint(const ConstraintVisual &constraint);

    void draw_wireframe_box(const Vec3f &pos, const Quatf &orient, const Vec3f &half_extents);
    void draw_wireframe_sphere(const Vec3f &pos, float radius, int segments = 16);
    void draw_wireframe_capsule(const Vec3f &pos, const Quatf &orient, float radius, float half_height);

    void set_color(float r, float g, float b, float a = 1.0f);
};

} // namespace phynity::render

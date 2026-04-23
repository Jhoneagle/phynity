#include "scene_renderer.hpp"

#include <core/math/quaternions/quat_conversions.hpp>

#include <GLFW/glfw3.h>

#include <cmath>

namespace phynity::render
{

void SceneRenderer::draw(const Camera &camera, const State &state, int viewport_width, int viewport_height)
{
    auto view = camera.view_matrix();
    auto proj = camera.projection_matrix(viewport_width, viewport_height);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(proj.data());
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view.data());

    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);

    draw_ground_grid();

    for (const auto &body : state.bodies)
    {
        draw_body(body);
    }

    for (const auto &particle : state.particles)
    {
        draw_particle(particle);
    }

    for (const auto &constraint : state.constraints)
    {
        draw_constraint(constraint);
    }

    glDisable(GL_DEPTH_TEST);
}

void SceneRenderer::draw_ground_grid()
{
    set_color(0.3f, 0.3f, 0.3f);
    glBegin(GL_LINES);

    const float extent = 20.0f;
    const float step = 2.0f;
    for (float i = -extent; i <= extent; i += step)
    {
        glVertex3f(i, 0.0f, -extent);
        glVertex3f(i, 0.0f, extent);
        glVertex3f(-extent, 0.0f, i);
        glVertex3f(extent, 0.0f, i);
    }

    glEnd();
}

void SceneRenderer::draw_body(const BodyVisual &body)
{
    if (body.selected)
    {
        set_color(1.0f, 1.0f, 0.0f);
        glLineWidth(2.0f);
    }
    else
    {
        set_color(0.2f, 0.7f, 1.0f);
        glLineWidth(1.0f);
    }

    switch (body.type)
    {
        case ShapeType::Box:
            draw_wireframe_box(body.position, body.orientation, body.dimensions);
            break;
        case ShapeType::Sphere:
            draw_wireframe_sphere(body.position, body.dimensions.x);
            break;
        case ShapeType::Capsule:
            draw_wireframe_capsule(body.position, body.orientation, body.dimensions.x, body.dimensions.y);
            break;
        case ShapeType::Unknown:
            draw_wireframe_sphere(body.position, 0.3f);
            break;
    }

    glLineWidth(1.0f);
}

void SceneRenderer::draw_particle(const ParticleVisual &particle)
{
    if (particle.selected)
    {
        set_color(1.0f, 1.0f, 0.0f);
    }
    else
    {
        set_color(0.0f, 1.0f, 0.5f);
    }

    // Draw as small cross
    const float s = particle.radius;
    glBegin(GL_LINES);
    glVertex3f(particle.position.x - s, particle.position.y, particle.position.z);
    glVertex3f(particle.position.x + s, particle.position.y, particle.position.z);
    glVertex3f(particle.position.x, particle.position.y - s, particle.position.z);
    glVertex3f(particle.position.x, particle.position.y + s, particle.position.z);
    glVertex3f(particle.position.x, particle.position.y, particle.position.z - s);
    glVertex3f(particle.position.x, particle.position.y, particle.position.z + s);
    glEnd();
}

void SceneRenderer::draw_constraint(const ConstraintVisual &constraint)
{
    set_color(1.0f, 0.5f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(constraint.pos_a.x, constraint.pos_a.y, constraint.pos_a.z);
    glVertex3f(constraint.pos_b.x, constraint.pos_b.y, constraint.pos_b.z);
    glEnd();
}

void SceneRenderer::draw_wireframe_box(const Vec3f &pos, const Quatf &orient, const Vec3f &half_extents)
{
    auto rot = phynity::math::quaternions::toRotationMatrix(orient);

    // 8 corners of the box in local space
    Vec3f corners[8];
    int idx = 0;
    for (int sx = -1; sx <= 1; sx += 2)
    {
        for (int sy = -1; sy <= 1; sy += 2)
        {
            for (int sz = -1; sz <= 1; sz += 2)
            {
                Vec3f local(half_extents.x * static_cast<float>(sx),
                            half_extents.y * static_cast<float>(sy),
                            half_extents.z * static_cast<float>(sz));
                // Rotate and translate
                corners[idx] = pos + Vec3f(rot(0, 0) * local.x + rot(0, 1) * local.y + rot(0, 2) * local.z,
                                           rot(1, 0) * local.x + rot(1, 1) * local.y + rot(1, 2) * local.z,
                                           rot(2, 0) * local.x + rot(2, 1) * local.y + rot(2, 2) * local.z);
                ++idx;
            }
        }
    }

    // 12 edges
    static const int edges[][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7}, // z-edges
        {0, 2}, {1, 3}, {4, 6}, {5, 7}, // y-edges
        {0, 4}, {1, 5}, {2, 6}, {3, 7}, // x-edges
    };

    glBegin(GL_LINES);
    for (const auto &edge : edges)
    {
        glVertex3f(corners[edge[0]].x, corners[edge[0]].y, corners[edge[0]].z);
        glVertex3f(corners[edge[1]].x, corners[edge[1]].y, corners[edge[1]].z);
    }
    glEnd();
}

void SceneRenderer::draw_wireframe_sphere(const Vec3f &pos, float radius, int segments)
{
    const float pi = 3.14159265f;

    // Draw 3 rings (XY, XZ, YZ planes)
    for (int ring = 0; ring < 3; ++ring)
    {
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < segments; ++i)
        {
            float angle = 2.0f * pi * static_cast<float>(i) / static_cast<float>(segments);
            float ca = std::cos(angle) * radius;
            float sa = std::sin(angle) * radius;

            switch (ring)
            {
                case 0:
                    glVertex3f(pos.x + ca, pos.y + sa, pos.z);
                    break;
                case 1:
                    glVertex3f(pos.x + ca, pos.y, pos.z + sa);
                    break;
                case 2:
                    glVertex3f(pos.x, pos.y + ca, pos.z + sa);
                    break;
            }
        }
        glEnd();
    }
}

void SceneRenderer::draw_wireframe_capsule(const Vec3f &pos, const Quatf &orient, float radius, float half_height)
{
    // Simplified: draw as cylinder between two spheres
    auto rot = phynity::math::quaternions::toRotationMatrix(orient);

    // Capsule axis is local Y
    Vec3f axis(rot(0, 1), rot(1, 1), rot(2, 1));
    Vec3f top = pos + axis * half_height;
    Vec3f bottom = pos - axis * half_height;

    draw_wireframe_sphere(top, radius, 12);
    draw_wireframe_sphere(bottom, radius, 12);

    // 4 lines connecting top and bottom caps
    const float pi = 3.14159265f;
    Vec3f right(rot(0, 0), rot(1, 0), rot(2, 0));
    Vec3f fwd(rot(0, 2), rot(1, 2), rot(2, 2));

    glBegin(GL_LINES);
    for (int i = 0; i < 4; ++i)
    {
        float angle = pi * 0.5f * static_cast<float>(i);
        Vec3f offset = right * (std::cos(angle) * radius) + fwd * (std::sin(angle) * radius);
        Vec3f t = top + offset;
        Vec3f b = bottom + offset;
        glVertex3f(t.x, t.y, t.z);
        glVertex3f(b.x, b.y, b.z);
    }
    glEnd();
}

void SceneRenderer::set_color(float r, float g, float b, float a)
{
    glColor4f(r, g, b, a);
}

} // namespace phynity::render

#pragma once

#include <core/math/vectors/vec3.hpp>

#include <array>

namespace phynity::render
{

using phynity::math::vectors::Vec3f;

/// Simple orbit camera for debug visualization.
/// Left-drag rotates, right-drag pans, scroll zooms.
class Camera
{
public:
    Camera();

    /// Get 4x4 view matrix (column-major, OpenGL convention)
    std::array<float, 16> view_matrix() const;

    /// Get 4x4 perspective projection matrix
    std::array<float, 16> projection_matrix(int viewport_width, int viewport_height) const;

    /// Handle mouse input for orbit/pan/zoom
    void on_mouse_button(int button, bool pressed, float mouse_x, float mouse_y);
    void on_mouse_move(float mouse_x, float mouse_y);
    void on_scroll(float y_offset);

    /// Reset to default position
    void reset();

    /// Get camera world position (for picking)
    Vec3f get_position() const;

    /// Unproject screen coordinates to world-space ray direction
    Vec3f screen_to_ray(float screen_x, float screen_y, int viewport_width, int viewport_height) const;

private:
    Vec3f target_ = Vec3f(0.0f, 2.0f, 0.0f);
    float distance_ = 15.0f;
    float yaw_ = 0.3f;   // radians
    float pitch_ = 0.4f;  // radians
    float fov_ = 60.0f;   // degrees
    float near_ = 0.1f;
    float far_ = 500.0f;

    bool left_dragging_ = false;
    bool right_dragging_ = false;
    float last_mouse_x_ = 0.0f;
    float last_mouse_y_ = 0.0f;
};

} // namespace phynity::render

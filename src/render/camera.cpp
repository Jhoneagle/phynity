#include "camera.hpp"

#include <cmath>

namespace phynity::render
{

Camera::Camera() = default;

void Camera::reset()
{
    target_ = Vec3f(0.0f, 2.0f, 0.0f);
    distance_ = 15.0f;
    yaw_ = 0.3f;
    pitch_ = 0.4f;
}

Vec3f Camera::get_position() const
{
    float cos_pitch = std::cos(pitch_);
    float x = target_.x + distance_ * cos_pitch * std::sin(yaw_);
    float y = target_.y + distance_ * std::sin(pitch_);
    float z = target_.z + distance_ * cos_pitch * std::cos(yaw_);
    return Vec3f(x, y, z);
}

std::array<float, 16> Camera::view_matrix() const
{
    Vec3f eye = get_position();
    Vec3f forward = (target_ - eye);
    float len = std::sqrt(forward.x * forward.x + forward.y * forward.y + forward.z * forward.z);
    if (len > 0.0f)
    {
        forward.x /= len;
        forward.y /= len;
        forward.z /= len;
    }

    Vec3f up(0.0f, 1.0f, 0.0f);

    // right = forward x up
    Vec3f right(
        forward.y * up.z - forward.z * up.y, forward.z * up.x - forward.x * up.z, forward.x * up.y - forward.y * up.x);
    float rlen = std::sqrt(right.x * right.x + right.y * right.y + right.z * right.z);
    if (rlen > 0.0f)
    {
        right.x /= rlen;
        right.y /= rlen;
        right.z /= rlen;
    }

    // recalculate up = right x forward
    Vec3f real_up(right.y * forward.z - right.z * forward.y,
                  right.z * forward.x - right.x * forward.z,
                  right.x * forward.y - right.y * forward.x);

    // Column-major lookAt matrix
    std::array<float, 16> m = {};
    m[0] = right.x;
    m[1] = real_up.x;
    m[2] = -forward.x;
    m[3] = 0.0f;
    m[4] = right.y;
    m[5] = real_up.y;
    m[6] = -forward.y;
    m[7] = 0.0f;
    m[8] = right.z;
    m[9] = real_up.z;
    m[10] = -forward.z;
    m[11] = 0.0f;
    m[12] = -(right.x * eye.x + right.y * eye.y + right.z * eye.z);
    m[13] = -(real_up.x * eye.x + real_up.y * eye.y + real_up.z * eye.z);
    m[14] = (forward.x * eye.x + forward.y * eye.y + forward.z * eye.z);
    m[15] = 1.0f;
    return m;
}

std::array<float, 16> Camera::projection_matrix(int viewport_width, int viewport_height) const
{
    float aspect =
        (viewport_height > 0) ? static_cast<float>(viewport_width) / static_cast<float>(viewport_height) : 1.0f;
    float fov_rad = fov_ * 3.14159265f / 180.0f;
    float f = 1.0f / std::tan(fov_rad * 0.5f);

    std::array<float, 16> m = {};
    m[0] = f / aspect;
    m[5] = f;
    m[10] = (far_ + near_) / (near_ - far_);
    m[11] = -1.0f;
    m[14] = (2.0f * far_ * near_) / (near_ - far_);
    return m;
}

void Camera::on_mouse_button(int button, bool pressed, float mouse_x, float mouse_y)
{
    if (button == 0)
    { // Left
        left_dragging_ = pressed;
    }
    if (button == 1)
    { // Right
        right_dragging_ = pressed;
    }
    last_mouse_x_ = mouse_x;
    last_mouse_y_ = mouse_y;
}

void Camera::on_mouse_move(float mouse_x, float mouse_y)
{
    float dx = mouse_x - last_mouse_x_;
    float dy = mouse_y - last_mouse_y_;

    if (left_dragging_)
    {
        // Orbit
        yaw_ += dx * 0.005f;
        pitch_ += dy * 0.005f;
        // Clamp pitch to avoid gimbal lock
        const float max_pitch = 1.5f;
        if (pitch_ > max_pitch)
            pitch_ = max_pitch;
        if (pitch_ < -max_pitch)
            pitch_ = -max_pitch;
    }

    if (right_dragging_)
    {
        // Pan
        float cos_pitch = std::cos(pitch_);
        Vec3f right(std::cos(yaw_), 0.0f, -std::sin(yaw_));
        Vec3f up(0.0f, 1.0f, 0.0f);
        float scale = distance_ * 0.002f;
        target_.x -= right.x * dx * scale + up.x * dy * scale;
        target_.y -= right.y * dx * scale + up.y * dy * scale;
        target_.z -= right.z * dx * scale + up.z * dy * scale;
        (void) cos_pitch;
    }

    last_mouse_x_ = mouse_x;
    last_mouse_y_ = mouse_y;
}

void Camera::on_scroll(float y_offset)
{
    distance_ *= (1.0f - y_offset * 0.1f);
    if (distance_ < 0.5f)
        distance_ = 0.5f;
    if (distance_ > 200.0f)
        distance_ = 200.0f;
}

Vec3f Camera::screen_to_ray(float screen_x, float screen_y, int viewport_width, int viewport_height) const
{
    // Convert screen to NDC [-1, 1]
    float ndc_x = (2.0f * screen_x) / static_cast<float>(viewport_width) - 1.0f;
    float ndc_y = 1.0f - (2.0f * screen_y) / static_cast<float>(viewport_height);

    // Inverse projection: NDC -> view-space direction
    float fov_rad = fov_ * 3.14159265f / 180.0f;
    float f = 1.0f / std::tan(fov_rad * 0.5f);
    float aspect =
        (viewport_height > 0) ? static_cast<float>(viewport_width) / static_cast<float>(viewport_height) : 1.0f;

    float view_x = ndc_x * aspect / f;
    float view_y = ndc_y / f;
    float view_z = -1.0f;

    // Transform from view space to world space using inverse view matrix
    // We construct the camera basis vectors directly
    Vec3f eye = get_position();
    Vec3f fwd = (target_ - eye);
    float len = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y + fwd.z * fwd.z);
    if (len > 0.0f)
    {
        fwd.x /= len;
        fwd.y /= len;
        fwd.z /= len;
    }

    Vec3f world_up(0.0f, 1.0f, 0.0f);
    Vec3f right(fwd.y * world_up.z - fwd.z * world_up.y,
                fwd.z * world_up.x - fwd.x * world_up.z,
                fwd.x * world_up.y - fwd.y * world_up.x);
    float rlen = std::sqrt(right.x * right.x + right.y * right.y + right.z * right.z);
    if (rlen > 0.0f)
    {
        right.x /= rlen;
        right.y /= rlen;
        right.z /= rlen;
    }

    Vec3f up(right.y * fwd.z - right.z * fwd.y, right.z * fwd.x - right.x * fwd.z, right.x * fwd.y - right.y * fwd.x);

    // Ray direction in world space
    Vec3f ray_dir(right.x * view_x + up.x * view_y + fwd.x * view_z,
                  right.y * view_x + up.y * view_y + fwd.y * view_z,
                  right.z * view_x + up.z * view_y + fwd.z * view_z);

    float ray_len = std::sqrt(ray_dir.x * ray_dir.x + ray_dir.y * ray_dir.y + ray_dir.z * ray_dir.z);
    if (ray_len > 0.0f)
    {
        ray_dir.x /= ray_len;
        ray_dir.y /= ray_len;
        ray_dir.z /= ray_len;
    }

    return ray_dir;
}

} // namespace phynity::render

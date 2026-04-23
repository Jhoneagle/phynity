#pragma once

#include "camera.hpp"
#include "scene_renderer.hpp"

#include <optional>

namespace phynity::render
{

/// Result of a pick operation
struct PickResult
{
    int body_id = -1;
    float distance = 0.0f;
};

/// Test a screen-space click against scene bodies using ray-sphere intersection.
/// Returns the closest hit body ID, or nullopt if nothing hit.
std::optional<PickResult> pick_body(const Camera &camera,
                                     float screen_x,
                                     float screen_y,
                                     int viewport_width,
                                     int viewport_height,
                                     const SceneRenderer::State &state);

} // namespace phynity::render

#pragma once

#include <core/math/matrices/mat3.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/vectors/vec3.hpp>

#include <optional>

namespace phynity::render
{

using phynity::math::matrices::Mat3f;
using phynity::math::quaternions::Quatf;
using phynity::math::vectors::Vec3f;

/// Data struct for body inspector (decoupled from core physics types)
struct BodyInspectorData
{
    int id = -1;
    bool active = true;
    float lifetime = -1.0f;

    // Transform
    Vec3f position;
    Quatf orientation;
    Vec3f euler_angles; // for display

    // Linear dynamics
    Vec3f velocity;
    Vec3f force_accumulator;
    float mass = 0.0f;
    float inv_mass = 0.0f;

    // Angular dynamics
    Vec3f angular_velocity;
    Vec3f torque_accumulator;

    // Shape
    int shape_type = -1; // maps to ShapeType enum
    Vec3f shape_dimensions;
    float bounding_radius = 0.0f;

    // Material
    float restitution = 0.0f;
    float friction = 0.0f;
    float linear_damping = 0.0f;
    float angular_damping = 0.0f;

    // Energy
    float linear_ke = 0.0f;
    float angular_ke = 0.0f;
    float total_ke = 0.0f;
    Vec3f linear_momentum;
    Vec3f angular_momentum;
};

/// Edit requests returned when user modifies values while paused
struct BodyInspectorEdit
{
    std::optional<Vec3f> new_position;
    std::optional<Vec3f> new_velocity;
    std::optional<Vec3f> new_angular_velocity;
    std::optional<float> new_mass;
};

/// Draws a body inspector window. Returns edit requests if user modified values.
class BodyInspector
{
public:
    /// Draw inspector for the given body data.
    /// @param data Body properties to display
    /// @param is_paused Whether editing is allowed
    /// @param open Set to false when user closes the window
    /// @return Edit requests (empty if no edits)
    BodyInspectorEdit draw(const BodyInspectorData &data, bool is_paused, bool &open);
};

} // namespace phynity::render

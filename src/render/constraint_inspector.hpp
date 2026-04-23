#pragma once

#include <core/math/vectors/vec3.hpp>

#include <string>

namespace phynity::render
{

using phynity::math::vectors::Vec3f;

struct ConstraintInspectorData
{
    size_t index = 0;
    std::string type_name;
    float error = 0.0f;
    bool active = true;
};

/// Draws a constraint inspector window.
class ConstraintInspector
{
public:
    void draw(const ConstraintInspectorData &data, bool &open);
};

} // namespace phynity::render

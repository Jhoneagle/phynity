#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/physics_constants.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <memory>

namespace phynity::physics {

using phynity::math::vectors::Vec3f;
using namespace phynity::physics::constants;

/// ============================================================================
/// Abstract Force Field Base Class
/// ============================================================================

/// Represents a force field that applies forces to particles.
/// Subclasses implement specific force behaviors (gravity, drag, etc.).
class ForceField {
public:
    virtual ~ForceField() = default;

    /// Apply the force field to a particle at a given position with given velocity.
    /// @param position Current position of the particle
    /// @param velocity Current velocity of the particle
    /// @param mass Mass of the particle
    /// @return Force vector (in Newtons or equivalent units)
    virtual Vec3f apply(const Vec3f& position, const Vec3f& velocity, float mass) const = 0;

    /// Returns a human-readable name for this force field
    virtual const char* name() const = 0;
};

// ============================================================================
// Concrete Force Field Implementations
// ============================================================================

/// Uniform gravitational field with constant acceleration.
/// Applies F = m * g, where g is the gravitational acceleration vector.
class GravityField : public ForceField {
private:
    Vec3f gravity_;

public:
    /// Constructor with gravitational acceleration vector.
    /// @param gravity Acceleration due to gravity (default: Earth gravity, 9.80665 m/s²)
    constexpr explicit GravityField(const Vec3f& gravity = EARTH_GRAVITY_VECTOR)
        : gravity_(gravity)
    {
    }

    /// Apply gravity: F = m * g
    Vec3f apply(const Vec3f& /*position*/, const Vec3f& /*velocity*/, float mass) const override {
        return gravity_ * mass;
    }

    /// Get the gravitational acceleration vector
    constexpr Vec3f gravity() const {
        return gravity_;
    }

    /// Set new gravitational acceleration
    void set_gravity(const Vec3f& gravity) {
        gravity_ = gravity;
    }

    const char* name() const override {
        return "GravityField";
    }
};

/// Velocity-dependent damping force field.
/// Applies F = -drag_coefficient * velocity
/// Simulates air/fluid resistance proportional to velocity (linear drag).
class DragField : public ForceField {
private:
    float drag_coefficient_;

public:
    /// Constructor with drag coefficient.
    /// @param drag_coefficient Linear drag coefficient (>= 0)
    ///        Typical values: air ~0.01-0.1, water ~0.5-2.0, very viscous ~10+
    constexpr explicit DragField(float drag_coefficient = 0.0f)
        : drag_coefficient_(drag_coefficient)
    {
    }

    /// Apply linear drag: F = -drag_coefficient * velocity
    Vec3f apply(const Vec3f& /*position*/, const Vec3f& velocity, float /*mass*/) const override {
        return velocity * (-drag_coefficient_);
    }

    /// Get the current drag coefficient
    constexpr float drag_coefficient() const {
        return drag_coefficient_;
    }

    /// Set new drag coefficient
    void set_drag_coefficient(float coefficient) {
        drag_coefficient_ = coefficient;
    }

    const char* name() const override {
        return "DragField";
    }
};

/// Quadratic (squared velocity) drag field.
/// Applies F = -drag_coefficient * |velocity|^2 * velocity_direction
/// More realistic for high-speed motion through fluids.
class QuadraticDragField : public ForceField {
private:
    float drag_coefficient_;

public:
    /// Constructor with drag coefficient for quadratic drag.
    /// @param drag_coefficient Quadratic drag coefficient (>= 0)
    constexpr explicit QuadraticDragField(float drag_coefficient = 0.0f)
        : drag_coefficient_(drag_coefficient)
    {
    }

    /// Apply quadratic drag: F = -drag_coefficient * |velocity| * velocity
    Vec3f apply(const Vec3f& /*position*/, const Vec3f& velocity, float /*mass*/) const override {
        using phynity::math::utilities::is_zero;
        float speed = velocity.length();
        if (is_zero(speed, VELOCITY_EPSILON)) {
            return Vec3f(0.0f);
        }
        return velocity * (-drag_coefficient_ * speed);
    }

    /// Get the current drag coefficient
    constexpr float drag_coefficient() const {
        return drag_coefficient_;
    }

    /// Set new drag coefficient
    void set_drag_coefficient(float coefficient) {
        drag_coefficient_ = coefficient;
    }

    const char* name() const override {
        return "QuadraticDragField";
    }
};

/// Spring force field (Hooke's law style).
/// Applies restoring force toward a center point: F = -k * (position - center)
class SpringField : public ForceField {
private:
    Vec3f center_;
    float spring_constant_;

public:
    /// Constructor with center and spring constant.
    /// @param center The equilibrium position
    /// @param spring_constant Spring stiffness (k in F = -kx), typically > 0
    constexpr SpringField(const Vec3f& center = Vec3f(0.0f), float spring_constant = 1.0f)
        : center_(center), spring_constant_(spring_constant)
    {
    }

    /// Apply spring force: F = -k * (position - center)
    Vec3f apply(const Vec3f& position, const Vec3f& /*velocity*/, float /*mass*/) const override {
        Vec3f displacement = position - center_;
        return displacement * (-spring_constant_);
    }

    /// Get equilibrium center
    constexpr Vec3f center() const {
        return center_;
    }

    /// Set new equilibrium center
    void set_center(const Vec3f& center) {
        center_ = center;
    }

    /// Get spring constant
    constexpr float spring_constant() const {
        return spring_constant_;
    }

    /// Set new spring constant
    void set_spring_constant(float k) {
        spring_constant_ = k;
    }

    const char* name() const override {
        return "SpringField";
    }
};

}  // namespace phynity::physics

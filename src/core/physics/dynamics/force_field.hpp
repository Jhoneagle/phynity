#pragma once

#include <core/math/utilities/float_comparison.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/physics/config/physics_constants.hpp>
#include <core/physics/shapes/aabb.hpp>

#include <algorithm>
#include <memory>
#include <optional>

namespace phynity::physics
{

using phynity::math::vectors::Vec3f;
using phynity::physics::constants::EARTH_GRAVITY_VECTOR;
using phynity::physics::constants::VELOCITY_EPSILON;
using phynity::physics::shapes::AABB;

/// ============================================================================
/// Force Application Context
/// ============================================================================

/// Bundles the per-body state a force field reads when computing its force.
/// Passing a context (rather than a fixed parameter list) lets future fields
/// require new inputs additively: extend this struct and existing fields simply
/// ignore the new members, with no signature churn across every field/call site.
struct ForceContext
{
    Vec3f position{0.0f}; ///< Current position of the body
    Vec3f velocity{0.0f}; ///< Current velocity of the body
    float mass{0.0f}; ///< Mass of the body
    Vec3f gravity{EARTH_GRAVITY_VECTOR}; ///< Ambient gravitational acceleration (shared environment state)
    float charge{0.0f}; ///< Electric charge of the body (read by electromagnetic fields)
    // grows additively later: float volume; float temperature; ...
};

/// ============================================================================
/// Abstract Force Field Base Class
/// ============================================================================

/// Represents a force field that applies forces to particles.
/// Subclasses implement specific force behaviors (gravity, drag, etc.).
class ForceField
{
public:
    virtual ~ForceField() = default;

    /// Apply the force field to a body described by the given context.
    /// @param ctx Per-body state (position, velocity, mass, ...)
    /// @return Force vector (in Newtons or equivalent units)
    virtual Vec3f apply(const ForceContext &ctx) const = 0;

    /// Returns a human-readable name for this force field
    virtual const char *name() const = 0;
};

// ============================================================================
// Concrete Force Field Implementations
// ============================================================================

/// Uniform gravitational field with constant acceleration.
/// Applies F = m * g, where g is the gravitational acceleration vector.
class GravityField : public ForceField
{
private:
    Vec3f gravity_;

public:
    /// Constructor with gravitational acceleration vector.
    /// @param gravity Acceleration due to gravity (default: Earth gravity, 9.80665 m/s²)
    constexpr explicit GravityField(const Vec3f &gravity = EARTH_GRAVITY_VECTOR) : gravity_(gravity)
    {
    }

    /// Apply gravity: F = m * g
    Vec3f apply(const ForceContext &ctx) const override
    {
        return gravity_ * ctx.mass;
    }

    /// Get the gravitational acceleration vector
    constexpr Vec3f gravity() const
    {
        return gravity_;
    }

    /// Set new gravitational acceleration
    void set_gravity(const Vec3f &gravity)
    {
        gravity_ = gravity;
    }

    const char *name() const override
    {
        return "GravityField";
    }
};

/// Velocity-dependent damping force field.
/// Applies F = -drag_coefficient * velocity
/// Simulates air/fluid resistance proportional to velocity (linear drag).
class DragField : public ForceField
{
private:
    float drag_coefficient_;

public:
    /// Constructor with drag coefficient.
    /// @param drag_coefficient Linear drag coefficient (>= 0)
    ///        Typical values: air ~0.01-0.1, water ~0.5-2.0, very viscous ~10+
    constexpr explicit DragField(float drag_coefficient = 0.0f) : drag_coefficient_(drag_coefficient)
    {
    }

    /// Apply linear drag: F = -drag_coefficient * velocity
    Vec3f apply(const ForceContext &ctx) const override
    {
        return ctx.velocity * (-drag_coefficient_);
    }

    /// Get the current drag coefficient
    constexpr float drag_coefficient() const
    {
        return drag_coefficient_;
    }

    /// Set new drag coefficient
    void set_drag_coefficient(float coefficient)
    {
        drag_coefficient_ = coefficient;
    }

    const char *name() const override
    {
        return "DragField";
    }
};

/// Quadratic (squared velocity) drag field.
/// Applies F = -drag_coefficient * |velocity|^2 * velocity_direction
/// More realistic for high-speed motion through fluids.
class QuadraticDragField : public ForceField
{
private:
    float drag_coefficient_;

public:
    /// Constructor with drag coefficient for quadratic drag.
    /// @param drag_coefficient Quadratic drag coefficient (>= 0)
    constexpr explicit QuadraticDragField(float drag_coefficient = 0.0f) : drag_coefficient_(drag_coefficient)
    {
    }

    /// Apply quadratic drag: F = -drag_coefficient * |velocity| * velocity
    Vec3f apply(const ForceContext &ctx) const override
    {
        using phynity::math::utilities::is_zero;
        float speed = ctx.velocity.length();
        if (is_zero(speed, VELOCITY_EPSILON))
        {
            return Vec3f(0.0f);
        }
        return ctx.velocity * (-drag_coefficient_ * speed);
    }

    /// Get the current drag coefficient
    constexpr float drag_coefficient() const
    {
        return drag_coefficient_;
    }

    /// Set new drag coefficient
    void set_drag_coefficient(float coefficient)
    {
        drag_coefficient_ = coefficient;
    }

    const char *name() const override
    {
        return "QuadraticDragField";
    }
};

/// Spring force field (Hooke's law style).
/// Applies restoring force toward a center point: F = -k * (position - center)
class SpringField : public ForceField
{
private:
    Vec3f center_;
    float spring_constant_;

public:
    /// Constructor with center and spring constant.
    /// @param center The equilibrium position
    /// @param spring_constant Spring stiffness (k in F = -kx), typically > 0
    constexpr SpringField(const Vec3f &center = Vec3f(0.0f), float spring_constant = 1.0f)
        : center_(center), spring_constant_(spring_constant)
    {
    }

    /// Apply spring force: F = -k * (position - center)
    Vec3f apply(const ForceContext &ctx) const override
    {
        Vec3f displacement = ctx.position - center_;
        return displacement * (-spring_constant_);
    }

    /// Get equilibrium center
    constexpr Vec3f center() const
    {
        return center_;
    }

    /// Set new equilibrium center
    void set_center(const Vec3f &center)
    {
        center_ = center;
    }

    /// Get spring constant
    constexpr float spring_constant() const
    {
        return spring_constant_;
    }

    /// Set new spring constant
    void set_spring_constant(float k)
    {
        spring_constant_ = k;
    }

    const char *name() const override
    {
        return "SpringField";
    }
};

/// Radial ("point source") gravity field — a gravity well.
/// Applies an inverse-square attraction toward a center: F = m * strength / r² * dir_to_center,
/// where strength = G·M. A minimum-distance softening clamp bounds the force near the center
/// to avoid singularities.
class PointGravityField : public ForceField
{
private:
    Vec3f center_;
    float strength_; ///< G·M (gravitational parameter)
    float min_distance_; ///< Softening clamp on the effective distance

public:
    /// Constructor with center, strength, and softening distance.
    /// @param center The location of the attracting mass
    /// @param strength Gravitational parameter G·M (>= 0 for attraction)
    /// @param min_distance Minimum effective distance (softening clamp, > 0)
    constexpr PointGravityField(const Vec3f &center = Vec3f(0.0f), float strength = 1.0f, float min_distance = 1e-3f)
        : center_(center), strength_(strength), min_distance_(min_distance)
    {
    }

    /// Apply radial gravity: F = m * strength / r² toward the center.
    Vec3f apply(const ForceContext &ctx) const override
    {
        using phynity::math::utilities::is_zero;
        Vec3f d = center_ - ctx.position;
        float r2 = std::max(d.squaredLength(), min_distance_ * min_distance_);
        if (is_zero(r2))
        {
            return Vec3f(0.0f);
        }
        return d.normalized() * (ctx.mass * strength_ / r2);
    }

    /// Get the center of attraction
    constexpr Vec3f center() const
    {
        return center_;
    }

    /// Set the center of attraction
    void set_center(const Vec3f &center)
    {
        center_ = center;
    }

    /// Get the gravitational parameter (G·M)
    constexpr float strength() const
    {
        return strength_;
    }

    /// Set the gravitational parameter (G·M)
    void set_strength(float strength)
    {
        strength_ = strength;
    }

    /// Get the softening minimum distance
    constexpr float min_distance() const
    {
        return min_distance_;
    }

    /// Set the softening minimum distance
    void set_min_distance(float min_distance)
    {
        min_distance_ = min_distance;
    }

    const char *name() const override
    {
        return "PointGravityField";
    }
};

/// Wind / drag volume — pushes bodies toward a target wind velocity.
/// Applies F = c * (wind_velocity - velocity), i.e. a linear drag relative to a
/// moving air mass rather than to still air. Optionally gated to an AABB region,
/// so the wind only acts on bodies inside a bounded volume.
class WindField : public ForceField
{
private:
    Vec3f wind_velocity_;
    float drag_coefficient_;
    std::optional<AABB> region_; ///< Confining volume; unset means the wind acts everywhere

public:
    /// Constructor. Omit @p region (or pass std::nullopt) for wind that acts
    /// everywhere; pass an AABB to confine the wind to that volume.
    /// @param wind_velocity Velocity of the air mass
    /// @param drag_coefficient Coupling strength (>= 0)
    /// @param region Optional AABB the wind is confined to
    constexpr explicit WindField(const Vec3f &wind_velocity = Vec3f(0.0f),
                                 float drag_coefficient = 0.0f,
                                 std::optional<AABB> region = std::nullopt)
        : wind_velocity_(wind_velocity), drag_coefficient_(drag_coefficient), region_(region)
    {
    }

    /// Apply wind: F = c * (wind_velocity - velocity), gated to the region if bounded.
    Vec3f apply(const ForceContext &ctx) const override
    {
        if (region_ && !region_->contains_point(ctx.position))
        {
            return Vec3f(0.0f);
        }
        return (wind_velocity_ - ctx.velocity) * drag_coefficient_;
    }

    /// Get the wind velocity
    constexpr Vec3f wind_velocity() const
    {
        return wind_velocity_;
    }

    /// Set the wind velocity
    void set_wind_velocity(const Vec3f &wind_velocity)
    {
        wind_velocity_ = wind_velocity;
    }

    /// Get the drag coefficient
    constexpr float drag_coefficient() const
    {
        return drag_coefficient_;
    }

    /// Set the drag coefficient
    void set_drag_coefficient(float coefficient)
    {
        drag_coefficient_ = coefficient;
    }

    /// Whether the wind is confined to a bounded region
    constexpr bool is_bounded() const
    {
        return region_.has_value();
    }

    /// Get the bounding region (only meaningful when bounded)
    AABB region() const
    {
        return region_.value_or(AABB{});
    }

    const char *name() const override
    {
        return "WindField";
    }
};

/// Spring–damper field (damped harmonic oscillator).
/// Applies a restoring spring force plus a viscous damping force:
/// F = -k * (position - center) - c * velocity.
/// Unlike SpringField, this couples position and velocity so oscillations decay.
class SpringDamperField : public ForceField
{
private:
    Vec3f center_;
    float spring_constant_;
    float damping_;

public:
    /// Constructor with center, spring constant, and damping coefficient.
    /// @param center The equilibrium position
    /// @param spring_constant Spring stiffness (k), typically > 0
    /// @param damping Viscous damping coefficient (c), typically >= 0
    constexpr SpringDamperField(const Vec3f &center = Vec3f(0.0f), float spring_constant = 1.0f, float damping = 0.0f)
        : center_(center), spring_constant_(spring_constant), damping_(damping)
    {
    }

    /// Apply spring + damping: F = -k * (position - center) - c * velocity.
    Vec3f apply(const ForceContext &ctx) const override
    {
        return (ctx.position - center_) * (-spring_constant_) + ctx.velocity * (-damping_);
    }

    /// Get equilibrium center
    constexpr Vec3f center() const
    {
        return center_;
    }

    /// Set new equilibrium center
    void set_center(const Vec3f &center)
    {
        center_ = center;
    }

    /// Get spring constant
    constexpr float spring_constant() const
    {
        return spring_constant_;
    }

    /// Set new spring constant
    void set_spring_constant(float k)
    {
        spring_constant_ = k;
    }

    /// Get damping coefficient
    constexpr float damping() const
    {
        return damping_;
    }

    /// Set new damping coefficient
    void set_damping(float c)
    {
        damping_ = c;
    }

    const char *name() const override
    {
        return "SpringDamperField";
    }
};

/// Buoyancy field for simple fluids (Archimedes' principle).
/// A body submerged below a flat fluid surface experiences an upward force equal
/// to the weight of the displaced fluid: F = -gravity * ρ_fluid * V_submerged.
/// The body's volume is derived from its mass and density: V = mass / object_density.
/// A body fully above the surface receives no force. Submersion is treated as
/// all-or-nothing about the surface plane (no partial-submersion ramp).
///
/// Gravity (which defines "down", the surface's up axis, and the force magnitude)
/// is read from the shared ForceContext each step rather than stored, so buoyancy
/// always tracks the system's ambient gravity — changing gravity mid-simulation
/// (e.g. switching planets) can never leave a stale, divergent copy here.
class BuoyancyField : public ForceField
{
private:
    float fluid_density_; ///< Density of the surrounding fluid (kg/m³)
    float object_density_; ///< Density of the body, used to derive its volume (kg/m³)
    float surface_height_; ///< Height of the flat fluid surface along the up axis

public:
    /// Constructor with fluid/object densities and surface height.
    /// @param fluid_density Density of the fluid (kg/m³)
    /// @param object_density Density of the body (kg/m³, > 0)
    /// @param surface_height Height of the fluid surface along the up axis
    constexpr BuoyancyField(float fluid_density = 1000.0f, float object_density = 1000.0f, float surface_height = 0.0f)
        : fluid_density_(fluid_density), object_density_(object_density), surface_height_(surface_height)
    {
    }

    /// Apply buoyancy: upward force equal to the weight of displaced fluid.
    /// Uses the ambient gravity supplied by the context to orient and scale the force.
    Vec3f apply(const ForceContext &ctx) const override
    {
        using phynity::math::utilities::is_zero;
        if (is_zero(object_density_) || is_zero(ctx.gravity.squaredLength()))
        {
            return Vec3f(0.0f);
        }

        Vec3f up = ctx.gravity.normalized() * -1.0f;
        float depth = surface_height_ - ctx.position.dot(up);
        if (depth <= 0.0f)
        {
            return Vec3f(0.0f);
        }

        float volume = ctx.mass / object_density_;
        return ctx.gravity * (-fluid_density_ * volume);
    }

    /// Get the fluid density
    constexpr float fluid_density() const
    {
        return fluid_density_;
    }

    /// Set the fluid density
    void set_fluid_density(float density)
    {
        fluid_density_ = density;
    }

    /// Get the object density
    constexpr float object_density() const
    {
        return object_density_;
    }

    /// Set the object density
    void set_object_density(float density)
    {
        object_density_ = density;
    }

    /// Get the fluid surface height
    constexpr float surface_height() const
    {
        return surface_height_;
    }

    /// Set the fluid surface height
    void set_surface_height(float height)
    {
        surface_height_ = height;
    }

    const char *name() const override
    {
        return "BuoyancyField";
    }
};

// ============================================================================
// Electromagnetic Force Fields
// ============================================================================

/// Uniform electric field. Applies the electrostatic force on a charged body:
/// F = q * E, where q is the body's charge (read from the context) and E is the
/// constant field vector. Charge and field are in simulation units (see
/// COULOMB_CONSTANT); an uncharged body (q = 0) feels no force.
class UniformElectricField : public ForceField
{
private:
    Vec3f field_;

public:
    /// Constructor with the electric field vector.
    /// @param field Electric field E (simulation units)
    constexpr explicit UniformElectricField(const Vec3f &field = Vec3f(0.0f)) : field_(field)
    {
    }

    /// Apply the electric force: F = q * E
    Vec3f apply(const ForceContext &ctx) const override
    {
        return field_ * ctx.charge;
    }

    /// Get the electric field vector
    constexpr Vec3f field() const
    {
        return field_;
    }

    /// Set the electric field vector
    void set_field(const Vec3f &field)
    {
        field_ = field;
    }

    const char *name() const override
    {
        return "UniformElectricField";
    }
};

} // namespace phynity::physics

# Utilities

Mathematical utility functions and helpers for the core math module.

## Components

### Constants (`constants.hpp`)
- **Mathematical constants**: π, 2π, π/2, e, √2
- **Conversion factors**: degrees↔radians
- **Physics constants**: speed of light, gravitational constant, Earth gravity, Boltzmann constant, etc.
- **Tolerance thresholds**: rotation, collision, geometry, physics simulation
- **Precision helpers**: epsilon, infinity for floating-point types

### Trigonometry (`trigonometry.hpp`)
- **Angle normalization**: normalize angles to [-π, π]
- **Normalized trig functions**: `sin_normalized()`, `cos_normalized()`, `tan_normalized()`
- **Arc functions**: `asin()`, `acos()`, `atan()`, `atan2()`
- **Small-angle approximations**: 
  - sin(θ) ≈ θ for |θ| < 0.1 radians
  - cos(θ) ≈ 1 - θ²/2 for |θ| < 0.1 radians
  - tan(θ) ≈ θ for |θ| < 0.1 radians
- **Accuracy**: approximations validated for numerical stability in physics

### Conversions (`conversions.hpp`)
- **Degree↔Radian conversions**: `degrees_to_radians()`, `radians_to_degrees()`
- **Aliases**: `deg2rad()`, `rad2deg()` for brevity

### Numeric Helpers (`numeric.hpp`)
- **Clamping**: `clamp()` - restrict values to a range
- **Interpolation**: 
  - `lerp()` - linear interpolation
  - `inverse_lerp()` - find parameter for a value between two endpoints
  - `smoothstep()` - smooth Hermite interpolation
  - `smootherstep()` - smoother 5th-order interpolation
- **Min/Max utilities**: `min()`, `max()`, `abs()`, `sign()`
- **Fractional utilities**: `fract()`, `repeat()`, `ping_pong()`

### Float Comparison (`float_comparison.hpp`)
- **Validity checks**: 
  - `is_nan()` - check for NaN
  - `is_inf()` - check for infinity
  - `is_finite()` - check if not NaN or infinite
  - `is_normal()` - check if normal or zero
- **Approximate equality**:
  - `equals_absolute()` - within absolute tolerance
  - `equals_relative()` - within relative tolerance
  - `equals()` - combined absolute and relative tolerance
  - `is_zero()` - check if close to zero
- **Safe range clamping**: `clamp_to_safe_range()`

### Geometry (`geometry.hpp`)
- **Basic types**: `Vec3<T>`, `Plane<T>`, `Sphere<T>`, `AABB<T>`
- **Ray-Plane intersection**: `ray_plane_intersection()`
- **Sphere tests**:
  - `point_in_sphere()` - containment test
  - `point_on_sphere()` - surface proximity test
  - `sphere_sphere_intersection()` - intersection test
  - `closest_point_on_sphere()` - closest point computation
- **AABB tests**:
  - `point_in_aabb()` - containment test
  - `aabb_aabb_intersection()` - intersection test
  - `closest_point_on_aabb()` - closest point computation

## Usage

```cpp
#include <core/math/utilities/constants.hpp>
#include <core/math/utilities/trigonometry.hpp>
#include <core/math/utilities/conversions.hpp>
#include <core/math/utilities/numeric.hpp>
#include <core/math/utilities/float_comparison.hpp>
#include <core/math/utilities/geometry.hpp>

using namespace phynity::math::utilities;

// Mathematical constants
float pi = mathf::pi;
float half_pi = mathf::half_pi;

// Angle conversions
float angle_rad = degrees_to_radians(45.0f);
float angle_deg = radians_to_degrees(mathf::half_pi);

// Trigonometry
float normalized_sin = sin_normalized(angle_rad);
float small_sin = sin_small_angle(0.05f);  // ≈ 0.05

// Numeric helpers
float clamped = clamp(value, 0.0f, 1.0f);
float interpolated = lerp(a, b, 0.5f);

// Floating-point comparison
if (equals(a, b, 1e-6f)) {
    // Values are approximately equal
}

// Geometry
Sphere<float> sphere(0, 0, 0, 1.0f);
bool contains = point_in_sphere(point, sphere);
```

## Technical Debt Resolution

This module eliminates technical debt by centralizing:
- All mathematical constants (previously scattered as `#define M_PI` macros)
- Common numerical operations (lerp, clamp, smoothstep)
- Consistent floating-point comparisons with proper tolerances
- Geometry helpers for physics and collision detection

All test files have been refactored to use `phynity::math::utilities::mathf::pi` instead of local macro definitions.

## Status

✅ Phase 3 Complete: All utility components implemented with comprehensive test coverage.


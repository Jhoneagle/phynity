# Vectors

This module contains fixed and variable-length vector implementations for use in physics, graphics, and general linear algebra computations.

## Headers

- **vec2.hpp** - Specialized 2D vector, optimized for 2D physics and screen coordinates
- **vec3.hpp** - Specialized 3D vector, the most commonly used vector for 3D physics and graphics
- **vec4.hpp** - Specialized 4D vector, for homogeneous coordinates and RGBA colors
- **vec_n.hpp** - Template-based fixed-length vectors (`VecN<N>`) with compile-time known size
- **vec_dynamic.hpp** - Runtime-sized vectors for when dimensions are not known at compile time

## Classes

### Vec2, Vec3, Vec4
Hand-optimized specializations with member variables `x`, `y`, (z), (w).

**Operations:**
- Arithmetic: `+`, `-`, `*` (scalar/component-wise), `/` (scalar/component-wise)
- Assignment: `+=`, `-=`, `*=`, `/=`
- Comparison: `==`, `!=`
- Index access: `operator[int]`
- Vector: `dot()`, `cross()` (3D only), `length()`, `normalized()`, `distance()`, `angle()`
- Utilities: `lerp()`, `project()`, `reflect()`, `clamp()`, `min()`, `max()`, `isZero()`, `isNormalized()`

### VecN<N>
Template-based vector with `std::array<float, N>` storage.

**Use cases:**
- 6D vectors for 6-DOF physics (rotations + translations)
- 8D+ for neural networks or specialized applications
- Compile-time optimization - zero runtime overhead

**Aliases provided:**
- `Vec6`, `Vec8`, `Vec16`

### VecDynamic
Runtime-sized vector using `std::vector<float>`.

**Use cases:**
- Data-driven systems with variable dimensions
- Machine learning frameworks
- Cases where size isn't known at compile time

**Note:** Includes bounds checking via `at()` and throws exceptions on size mismatches.

## Namespace

All vectors are in the nested namespace `phynity::math::vectors`.

```cpp
using phynity::math::vectors::Vec2;
using phynity::math::vectors::Vec3;
using phynity::math::vectors::Vec4;
```

## Performance Considerations

- **Vec2/Vec3/Vec4:** Best for performance-critical code (physics, rendering)
- **VecN<N>:** Near-zero overhead for compile-time known sizes
- **VecDynamic:** Runtime overhead due to dynamic allocation; avoid in hot loops

## Examples

```cpp
// Vec3 - most common
Vec3 pos(1.0f, 2.0f, 3.0f);
Vec3 vel = Vec3::forward() * 5.0f;
float dist = pos.distance(vel);

// Vec2 for 2D
Vec2 screen(800.0f, 600.0f);
Vec2 up = Vec2::up();

// VecN<6> for 6-DOF
VecN<6> state;  // 3 linear + 3 angular
state[0] = 1.0f;  // x position

// VecDynamic for unknown size
VecDynamic weights(100);
weights[0] = 0.5f;
```

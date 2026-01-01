# Matrices

This module contains matrix implementations for linear transformations and 3D graphics operations.

## Headers

- **mat2.hpp** - Specialized 2×2 matrix for 2D transformations
- **mat3.hpp** - Specialized 3×3 matrix for 2D homogeneous coordinates and 3D rotations
- **mat4.hpp** - Specialized 4×4 matrix for 3D transformations (the most commonly used)
- **mat_n.hpp** - Template-based MxN matrices for general linear algebra
- **mat_dynamic.hpp** - Runtime-sized matrices

## Classes

### Mat2, Mat3, Mat4
Hand-optimized specializations with row-major storage `m[row][col]`.

**Operations:**
- Arithmetic: `+`, `-`, `*` (scalar/matrix/vector), `/` (scalar)
- Assignment: `+=`, `-=`, `*=`, `/=`
- Comparison: `==`, `!=`
- Element access: `operator()(row, col)`
- Matrix operations: `determinant()`, `inverse()`, `transposed()`, `transpose()`, `trace()`
- Utilities: `identity()`, `zero()`, `rotation()`, `scale()`
- Matrix-vector multiplication: `matrix * vector`, `vector * matrix`

### MatN<M, N>
Template-based matrix with compile-time dimensions using `std::array` storage.

**Use cases:**
- Compile-time sized matrices with zero runtime overhead
- Custom dimensions for specific algorithms
- Linear algebra operations

**Operations:**
- Arithmetic: `+`, `-`, `*` (scalar/matrix/vector), `/` (scalar)
- Assignment: `+=`, `-=`, `*=`, `/=`
- Element access: `operator()(row, col)`
- Matrix ops (square N≤4): `trace()`, `minor(r,c)`, `cofactor()`, `determinant()`, `inverse()`
- Utilities: `identity()` (square), `zero()`, `transposed()`, `transpose()`

### MatDynamic
Runtime-sized matrix using `std::vector<float>` storage.

**Use cases:**
- Data-driven systems with variable dimensions
- Machine learning frameworks
- Cases where size isn't known at compile time

**Operations:**
- Arithmetic: `+`, `-`, `*` (scalar/matrix/vector), `/` (scalar)
- Assignment: `+=`, `-=`, `*=`, `/=`
- Element access: `operator()(row, col)`, `at(row, col)` (bounds-checked)
- Matrix ops: `transposed()`, `transpose()`, `identity(n)`, `zero(r, c)`

## Namespace

All matrices are in the nested namespace `phynity::math::matrices`.

```cpp
using phynity::math::matrices::Mat2;
using phynity::math::matrices::Mat4;
```

## Performance Considerations

- **Mat2/Mat3/Mat4:** Best for performance-critical code (physics, rendering)
- **MatN<M, N>:** Near-zero overhead for compile-time known dimensions
- **MatDynamic:** Use only when dimensions are truly dynamic

## Status

- **Mat2:** ✓ Implemented
- **Mat3:** ✓ Implemented
- **Mat4:** ✓ Implemented
- **MatN:** ✓ Implemented (small-N determinant/inverse up to 4×4)
- **MatDynamic:** ✓ Implemented
- **Linear algebra operations:** determinant/inverse available for Mat2/3/4 and MatN (N≤4)

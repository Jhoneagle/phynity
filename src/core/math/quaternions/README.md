# Quaternions

This module provides quaternion implementations for efficient 3D rotations in the physics engine.

## Implemented Classes

- **Quat** (`quat.hpp`) - Quaternion for representing 3D rotations without gimbal lock
- **Conversions** (`quat_conversions.hpp`) - Conversion utilities between quaternions and other rotation representations
- **Interpolation** (`quat_interpolation.hpp`) - Quaternion interpolation functions for smooth rotation transitions

## Status

### ✅ Basic Arithmetic
- Addition, subtraction, negation
- Quaternion multiplication (rotation composition)
- Scalar multiplication and division
- Dot product

### ✅ Conversions
- **To/from rotation matrices** (3×3)
- **To/from Euler angles** (roll, pitch, yaw with ZYX order)
- **To/from axis-angle representation**
- Numerically stable implementations using Shepperd's method and gimbal lock handling

### ✅ Core Operations
- Normalization (in-place and copy)
- Magnitude and squared magnitude
- Conjugate and inverse
- Vector rotation (`rotateVector`, `unrotateVector`)

### ✅ Interpolation
- **NLERP** (Normalized Linear Interpolation) - Fast approximation for smooth rotation interpolation
  - Automatic shortest-path selection
  - Suitable for physics state blending and real-time applications

### ❌ Pending Implementation
- **SLERP** (Spherical Linear Interpolation) - Planned for constant angular velocity interpolation

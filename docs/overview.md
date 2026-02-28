# Project Overview

**Status**: Active Development (production-leaning)  
**Last Updated**: February 28, 2026

## Current Milestones

### ✅ Completed
- **Core Math Bedrock**: Comprehensive vector/matrix/quaternion library with linear algebra
- **Calculus Utilities**: Numerical differentiation, integration, and curve fitting
- **Particle Simulation v2**: Deterministic particle physics engine with force fields and collisions
- **Collision System**: Broadphase + narrowphase + contact resolution pipeline
- **Constraint System**: Solver, contact constraints, fixed joints

### 📋 Planned
- **Parallel Core Expansion**: Scheduling improvements and task graph tooling
- **Rigid Body Dynamics**: Continued expansion (constraints, stability, demos)

See [roadmap.md](roadmap.md) for detailed timeline.

## Docs Index

- [architecture.md](architecture.md) - System design and module boundaries
- [phynity_user_guide.md](phynity_user_guide.md) - Usage patterns and scale guidance
- [roadmap.md](roadmap.md) - Feature timeline and milestones
- [observability.md](observability.md) - Profiling and diagnostics
- [test_organization.md](test_organization.md) - Test mapping and structure
- [testing_best_practices.md](testing_best_practices.md) - Test standards
- [golden_tests.md](golden_tests.md) - Golden test workflow

## Build, Test, Run

Use the wrappers in `tools/` as the primary workflow.

### Windows

```bat
tools\build.bat debug
tools\test.bat debug
tools\run.bat debug
```

### Linux/macOS/MSYS

```bash
./tools/build.sh debug
./tools/test.sh debug
./tools/run.sh debug
```

---

## Technical Achievements

### Physics Engine (v2)
The physics subsystem is feature-complete for particles and has a working rigid-body pipeline:
- **Deterministic simulation**: Bit-identical results for identical inputs
- **Material system**: Preset materials (steel, rubber, wood, fluid) with extensible properties
- **Force fields**: Pluggable architecture (gravity, drag, custom)
- **Collision system**: Broadphase + narrowphase with contact caching and resolution
- **Timestep control**: Fixed timestep accumulator with overflow handling
- **Diagnostics**: Real-time energy and momentum tracking
- **Test coverage**: 70 passing tests with analytical validation

### Mathematics Library
Robust numerical foundation for simulation and scientific computing:
- **Vector types**: vec2, vec3, vec4, vec_n (static), vec_dynamic
- **Matrix types**: mat2, mat3, mat4, mat_n (static), mat_dynamic
- **Quaternions**: Full support with conversions and interpolation (slerp, nlerp)
- **Linear algebra**: LU, QR, SVD, Cholesky decomposition with numerical stability
- **Calculus utilities**: Finite differences, numerical integrators, curve fitting

### Quality Assurance
- **Unit testing**: Catch2-based test suite (unit + validation + golden)
- **Validation testing**: Reference-based tests against analytical solutions
- **Determinism validation**: Fixed timestep reproducibility guarantees
- **Build system**: CMake with vcpkg dependency management

---

## Motivation

This project was created as a long-term learning and experimentation
platform focused on physics simulation, mathematics, and modern C++
software engineering.

The author has a strong background in software development and wants to
use this project to deepen practical knowledge in:

- Modern C++ design and tooling
- Numerical methods and physics simulation
- Concurrency and parallel computing
- Performance-oriented and data-oriented design
- Engine-style architecture and scalability

## Learning-Oriented Design

This is intentionally a hobby and free-time project. Design decisions
prioritize:

- Clarity over premature optimization
- Incremental complexity
- Replaceability of components
- Strong tooling and observability

The project is expected to evolve over time as understanding deepens.

## Long-Term Vision

In the long term, the project aims to evolve toward a production-quality,
industry-grade simulation engine, without sacrificing its role as an
experimentation and learning environment.
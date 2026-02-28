# Architecture Overview

## High-Level Structure

The project is organized into clearly separated layers:

- Core simulation (math, physics, memory, jobs)
- Platform abstractions
- Optional rendering and visualization
- Application-level executables

The simulation core is fully decoupled from rendering and platform code.

## Core Principles

- Data-oriented design where appropriate
- Clear ownership and lifetime rules
- Deterministic simulation steps when possible
- Explicit dependencies between modules

## Module Responsibilities

### core/
Contains all platform-agnostic logic:
- **Math and numerical utilities** (`core/math/`)
  - Vectors, matrices, quaternions with SIMD potential
  - Linear algebra: LU, QR, SVD, Cholesky decomposition
  - Calculus: finite differences, numerical integrators, curve fitting
- **Physics simulation** (`core/physics/`)
  - Particle-based simulation with material system
  - Force fields (gravity, drag, custom)
  - Timestep controller with determinism guarantees
  - Collision detection and response (sphere-sphere)
  - Energy and momentum diagnostics
- **Memory management** (`core/memory/`)
  - Thread-safe arena allocator for per-frame allocations
- **Job system and concurrency primitives** (`core/jobs/`)
  - Minimal job system with deterministic scheduling mode
  - Parallel-for helper for data-parallel loops

### platform/
Thin abstractions over OS and compiler features such as:
- Threading
- Timing
- CPU feature detection

### render/
Visualization and debugging tools only.
Depends on core, never the other way around.

### app/
Executable entry points and experiment sandboxes.
Minimal logic; orchestration only.

## Dependency Rules

- core must not depend on platform or render
- render may depend on core
- app may depend on everything
## Physics Subsystem Details

The physics subsystem (`core/physics/`) implements a deterministic particle-based simulation engine with the following architecture:

### Component Overview

```
ParticleSystem (manager)
    ├── Particle[] (state container)
    │   └── Material (properties)
    ├── ForceField[] (pluggable forces)
    │   ├── GravityField
    │   ├── DragField
    │   └── CustomField (extensible)
    └── TimestepController (determinism)
```

### Core Components

**Particle** (`particle.hpp`)
- Kinematic state: position, velocity, acceleration
- Force accumulation pattern for composable forces
- Material reference for physical properties
- Lifecycle management (lifetime, active flags)
- Semi-implicit Euler integration
- Collision properties (radius)

**Material** (`material.hpp`)
- Physical properties: mass, restitution, friction
- Damping coefficients: linear, angular (reserved)
- Drag coefficient for velocity-dependent forces
- Preset factories: steel, rubber, wood, fluid_particle
- Designed for data-driven extension

**ForceField** (`force_field.hpp`)
- Abstract base class for pluggable force systems
- Concrete implementations: GravityField, DragField
- Extensible via CustomField interface
- Applied per-particle each simulation step

**TimestepController** (`timestep_controller.hpp`)
- Accumulator pattern for fixed timestep physics
- Overflow handling modes: CLAMP, SUBDIVIDE, UNCONSTRAINED
- Determinism enforcement and validation
- Statistics tracking (steps, overflows, subdivisions)

**ParticleSystem** (`particle_system.hpp`)
- Manages particle lifecycle (spawn, remove, update)
- Force field registration and application
- Collision detection and impulse-based resolution
- Energy and momentum diagnostics
- Update pipeline: clear forces → apply fields → integrate → collide → cleanup

### Determinism Contract

The physics subsystem guarantees bit-identical results for identical input sequences when:
- Using fixed timestep mode
- Applying forces in consistent order
- Using the same floating-point precision settings

This is validated through reference-based tests comparing against analytical solutions.

### Integration Points

**Application Layer** (`app/`)
- `PhysicsContext`: Manages subsystem lifecycle and configuration
- `demo_scenarios.hpp`: Reusable scenario templates
- Example: GravityWell, OrbitStability, MultiParticleCollision

**Testing**
- 15+ unit tests for component behaviors
- 6 reference-based determinism tests with analytical validation
- 10+ validation scenarios for energy/momentum conservation
- All tests use Catch2 framework

### Design Decisions

1. **Semi-implicit Euler integration**: Chosen for energy stability over explicit Euler
2. **Force accumulation pattern**: Enables composable, debuggable force application
3. **Material-based properties**: Clear separation of particle state vs. physical constants
4. **Pluggable force fields**: Extensible without modifying core particle logic
5. **Collision at core level**: Integrated into ParticleSystem rather than app layer

### Performance Characteristics

- O(n) force application per field
- O(n²) collision detection (brute force, suitable for small-to-medium particle counts)
- Update loop can use the job system for data-parallel passes when configured

### Future Extensions

See [roadmap.md](roadmap.md) for planned additions:
- Broadphase spatial acceleration (spatial grid)
- Rigid body dynamics with inertia tensors
- Constraint solvers (joints, contacts)
- Continuous collision detection (CCD)
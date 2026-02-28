# Architecture Overview

### Dual-Scale Physics Engine

```
PhysicsEngine
├── ParticleSystem (micro-scale)
│   ├── Point mass particles
│   ├── Broadphase + narrowphase collisions
│   └── No rotation
│
└── RigidBodySystem (macro-scale)
    ├── 6-DOF rigid bodies (position + rotation)
    ├── Inertia tensors
    ├── Angular dynamics
    ├── Convex shape collisions (sphere, box, capsule)
    ├── Constraints (fixed; hinge planned)
    └── Deterministic simulation support
```

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
  - Collision detection and response (broadphase + narrowphase + contact)
  - Constraints (solver, joints, contact)
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

The physics subsystem (`core/physics/`) implements deterministic micro- and macro-scale simulation with shared collision and constraint infrastructure.

Collision is organized into `broadphase/`, `narrowphase/`, `shapes/`, and `contact/` modules. Constraints are split into `solver/`, `joints/`, and `contact/` for clearer responsibilities.

### Current Module Layout

```text
core/physics/
├── common/
├── micro/
├── macro/
├── collision/
│   ├── broadphase/
│   ├── narrowphase/
│   ├── shapes/
│   └── contact/
└── constraints/
    ├── solver/
    ├── joints/
    └── contact/
```

### Component Overview

```
PhysicsContext
├── Micro: ParticleSystem
│   ├── Particle[] + Material
│   ├── ForceField[]
│   └── Collision + constraint integration
├── Macro: RigidBodySystem
│   ├── RigidBody[] + Shape + Inertia
│   ├── ForceField[]
│   └── Constraint solving
├── Collision Pipeline
│   ├── broadphase/
│   ├── narrowphase/
│   ├── shapes/
│   └── contact/
└── Constraints Pipeline
    ├── solver/
    ├── joints/
    └── contact/
```

### Core Components by Subsystem

**Common** (`common/`)
- Material properties and shared constants
- Force field abstraction and implementations
- Timestep control and determinism primitives

**Micro Scale** (`micro/`)
- Particle state and lifecycle management
- Force accumulation and integration
- Broadphase-assisted collision response for particle workloads

**Macro Scale** (`macro/`)
- 6-DOF rigid body state and integration
- Inertia tensor and shape-driven dynamics
- Constraint-driven contact/joint stabilization

**Collision** (`collision/`)
- Broadphase candidate generation
- Narrowphase manifold generation (sphere/AABB/GJK/EPA/SAT)
- Contact cache and impulse/PGS resolution components

**Constraints** (`constraints/`)
- Solver abstraction and iterative solver execution
- Contact constraints
- Joint constraints (fixed currently, additional joint types planned)

### Determinism Contract

The physics subsystem guarantees bit-identical results for identical input sequences when:
- Using fixed timestep mode
- Applying forces in consistent order
- Using the same floating-point precision settings

This is validated through reference-based tests comparing against analytical solutions.

### Integration Points

**Application Layer** (`app/`)
- `PhysicsContext`: Manages subsystem lifecycle and configuration
- `main.cpp` + scenario orchestration entry points
- Demo executable via `tools/run.(bat|sh)`

**Testing**
- 70 automated tests across unit and validation suites
- Reference-based determinism tests for reproducible stepping
- Golden and performance regression checks for key scenarios
- All tests use Catch2 framework

### Design Decisions

1. **Semi-implicit Euler integration**: Chosen for energy stability over explicit Euler
2. **Force accumulation pattern**: Enables composable, debuggable force application
3. **Material-based properties**: Clear separation of particle state vs. physical constants
4. **Pluggable force fields**: Extensible without modifying core particle logic
5. **Collision at core level**: Integrated in simulation systems rather than app-layer glue

### Performance Characteristics

- O(n) force application per field
- O(n^2) collision detection for brute-force fallback; spatial grid used for scale
- Update loop can use the job system for data-parallel passes when configured

### Future Extensions

See [roadmap.md](roadmap.md) for planned additions:
- Continuous collision detection (CCD)
- Advanced rigid-body constraints (hinges, motors, limits)
- Solver improvements and stability tuning
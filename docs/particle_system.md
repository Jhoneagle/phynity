# Particle System v2 - Design & Implementation Plan

## Overview
Upgrade from basic particle implementation to a robust, deterministic particle simulation system with variable timestep support, force fields, and material presets. This bridges math/calculus utilities into a cohesive physics subsystem.

---

## Current State

### What Exists
- **Math primitives**: vectors, matrices, quaternions, linear algebra
- **Calculus utilities**: forward/central differences, Euler and semi-implicit Euler integrators, curve fitting
- **Physics core**: material system, force fields, timestep controller, refactored particle, particle system manager
- **Collision system**: sphere-sphere collision detection and impulse-based resolution in core physics
- **Diagnostics**: energy and momentum tracking in particle system
- **Application integration**: PhysicsContext, demo scenarios, main loop integration
- **Validation tests**: comprehensive physics validation suite with analytical checks and reference-based determinism tests
- **Testing**: 31 unit tests and validation tests passing

### What's Missing
- Documentation updates (Doxygen + architecture/integration guides)
- CI/build script integration for validation tests

### Still Missing (Focused List)
- **Documentation updates** (Doxygen + docs/architecture.md, integration guides)
- **CI integration** for validation tests

---

## Design Goals

1. **Determinism**: Identical input sequences produce bit-identical results across runs
2. **Flexibility**: Support variable timesteps without breaking deterministic physics
3. **Performance**: Efficient force accumulation and integration patterns
4. **Debuggability**: Built-in energy/momentum tracking for validation
5. **Extensibility**: Material presets and force fields as pluggable systems

---

## Architecture

### Core Components

#### 1. Material System
```
Material:
  - mass
  - restitution (bounce coefficient)
  - friction
  - linear damping
  - angular damping (placeholder for v2)
  - drag coefficient
```
Predefined presets (kept in physics core for now, data-driven ready): steel, rubber, wood, fluid particle, etc. Treat this as the semi-final shape so refactors are minimal later.

#### 2. Force Fields
```
ForceField (abstract):
  - gravity field (constant acceleration)
  - drag field (velocity-dependent damping)
  - custom field interface (for extensions)
```

#### 3. Particle (refactored)
```
Particle:
  - position (vec3)
  - velocity (vec3)
  - acceleration (vec3, computed per step)
  - material (reference or embedded)
  - force accumulator (vec3)
  - lifetime/active flags (for pooling)
```

#### 4. Particle System Manager
```
ParticleSystem:
  - container of particles
  - active force fields
  - timestep controller
  - integration method selector
  - energy/momentum diagnostic tracker
```

#### 5. Timestep Controller
```
TimestepController:
  - target framerate / max timestep
  - accumulator pattern for deterministic stepping
  - overflow handling (clamping, subdivision)
  - determinism contract enforcement
```

---

## Implementation Plan

### Phase 1: Core Refactoring (src/core/physics/)

#### Step 1.1: Material System
- [x] Define `Material` struct with preset factory
- [x] Create material presets (steel, rubber, fluid particle)
- [x] Keep presets in physics core but design for future data-driven loading (table-based, constexpr defaults now)
- [x] Add unit tests for material creation and defaults
- **Files**: `src/core/physics/material.hpp`, `material.cpp`
- **Tests**: `tests/UnitTests/core/physics/material_test.cpp`

#### Step 1.2: Force Fields
- [x] Define abstract `ForceField` base class
- [x] Implement `GravityField` (constant acceleration)
- [x] Implement `DragField` (velocity-dependent)
- [x] Add custom field interface for extension
- [x] Add unit tests for field evaluation
- **Files**: `src/core/physics/force_field.hpp`, `force_field.cpp`
- **Tests**: `tests/UnitTests/core/physics/force_field_test.cpp`

#### Step 1.3: Timestep Controller
- [x] Define `TimestepController` with accumulator pattern
- [x] Implement determinism guardrails (max timestep, subdivision)
- [x] Support fixed timestep for tests, adaptive for real-time
- [x] Add diagnostics (overflow counts, step statistics)
- [x] Add unit tests for accumulation and overflow handling
- **Files**: `src/core/physics/timestep_controller.hpp`, `timestep_controller.cpp`
- **Tests**: `tests/UnitTests/core/physics/timestep_controller_test.cpp`

#### Step 1.4: Particle Refactor
- [x] Expand `Particle` with material reference, force accumulator, lifetime
- [x] Add methods: `apply_force()`, `clear_forces()`, `reset_acceleration()`
- [x] Support pooling/recycling for performance
- [x] Add unit tests for particle state management
- **Files**: `src/core/physics/particle.hpp`, `particle.cpp`
- **Tests**: `tests/UnitTests/core/physics/particle_test.cpp` (Refactor to use catch2)

#### Step 1.5: Particle System Manager
- [x] Define `ParticleSystem` class with container management
- [x] Implement `add_particle()`, `remove_particle()`, `update_all()`
- [x] Integrate force field application: `apply_forces(dt)`
- [x] Integrate integration step: `integrate(dt)` using calculus utilities
- [x] Add energy/momentum tracking for diagnostics
- [x] Add collision detection and response system: `enable_collisions()`, `resolve_collisions()`
- [x] Implement sphere-sphere impulse-based collision with penetration correction
- [x] Add unit tests for system update cycles
- **Files**: `src/core/physics/particle_system.hpp`, `particle_system.cpp`
- **Tests**: `tests/UnitTests/core/physics/particle_system_test.cpp`

#### Step 1.6: Physics Module Integration
- [x] Update existing `src/core/CMakeLists.txt` (no new nested CMake needed)
- [x] If physics adds `.cpp` files, switch `phynity_core` from `INTERFACE` to `STATIC` (or `OBJECT`) and list the new sources
- [x] Ensure physics headers stay exported via `target_include_directories`
- [x] Verify compilation and linking

---

### Phase 2: Example Application Integration (src/app/)

#### Step 2.1: Application Physics Context
- [x] Create `PhysicsContext` in app to manage particle system lifetime
- [x] Initialize default materials and force fields (Earth gravity, minimal drag)
- [x] Set up timestep controller for 60 FPS target with determinism mode option
- **Files**: `src/app/physics_context.hpp`, `physics_context.cpp`

#### Step 2.2: Demo Scenarios
- [x] **Gravity well**: particles falling under constant gravity
- [x] **Orbit stability**: circular orbit test (validates energy conservation)
- [x] **Drag interaction**: particles slowing with drag field
- [x] **Multi-particle collision**: sphere-sphere impulse-based collision using core physics system
- [x] **Core collision integration**: ParticleSystem.enable_collisions() for engine-level collision resolution
- **Files**: `src/app/demo_scenarios.hpp`, `demo_scenarios.cpp`

#### Step 2.3: Main Application Update
- [x] Integrate `PhysicsContext` into main loop
- [x] Call `timestep_controller.step(delta_time)`
- [x] Execute `particle_system.update(dt)` for each accumulated timestep
- [ ] Render particles (if render system available)
- [x] Display diagnostics (energy, momentum, particle count)
- **Files**: `src/app/main.cpp`

---

### Phase 3: Unit Testing (tests/UnitTests/core/physics/)

#### Step 3.1: Individual Component Tests
- [x] Material tests: creation, presets, defaults
- [x] Force field tests: gravity evaluation, drag computation
- [x] Timestep controller tests: accumulation, overflow, determinism
- [x] Particle tests: state updates, pooling
- [x] Particle system tests: add/remove, update cycles, force application

Each test file follows pattern:
```cpp
#include <catch2/catch_all.hpp>
#include "component.hpp"

TEST_CASE("ComponentName - scenario") {
  // Setup
  // Execute
  // Assert with tolerance (e.g., relative error < 1e-6 for floats)
}
```

---

### Phase 4: Validation Testing (tests/ValidationTests/)

#### Step 4.1: Fixed-Expectation Scenarios (new, same harness style as existing validation)
- [x] **Gravity well / fall**: analytical `y(t) = y0 - 0.5 * g * t^2` (1-2% tolerance)
- [x] **Circular orbit**: spring-centered orbit with radius drift tolerance
- [x] **Energy conservation** (multi-particle closed system): drift tolerance in validation tests
- [x] **Momentum conservation** (no-external-forces setup): absolute tolerance in validation tests
- [x] **Collision validation**: sphere-sphere impulse conservation and energy behavior

#### Step 4.2: Validation Test Harness
- [x] Implement `tests/ValidationTests/physics_validation_test.cpp`
- [x] Embed closed-form expectations in code (no external files)
- [x] Assert outputs against those expectations with tolerances

#### Step 4.3: Reference-Based Determinism Tests
- [x] **Free fall reference**: validates discrete semi-implicit Euler against closed-form solution
- [x] **Constant velocity reference**: validates no-force motion with position accumulation
- [x] **Constant acceleration reference**: validates initial velocity + gravity with analytical solution
- [x] **Linear drag reference**: validates velocity-dependent damping with geometric series
- [x] **2D coupled motion**: validates independent X/Y dynamics (lateral velocity + gravity)
- [x] **Spring oscillator**: validates harmonic motion against discrete recurrence relation
- [x] All tests use fixed timestep with tolerance adjusted for float accumulation (1e-4 to 1e-5)

---

### Phase 5: Documentation & Cleanup

#### Step 5.1: API Documentation
- [ ] Add Doxygen comments to all public headers
- [ ] Document Material presets and how to add custom ones
- [ ] Document ForceField interface for custom implementations
- [ ] Document TimestepController configuration options

#### Step 5.2: Integration Guide
- [ ] Update `docs/architecture.md` with physics subsystem overview
- [ ] Document "how to add a new force field" tutorial
- [ ] Document "how to add determinism check" for new features
- [ ] Document energy/momentum tracking for validation

#### Step 5.3: Build & CI
- [ ] Verify physics module builds in all configurations
- [ ] Add physics tests to CI pipeline
- [ ] Ensure validation tests run and report golden test diffs
- [ ] Update build scripts if needed (`tools/build.bat`, `tools/test.bat`)

---

## Dependencies & Prerequisites

### Required
- Math primitives (✅ complete)
- Calculus utilities with integrators (✅ complete)
- Validation test harness (✅ exists)

### Dependencies to Verify
- Catch2 for unit testing (should be in vcpkg)
- CMake configuration for physics module
- Render system (optional, for visualization)

---

## Deliverables Checklist

### Phase 1: Core Refactoring
- [x] Material system with presets
- [x] Force field abstractions (gravity, drag)
- [x] Timestep controller with determinism guardrails
- [x] Refactored Particle class
- [x] ParticleSystem manager
- [x] Physics module integration in build system
- [x] 15+ unit tests for physics components

### Phase 2: Application Integration
- [x] PhysicsContext in app
- [x] Demo scenarios (gravity well, orbit, drag, collision)
- [x] Main loop physics integration
- [x] Diagnostics display (energy, momentum)

### Phase 3: Unit Testing
- [x] Component unit tests (Material, ForceField, Timestep, Particle, ParticleSystem)

### Phase 4: Validation Testing
- [x] Fixed-expectation scenarios (gravity fall, orbit, conservation, collisions)
- [x] Validation test harness asserts against embedded reference values
- [x] Reference-based determinism tests (6 scenarios with closed-form analytical solutions)
- [x] Comprehensive coverage for regression detection during future refactoring

### Phase 5: Documentation
- [ ] API documentation (Doxygen comments)
- [ ] Architecture update
- [ ] Integration guide
- [ ] Build system verified

---

## Estimated Effort

| Phase | Task | Estimated Time |
|-------|------|-----------------|
| 1.1   | Material System | 2 hours |
| 1.2   | Force Fields | 3 hours |
| 1.3   | Timestep Controller | 4 hours |
| 1.4   | Particle Refactor | 3 hours |
| 1.5   | ParticleSystem Manager | 5 hours |
| 1.6   | Build Integration | 1 hour |
| **Phase 1 Total** | | **18 hours** |
| 2.1   | Physics Context | 2 hours |
| 2.2   | Demo Scenarios | 3 hours |
| 2.3   | Main App Integration | 2 hours |
| **Phase 2 Total** | | **7 hours** |
| 3.1-3.2 | Unit & Integration Tests | 8 hours |
| **Phase 3 Total** | | **8 hours** |
| 4.1-4.3 | Validation Tests | 6 hours |
| **Phase 4 Total** | | **6 hours** |
| 5.1-5.3 | Documentation & Polish | 4 hours |
| **Phase 5 Total** | | **4 hours** |
| | **TOTAL** | **~43 hours** |

---

## Success Criteria

- ✅ All physics components compile without warnings
- ✅ 31 tests passing (15 unit tests + 16 validation tests)
- ✅ 6 reference-based determinism tests validating analytical correctness
- ✅ 10+ scenario validation tests for energy, momentum, collisions, and orbit stability
- ✅ Example app runs demo scenarios and displays diagnostics
- ✅ Energy/momentum conserved within validation tolerances
- ✅ Determinism validated: simulations match closed-form analytical solutions
- ✅ Collision system integrated at core physics level
- [ ] API documented with Doxygen comments
- [ ] Architecture guide updated
- ✅ Zero regressions in existing tests

---

## Risk Mitigation

| Risk | Mitigation |
|------|-----------|
| Floating-point precision loss | Use relative tolerance comparisons; add numerical stability assertions |
| Determinism breakage | Test with fixed timestep first; validate bit-for-bit reproduction |
| Performance regression | Profile force accumulation hot loop; optimize if needed |
| Build system complexity | Start with simple physics CMakeLists.txt; integrate incrementally |
| Testing gaps | Write tests as features are implemented; use property-based testing for invariants |

---

## Notes

- All components should use modern C++20 idioms (templates, concepts where applicable)
- Keep physics module decoupled from render system (render is optional)
- Prioritize determinism over performance initially; optimize later
- Document design decisions as you encounter unexpected complexities
- Consider creating a simple particle inspector tool early (helpful for debugging)

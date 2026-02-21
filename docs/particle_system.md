# Particle System v2 - Design & Implementation Plan

## Overview
Upgrade from basic particle implementation to a robust, deterministic particle simulation system with variable timestep support, force fields, and material presets. This bridges math/calculus utilities into a cohesive physics subsystem.

---

## Current State

### What Exists
- **Math primitives**: vectors, matrices, quaternions, linear algebra
- **Calculus utilities**: forward/central differences, Euler and semi-implicit Euler integrators, curve fitting
- **Basic particle**: minimal particle class (likely position, velocity)
- **Validation harness**: integration test framework with golden tests support
- **Testing**: 24 unit tests, 100% passing

### What's Missing
- Variable timestep controller with determinism guardrails
- Force field system (gravity, drag, custom fields)
- Material system (mass, restitution, friction, damping coefficients)
- Multi-particle interactions (basic collision response)
- Energy/momentum diagnostics for physics validation
- Integration into example application (src/app/)
- Comprehensive physics unit tests (Catch2 testing framework)
- Physics validation tests (multi-particle scenarios)

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
- [ ] Define `Material` struct with preset factory
- [ ] Create material presets (steel, rubber, fluid particle)
- [ ] Keep presets in physics core but design for future data-driven loading (table-based, constexpr defaults now)
- [ ] Add unit tests for material creation and defaults
- **Files**: `src/core/physics/material.hpp`, `material.cpp`
- **Tests**: `tests/UnitTests/core/physics/material_test.cpp`

#### Step 1.2: Force Fields
- [ ] Define abstract `ForceField` base class
- [ ] Implement `GravityField` (constant acceleration)
- [ ] Implement `DragField` (velocity-dependent)
- [ ] Add custom field interface for extension
- [ ] Add unit tests for field evaluation
- **Files**: `src/core/physics/force_field.hpp`, `force_field.cpp`
- **Tests**: `tests/UnitTests/core/physics/force_field_test.cpp`

#### Step 1.3: Timestep Controller
- [ ] Define `TimestepController` with accumulator pattern
- [ ] Implement determinism guardrails (max timestep, subdivision)
- [ ] Support fixed timestep for tests, adaptive for real-time
- [ ] Add diagnostics (overflow counts, step statistics)
- [ ] Add unit tests for accumulation and overflow handling
- **Files**: `src/core/physics/timestep_controller.hpp`, `timestep_controller.cpp`
- **Tests**: `tests/UnitTests/core/physics/timestep_controller_test.cpp`

#### Step 1.4: Particle Refactor
- [ ] Expand `Particle` with material reference, force accumulator, lifetime
- [ ] Add methods: `apply_force()`, `clear_forces()`, `reset_acceleration()`
- [ ] Support pooling/recycling for performance
- [ ] Add unit tests for particle state management
- **Files**: `src/core/physics/particle.hpp`, `particle.cpp`
- **Tests**: `tests/UnitTests/core/physics/particle_test.cpp` (Refactor to use catch2)

#### Step 1.5: Particle System Manager
- [ ] Define `ParticleSystem` class with container management
- [ ] Implement `add_particle()`, `remove_particle()`, `update_all()`
- [ ] Integrate force field application: `apply_forces(dt)`
- [ ] Integrate integration step: `integrate(dt)` using calculus utilities
- [ ] Add energy/momentum tracking for diagnostics
- [ ] Add unit tests for system update cycles
- **Files**: `src/core/physics/particle_system.hpp`, `particle_system.cpp`
- **Tests**: `tests/UnitTests/core/physics/particle_system_test.cpp`

#### Step 1.6: Physics Module Integration
- [ ] Update existing `src/core/CMakeLists.txt` (no new nested CMake needed)
- [ ] If physics adds `.cpp` files, switch `phynity_core` from `INTERFACE` to `STATIC` (or `OBJECT`) and list the new sources
- [ ] Ensure physics headers stay exported via `target_include_directories`
- [ ] Verify compilation and linking

---

### Phase 2: Example Application Integration (src/app/)

#### Step 2.1: Application Physics Context
- [ ] Create `PhysicsContext` in app to manage particle system lifetime
- [ ] Initialize default materials and force fields (Earth gravity, minimal drag)
- [ ] Set up timestep controller for 60 FPS target with determinism mode option
- **Files**: `src/app/physics_context.hpp`, `physics_context.cpp`

#### Step 2.2: Demo Scenarios
- [ ] **Gravity well**: particles falling under constant gravity
- [ ] **Orbit stability**: circular orbit test (validates energy conservation)
- [ ] **Drag interaction**: particles slowing with drag field
- [ ] **Multi-particle collision**: simple sphere-sphere response
- **Files**: `src/app/demo_scenarios.hpp`, `demo_scenarios.cpp`

#### Step 2.3: Main Application Update
- [ ] Integrate `PhysicsContext` into main loop
- [ ] Call `timestep_controller.step(delta_time)`
- [ ] Execute `particle_system.update(dt)` for each accumulated timestep
- [ ] Render particles (if render system available)
- [ ] Display diagnostics (energy, momentum, particle count)
- **Files**: `src/app/main.cpp`

---

### Phase 3: Unit Testing (tests/UnitTests/core/physics/)

#### Step 3.1: Individual Component Tests
- [ ] Material tests: creation, presets, defaults
- [ ] Force field tests: gravity evaluation, drag computation
- [ ] Timestep controller tests: accumulation, overflow, determinism
- [ ] Particle tests: state updates, pooling
- [ ] Particle system tests: add/remove, update cycles, force application

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
- [ ] **Gravity well / fall**: analytical `y(t) = y0 - 0.5 * g * t^2`, rel error < 1e-4 (same pattern as current free-fall validation)
- [ ] **Circular orbit** (new): radius ± 0.1%, energy drift ± 0.01% using closed-form expectations
- [ ] **Energy conservation** (new multi-particle closed system): drift < 0.1% over 100 steps
- [ ] **Momentum conservation** (new no-external-forces setup): total momentum constant within float tolerance

#### Step 4.2: Validation Test Harness
- [ ] Implement `tests/ValidationTests/particle_system_validation.cpp`
- [ ] Embed closed-form expectations in code (no external files), same style as current free-fall/oscillator/pendulum tests
- [ ] Assert outputs against those expectations with tolerances
- [ ] Reuse shared scenario builders so app demos and tests stay consistent

#### Step 4.3: Determinism Validation (optional)
- [ ] Check simulations against the same embedded expectations (not run-vs-run comparisons)
- [ ] Use fixed timestep for strict determinism; adaptive mode can be tolerance-based
- [ ] Log deviations beyond tolerance

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
- [ ] Material system with presets
- [ ] Force field abstractions (gravity, drag)
- [ ] Timestep controller with determinism guardrails
- [ ] Refactored Particle class
- [ ] ParticleSystem manager
- [ ] Physics module integration in build system
- [ ] 15+ unit tests for physics components

### Phase 2: Application Integration
- [ ] PhysicsContext in app
- [ ] Demo scenarios (gravity well, orbit, drag, collision)
- [ ] Main loop physics integration
- [ ] Diagnostics display (energy, momentum)

### Phase 3: Unit Testing
- [ ] Component unit tests (Material, ForceField, Timestep, Particle, ParticleSystem)

### Phase 4: Validation Testing
- [ ] Fixed-expectation scenarios (gravity fall, orbit, conservation)
- [ ] Validation test harness asserts against embedded reference values
- [ ] Optional determinism check against fixed references

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
- ✅ 30+ unit tests passing (material, fields, timestep, particle, system)
- ✅ 5+ validation tests matching embedded reference expectations within tolerance
- ✅ Example app runs demo scenarios and displays diagnostics
- ✅ Energy/momentum conserved to within 0.1% over 100-step simulations
- ✅ Determinism validated: identical inputs produce identical outputs
- ✅ API documented with Doxygen comments
- ✅ Architecture guide updated
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

## Next Steps

1. **Validate this plan** with project stakeholders (none, solo project) ✓
2. **Begin Phase 1.1**: Material System implementation
3. **Commit frequently**: Each sub-step completion should be a separate commit
4. **Update log.txt** with progress as phases complete
5. **Iterate on validation**: Run golden tests early and often to catch issues

---

## Notes

- All components should use modern C++20 idioms (templates, concepts where applicable)
- Keep physics module decoupled from render system (render is optional)
- Prioritize determinism over performance initially; optimize later
- Document design decisions as you encounter unexpected complexities
- Consider creating a simple particle inspector tool early (helpful for debugging)

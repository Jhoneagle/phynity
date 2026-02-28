# Physics Engine Restructuring Plan

## Executive Summary

Transform the current flat physics directory structure into a modular, professional-grade architecture that cleanly separates:
- **Common infrastructure** (shared by all physics scales)
- **Micro-scale physics** (particle systems, SPH, molecular dynamics)
- **Macro-scale physics** (rigid bodies, soft bodies, articulated systems)
- **Cross-scale systems** (collision, constraints, forces)

## Goals

1. **Clarity**: Clear module boundaries with explicit dependencies
2. **Scalability**: Support micro (atoms/molecules) to macro (vehicles/buildings) seamlessly
3. **Maintainability**: Easy to locate, modify, and extend components
4. **Testability**: Each module independently testable
5. **Performance**: Enable scale-specific optimizations without coupling
6. **Professional Quality**: Industry-standard patterns and practices

## Current State Analysis

### Current Structure (Flat)
```
core/physics/
├── collision/              (17 files - mixed purpose)
├── constraints/            (5 files - mixed purpose)
├── force_field.cpp/hpp     (shared)
├── inertia.hpp             (rigid body specific)
├── material.cpp/hpp        (shared)
├── particle.cpp/hpp        (micro-scale)
├── particle_system.cpp/hpp (micro-scale)
├── physics_constants.hpp   (shared)
├── rigid_body.hpp          (macro-scale)
├── rigid_body_system.hpp   (macro-scale)
├── shape.hpp               (macro-scale)
└── timestep_controller.cpp/hpp (shared)
```

### Issues
1. ❌ No clear separation between micro and macro scale code
2. ❌ Collision code mixed (some for particles, some for rigid bodies)
3. ❌ Constraints unclear (contact vs. joint vs. particle)
4. ❌ Difficult to find relevant code for a given scale
5. ❌ No clear extension points for new physics types
6. ❌ Header-only rigid body code (should have .cpp for build speed)

## Proposed New Structure

### Phase 1: Core Reorganization

```
core/physics/
├── common/                          # Shared infrastructure
│   ├── material.hpp/cpp             # Physical properties
│   ├── physics_constants.hpp        # Universal constants
│   ├── timestep_controller.hpp/cpp  # Fixed timestep manager
│   ├── force_field.hpp/cpp          # Abstract force system
│   ├── diagnostics.hpp              # Energy/momentum tracking
│   └── physics_types.hpp            # Common type aliases
│
├── micro/                           # Particle-scale physics
│   ├── particle.hpp/cpp             # Point mass particle
│   ├── particle_system.hpp/cpp      # Particle manager
│   ├── particle_forces.hpp          # Particle-specific forces
│   └── sph/                         # Future: SPH fluid simulation
│
├── macro/                           # Rigid body scale physics
│   ├── rigid_body.hpp/cpp           # 6-DOF rigid body
│   ├── rigid_body_system.hpp/cpp    # Rigid body manager
│   ├── inertia.hpp/cpp              # Inertia tensor utilities
│   ├── shape.hpp/cpp                # Collision shapes
│   └── articulated/                 # Future: joints, vehicles
│
├── collision/                       # Universal collision detection
│   ├── broadphase/
│   │   ├── spatial_grid.hpp/cpp
│   │   ├── spatial_grid_hash.hpp
│   │   └── aabb.hpp
│   ├── narrowphase/
│   │   ├── sphere_sphere.hpp
│   │   ├── aabb_narrowphase.hpp
│   │   ├── gjk_solver.hpp/cpp
│   │   ├── epa_solver.hpp/cpp
│   │   └── sat_solver.hpp/cpp
│   ├── shapes/
│   │   ├── shape_base.hpp
│   │   ├── shape_factory.hpp
│   │   ├── convex_hull.hpp/cpp
│   │   └── support_function.hpp
│   └── contact/
│       ├── contact_manifold.hpp
│       ├── contact_cache.hpp/cpp
│       └── impulse_resolver.hpp/cpp
│
├── constraints/                     # Joint and contact constraints
│   ├── constraint_base.hpp          # Abstract constraint interface
│   ├── solver/
│   │   ├── constraint_solver.hpp/cpp
│   │   └── pgs_solver.hpp/cpp
│   ├── joints/                      # Macro-scale joints
│   │   ├── fixed_joint.hpp/cpp
│   │   ├── hinge_joint.hpp/cpp
│   │   ├── ball_joint.hpp           # Future
│   │   └── prismatic_joint.hpp      # Future
│   └── contact/                     # Contact constraints
│       ├── contact_constraint.hpp/cpp
│       └── friction_model.hpp       # Future
│
└── solvers/                         # Integration and solvers
    ├── integrators.hpp              # Euler, RK4, Verlet, etc.
    ├── linear_solver.hpp            # For constraint solving
    └── parallel_solver.hpp          # Future: multi-threaded solving
```

### Phase 2: Benefits of New Structure

#### 1. **Clear Module Boundaries**
```cpp
// Micro-scale simulation
#include <core/physics/micro/particle_system.hpp>
#include <core/physics/common/force_field.hpp>

// Macro-scale simulation
#include <core/physics/macro/rigid_body_system.hpp>
#include <core/physics/constraints/joints/hinge_joint.hpp>

// Both use same collision infrastructure
#include <core/physics/collision/broadphase/spatial_grid.hpp>
```

#### 2. **Scale-Specific Optimizations**
- Micro: SoA (Structure of Arrays) for cache efficiency with 1000s of particles
- Macro: AoS (Array of Structures) for complex rigid body state
- Collision: Shared infrastructure with scale-appropriate broadphase

#### 3. **Independent Testing**
```
tests/UnitTests/core/physics/
├── common/                  # Test shared infrastructure
├── micro/                   # Test particle physics in isolation
├── macro/                   # Test rigid body physics in isolation
├── collision/               # Test collision detection
└── integration/             # Test cross-scale interactions
```

#### 4. **Clear Extension Points**
```cpp
// Add new micro-scale physics (e.g., SPH)
core/physics/micro/sph/
    ├── sph_particle.hpp
    ├── sph_kernel.hpp
    └── sph_system.hpp

// Add new macro-scale physics (e.g., soft bodies)
core/physics/macro/soft_body/
    ├── soft_body.hpp
    ├── spring_constraint.hpp
    └── soft_body_system.hpp
```

## Migration Strategy

### Phase 1: Common Infrastructure (Week 1)
1. ✅ Create `common/` directory
2. ✅ Move and refactor shared components:
   - material.hpp/cpp
   - physics_constants.hpp
   - timestep_controller.hpp/cpp
   - force_field.hpp/cpp
3. ✅ Create diagnostics.hpp for energy/momentum tracking
4. ✅ Update all includes across codebase
5. ✅ Run all tests to verify no breakage

### Phase 2: Micro-Scale Separation (Week 1-2)
1. ✅ Create `micro/` directory
2. ✅ Move particle-specific code:
   - particle.hpp/cpp
   - particle_system.hpp/cpp
3. ✅ Extract particle-specific forces to particle_forces.hpp
4. ✅ Update includes and tests
5. ✅ Verify particle demos still work

### Phase 3: Macro-Scale Separation (Week 2)
1. ✅ Create `macro/` directory
2. ✅ Move rigid body code:
   - rigid_body.hpp → rigid_body.hpp/cpp (split implementation)
   - rigid_body_system.hpp → rigid_body_system.hpp/cpp
   - inertia.hpp → inertia.hpp/cpp
   - shape.hpp → shape.hpp/cpp
3. ✅ Update includes and tests
4. ✅ Verify rigid body demos still work

### Phase 4: Collision Reorganization (Week 3)
1. ✅ Reorganize collision/ into broadphase/narrowphase/shapes/contact
2. ✅ Move files to appropriate subdirectories
3. ✅ Update all collision includes
4. ✅ Run collision tests

### Phase 5: Constraints Reorganization (Week 3)
1. ✅ Reorganize constraints/ into solver/joints/contact
2. ✅ Split fixed_constraint_rb into joints/fixed_joint
3. ✅ Create clear separation between joint constraints and contact constraints
4. ✅ Update constraint test organization

### Phase 6: Documentation Update (Week 4)
1. ✅ Update architecture.md with new structure
2. ✅ Create PHYSICS_GUIDE.md explaining micro vs macro scale usage
3. ✅ Update roadmap.md with clearer scale separation
4. ✅ Document module dependencies
5. ✅ Create migration guide for external users

### Phase 7: Build System Update (Week 4)
1. ✅ Update CMakeLists.txt to reflect new structure
2. ✅ Create separate targets for micro/macro/common
3. ✅ Enable parallel compilation
4. ✅ Add module-level tests

## Code Quality Standards

### 1. **Header/Implementation Split**
- All non-trivial classes should have .cpp files
- Templates and inline functions stay in headers
- Reduces compile times and improves encapsulation

### 2. **Explicit Dependencies**
```cpp
// Good: Clear what you're using
#include <core/physics/common/material.hpp>
#include <core/physics/macro/rigid_body.hpp>

// Bad: Unclear where components come from
#include <core/physics/physics.hpp>  // Monolithic header
```

### 3. **Forward Declarations**
Use forward declarations in headers to minimize includes:
```cpp
// rigid_body.hpp
namespace phynity::physics {
    class Material;  // Forward declaration
    class Shape;
}
```

### 4. **Namespace Organization**
```cpp
namespace phynity::physics {
    namespace common { /* Shared code */ }
    namespace micro { /* Particle systems */ }
    namespace macro { /* Rigid bodies */ }
    namespace collision { /* Universal collision */ }
    namespace constraints { /* Joint and contact constraints */ }
}
```

### 5. **Testing Standards**
- Unit tests for each component in isolation
- Integration tests for cross-module interactions
- Golden tests for determinism validation
- Performance regression tests for critical paths

## Success Criteria

### Immediate (After Restructuring)
- ✅ All existing tests pass
- ✅ Zero functional regressions
- ✅ Build times improved by 20%+ (header/impl split)
- ✅ Clear module boundaries visible in code

### Short Term (1-2 months)
- ✅ New contributors can easily locate relevant code
- ✅ Can add new physics scale (e.g., SPH) without touching existing scales
- ✅ Test coverage increased by module-level testing
- ✅ Documentation accurately reflects structure

### Long Term (3-6 months)
- ✅ Performance optimizations isolated to specific scales
- ✅ Support for multi-scale simulations (particles + rigid bodies)
- ✅ Plugin system for custom physics modules
- ✅ Industry-grade code quality metrics

## Risk Mitigation

### Risk: Breaking Changes During Migration
**Mitigation**: 
- Incremental migration with tests at each step
- Keep old includes working with deprecation warnings initially
- Git branching strategy: feature/physics-restructure

### Risk: Test Coverage Gaps
**Mitigation**:
- Audit test coverage before migration
- Add missing tests during restructuring
- Run existing demos as integration tests

### Risk: Build System Complexity
**Mitigation**:
- Maintain flat CMakeLists.txt initially
- Modularize build gradually
- Document CMake patterns clearly

### Risk: Documentation Drift
**Mitigation**:
- Update docs alongside code changes
- Review architecture.md at each phase
- Auto-generate module dependency graphs

## Timeline Summary

**Total Duration**: 4 weeks (part-time effort)

- Week 1: Common infrastructure + Micro separation
- Week 2: Macro separation
- Week 3: Collision + Constraints reorganization
- Week 4: Documentation + Build system finalization

**Parallelization Opportunities**:
- Documentation can be updated alongside code changes
- Test improvements can happen concurrently
- Build system updates can follow code structure changes

## Next Steps

1. **Review and Approve Plan**: Discuss any concerns or modifications
2. **Create Feature Branch**: `feature/physics-restructure`
3. **Begin Phase 1**: Common infrastructure migration
4. **Iterative Review**: Check in after each phase completion
5. **Final Integration**: Merge to main after all phases complete

---

**Document Version**: 1.0  
**Last Updated**: 2026-02-28  
**Status**: Proposed - Awaiting Approval

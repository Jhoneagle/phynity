# Test Organization

This document maps Phynity test coverage by feature, subsystem, and test type.

## Purpose

- Make it easy to find the right test file for a feature.
- Keep naming, layout, and ownership consistent.
- Separate deterministic regression checks (golden) from behavioral validation.
- Clarify where to add new tests as physics systems evolve.

## Test Taxonomy

Phynity uses three main test types:

- Unit tests
  - Narrow API-level correctness.
  - Fast and deterministic.
  - Located in `tests/UnitTests/`.
- Validation tests
  - Scenario-level physics behavior checks.
  - May use tolerances and longer sequences.
  - Located in `tests/ValidationTests/`.
- Golden tests
  - Snapshot-based regression detection.
  - Stored in `tests/golden_outputs/`.
  - Mostly run under validation executables with golden labels.

## Top-Level Layout

```text
tests/
├── UnitTests/
├── ValidationTests/
├── golden_outputs/
├── test_utils/
├── golden_tests.md
├── test_organization.md
└── testing_best_practices.md
```

## Unit Test Layout

```text
tests/UnitTests/
├── core/
│   ├── diagnostics/
│   ├── math/
│   ├── physics/
│   └── jobs/
└── CMakeLists.txt
```

### Unit Test Conventions

- Target naming: `unit.<domain>.<subject>`
- File naming: `<subject>_test.cpp`
- Scope: one component per file whenever practical
- Deterministic assertions preferred over long scenario loops

## Validation Test Layout

```text
tests/ValidationTests/
├── math/
├── diagnostics/
├── physics/
│   ├── particles/
│   ├── materials/
│   ├── constraints/
│   └── collision/
│       ├── broadphase/
│       ├── narrowphase/
│       └── contact/
└── performance/
```

### Validation Test Conventions

- Target naming: `validation.<domain>.<subject>`
- File naming: `<subject>_validation_test.cpp` or `<subject>_golden_test.cpp`
- Stability/robustness tests use finite checks and bounded assertions
- Performance tests measure and emit machine-local outputs

## Golden Output Layout

```text
tests/golden_outputs/
├── diagnostics/
├── math/
├── physics/
│   ├── particles/
│   ├── collision/
│   ├── constraints/
│   └── materials/
└── performance/
```

### Golden File Conventions

- Scenario names should be stable and descriptive.
- Include duration or frame count where relevant.
- Avoid embedding machine-specific metadata in functional goldens.
- Performance baselines use paired files:
  - `<scenario>.json` baseline
  - `<scenario>.current.json` latest run

## Feature-to-Test Mapping

This section maps major engine features to canonical test locations.

### Particle Simulation

- Integration behavior
  - `tests/ValidationTests/physics/particles/particle_golden_test.cpp`
  - `tests/ValidationTests/physics/particles/integration_scenes_golden_test.cpp`
- Determinism replay
  - Existing particle and scene golden suites
- Long sequence stability
  - Particle validation scenes with bounded trajectory checks

### Material System

- Restitution behavior and decay
  - `tests/ValidationTests/physics/materials/restitution_validation_test.cpp`
- Friction behavior and edge coefficients
  - `tests/ValidationTests/physics/materials/friction_validation_test.cpp`
- Cross-material interactions
  - `tests/ValidationTests/physics/materials/material_interaction_test.cpp`

### Constraint Solver

- Convergence properties
  - `tests/ValidationTests/physics/constraints/convergence_validation_test.cpp`
- Stability and drift
  - `tests/ValidationTests/physics/constraints/stability_validation_test.cpp`
- Interaction edge cases
  - `tests/ValidationTests/physics/constraints/constraint_interaction_edge_cases_test.cpp`
- Jacobian correctness
  - `tests/UnitTests/core/physics/constraint_jacobian_test.cpp`

### Collision Pipeline

- Broadphase
  - `tests/ValidationTests/physics/collision/broadphase/broadphase_correctness_validation_test.cpp`
  - `tests/ValidationTests/physics/collision/broadphase/broadphase_performance_benchmark_test.cpp`
- Narrowphase
  - `tests/ValidationTests/physics/collision/narrowphase/sat_validation_test.cpp`
  - `tests/ValidationTests/physics/collision/narrowphase/gjk_epa_validation_test.cpp`
- Contact and resolution
  - `tests/ValidationTests/physics/collision/contact/contact_cache_golden_test.cpp`
  - `tests/ValidationTests/physics/collision/contact/pgs_solver_golden_test.cpp`
  - `tests/ValidationTests/physics/collision/contact/stacking_scenarios_test.cpp`
- CCD-oriented scenarios
  - `tests/ValidationTests/physics/collision/ccd_validation_test.cpp`
- Collision baseline captures
  - `tests/ValidationTests/physics/collision/collision_golden_test.cpp`

### Diagnostics and Observability

- Runtime monitors
  - `tests/UnitTests/core/diagnostics/`
- Validation and overhead checks
  - `tests/ValidationTests/diagnostics/`

### Performance Regression

- Scenario benchmark producer
  - `tests/ValidationTests/performance/collision_regression_test.cpp`
- Regression gate script
  - `tools/performance_regression_check.py`
- Baseline/current outputs
  - `tests/golden_outputs/performance/`

## By-Type Summary

### Unit

Use for:

- Mathematical identities and invariants.
- Jacobian structure checks.
- Utility and helper correctness.
- Diagnostics monitor state transitions.

Avoid in unit tests:

- Very long simulation loops unless necessary.
- Dependence on wall-clock timing.
- Broad integration scenarios that span many systems.

### Validation

Use for:

- Multi-step physics behavior.
- Cross-system interactions.
- Robustness under edge conditions.
- Long-run finite and bounded checks.

Avoid in validation tests:

- Tight exact-equality checks on floating-point trajectories.
- Assertions likely to vary by toolchain without tolerance.

### Golden

Use for:

- Deterministic baseline snapshots.
- Expected stable outputs over time.
- Regression detection after refactoring.

Avoid in golden tests:

- Scenarios expected to vary per machine clock.
- Non-deterministic random inputs without fixed seeds.

## Naming and Tagging Standards

### File Names

- Unit: `<subject>_test.cpp`
- Validation: `<subject>_validation_test.cpp`
- Golden: `<subject>_golden_test.cpp`
- Performance: `<subject>_regression_test.cpp` or `<subject>_benchmark_test.cpp`

### Test Case Names

- Format: `domain.subject.behavior`
- Examples:
  - `constraint_interaction.extreme_mass_ratios`
  - `ccd.high_speed_glancing_collision`
  - `validation.physics.collision.gjk_epa`

### Labels

- `validation` for validation suites
- `golden` for golden baseline comparisons
- `unit` via target naming conventions
- Add narrow feature tags when useful (`materials`, `constraints`, `collision`)

## Where to Add New Tests

Use this quick routing table:

- New vector/matrix/quaternion behavior
  - Add under `tests/UnitTests/core/math/`
- New constraint API or Jacobian logic
  - Add under `tests/UnitTests/core/physics/`
- New solver behavior scenario
  - Add under `tests/ValidationTests/physics/constraints/`
- New collision broadphase/narrowphase/contact behavior
  - Add under corresponding collision subsystem folder
- New deterministic trajectory baseline
  - Add under relevant validation folder plus `tests/golden_outputs/`
- New performance benchmark
  - Add under `tests/ValidationTests/performance/`

## Reorganization Notes (Current)

Collision validation tests are organized by subsystem:

- `broadphase/`
- `narrowphase/`
- `contact/`

This keeps CMake targets scoped by responsibility while preserving existing test names.

## Review Checklist for PRs Touching Tests

- Does the file location match the taxonomy above?
- Are test names consistent with domain/behavior naming?
- If golden files changed, were diffs reviewed intentionally?
- If performance changed, were current/baseline outputs compared?
- Are tolerances justified and documented in comments?
- Do new tests avoid hidden randomness?

## Planned Growth Areas

- Additional manifold aging sequences under contact validation.
- Expanded asymmetric material pair golden baselines.
- Optional dedicated CI target for performance regression compare step.
- Consolidated flaky-test guardrails for cross-toolchain reproducibility.

## Maintenance Guidance

- Prefer extending existing files before creating many tiny files.
- Split files when a single file mixes unrelated subsystems.
- Keep helper utilities in `tests/test_utils/` and reuse them.
- Keep scenario setup readable and deterministic.

## Cross References

- [golden_tests.md](golden_tests.md)
- [testing_best_practices.md](testing_best_practices.md)
- [test_organization.md](test_organization.md)
- `tests/ValidationTests/performance/collision_regression_test.cpp`
- `tools/performance_regression_check.py`


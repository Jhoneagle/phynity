# Testing Best Practices

This guide defines practical standards for writing and maintaining tests in Phynity.

## Goals

- Keep tests deterministic and actionable.
- Catch regressions early without creating flaky failures.
- Make failures easy to diagnose.
- Keep test code as maintainable as production code.

## Choosing the Right Test Type

### Unit Tests

Use unit tests when validating:

- Pure math correctness.
- Local algorithm behavior.
- Small API contracts.
- Edge cases that do not require full simulation loops.

Good traits:

- Fast.
- Isolated.
- Exact or tight tolerance assertions.

### Validation Tests

Use validation tests when validating:

- Multi-system interactions.
- Physical behavior over time.
- Robustness under stress.
- Finite/bounded state under adverse scenarios.

Good traits:

- Scenario-oriented setup.
- Clear invariants.
- Well-justified tolerances.

### Golden Tests

Use golden tests when validating:

- Stable deterministic outputs.
- Regression detection after refactor.
- End-to-end trajectory snapshots.

Good traits:

- Fixed seeds and deterministic setup.
- Intentional baseline update workflow.
- Human-reviewed golden diffs.

### Performance Regression Tests

Use performance regression tests when validating:

- Relative runtime drift against known baseline.
- Broad scenario throughput changes.

Good traits:

- Benchmark producer emits current metrics.
- Separate regression checker applies threshold.
- Avoid strict microsecond equality in test binaries.

## Test Design Principles

- One behavior per test case.
- Descriptive names that explain failure intent.
- Avoid hidden global state.
- Keep setup local when possible.
- Extract helper code only when reuse is clear.
- Prefer stable bounds over brittle exact values.

## Tolerance Guidelines

Use tolerances by test category, not by convenience.

### 1% (strict)

Use for:

- Deterministic math identities.
- Unit-level numerical algorithms with stable conditioning.
- Short-step expected-value checks with low accumulation error.

### 5% (default physics validation)

Use for:

- Typical physics behavior verification.
- Restitution/friction scenario checks.
- Constraint settling and convergence trend checks.

### 10% (robustness/long sequence)

Use for:

- Long multi-step simulations.
- Multi-contact chains and heavy interaction coupling.
- Cross-platform/toolchain-sensitive scenario checks.

### Rules for choosing tolerance

- Start with the smallest defensible tolerance.
- Increase only when justified by measured noise.
- Explain wider tolerances in comments near assertions.
- Prefer relative tolerance for scale-dependent values.
- Prefer absolute tolerance near zero-valued expectations.

## Golden Test Workflow

### Compare mode (normal)

- Run labeled golden tests.
- Fail on unexpected baseline drift.

### Capture mode (intentional update)

- Enable golden capture mode in CMake.
- Re-run only affected scenarios when possible.
- Review file diffs before commit.

### Review checklist for golden changes

- Is the change expected from intentional logic changes?
- Do all changed files map to touched subsystems?
- Are differences physically reasonable and stable?
- Were unrelated golden files kept unchanged?

## Performance Regression Workflow

Phynity uses a two-step approach:

1. Benchmark test writes `.current.json` results.
2. Python checker compares current vs baseline with threshold.

Why this pattern:

- Keeps benchmark measurement stable.
- Avoids flaky pass/fail inside benchmark binary.
- Makes threshold policy explicit and adjustable.

Recommended local flow:

1. Run `validation.performance.collision_regression`.
2. Run `python tools/performance_regression_check.py`.
3. If intentional change, recapture baselines and re-check.

## Writing Physics Scenarios

### Setup checklist

- Use explicit initial positions and velocities.
- Set deterministic seeds for pseudo-random generation.
- Keep dt explicit and consistent.
- Keep mass/radius/restition/friction values readable.
- Minimize unrelated forces in focused scenarios.

### Assertion checklist

- Always check finite values in stress tests.
- Assert physically meaningful bounds.
- Prefer trend checks in iterative solvers.
- Include both local and aggregate sanity assertions.

### Edge-case checklist

- Extreme coefficients (near 0 and high values).
- Mass ratio extremes.
- Long-run drift/stability.
- Multi-body chain propagation.
- Degenerate geometric cases.

## Commenting Standards in Test Files

Each new test case should include:

- 2 to 3 lines describing scenario intent.
- Why this scenario is high value.
- What property is being validated.

Each test section should include:

- A header block that labels the topic.
- Brief explanation of expected behavior.

## CMake and Naming Standards

- Keep executable names aligned with file purpose.
- Preserve stable ctest test names across reorganizations.
- Apply labels (`validation`, `golden`) consistently.
- Group related tests in subsystem subdirectories.

## Anti-Patterns to Avoid

- Overly broad tests covering many behaviors at once.
- Assertions that depend on debug print timing.
- Hidden random values without fixed seed.
- Re-baselining goldens without diff review.
- Tight tolerances copied from unrelated tests.
- Expanding scope to unrelated subsystems in one PR.

## Failure Triage Playbook

When a test fails:

1. Reproduce in isolation.
2. Check whether behavior changed intentionally.
3. Inspect recent code changes in affected subsystem.
4. Validate finite-state and invariants first.
5. Tighten or relax tolerance only with evidence.
6. For golden failures, inspect diff before recapture.
7. For performance failures, compare with repeated runs.

## PR Checklist for Test Changes

- Added tests in the correct folder.
- Test names describe behavior clearly.
- Tolerances are justified and documented.
- Labels and CMake entries are correct.
- New helper logic placed in `tests/test_utils/` when reusable.
- Golden and performance workflows updated if affected.

## Reference Files

- [golden_tests.md](golden_tests.md)
- [test_organization.md](test_organization.md)
- [../tests/test_utils/physics_test_helpers.hpp](../tests/test_utils/physics_test_helpers.hpp)
- [../tests/test_utils/golden_serializer.hpp](../tests/test_utils/golden_serializer.hpp)


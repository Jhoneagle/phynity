# Project Infrastructure Status

This file is the tracked repository mirror for CI/CD and infrastructure status.

The detailed planning file remains in `docs/project-infra.txt`, which is intentionally ignored by the repository. This mirror exists so the current implementation state is versioned without changing `.gitignore` or renaming the local planning document.

## Current Status

- Implemented workflows: `pr-gate.yml`, `nightly-quality.yml`, `perf-history-append.yml`
- PR gate coverage:
  - Windows/Linux/macOS build
  - unit test execution with bounded retries and retry reports
  - validation test execution with bounded retries and retry reports
  - clang-format verification
  - clang-tidy gating over the full `src` tree
  - Linux/macOS ASAN+UBSAN
  - non-blocking coverage report generation on Linux
- Nightly coverage:
  - Linux/macOS ASAN+UBSAN
  - Linux TSAN via CMake-native `debug-tsan` preset
  - golden/determinism validation
  - performance regression runs on Windows/Linux/macOS
  - append-only CSV trend history on `perf-history`

## Known Constraints

- Windows sanitizers remain disabled in current toolchains.
- Performance thresholds are currently configured per benchmark in `tests/golden_outputs/performance/thresholds.json`.
- The local planning file may contain additional notes ahead of this tracked mirror.

## Key Support Files

- Retry runner: `tools/run_ctest_with_retries.py`
- Performance regression checker: `tools/performance_regression_check.py`
- Perf history CSV appender: `tools/append_perf_history.py`
- Threshold config: `tests/golden_outputs/performance/thresholds.json`

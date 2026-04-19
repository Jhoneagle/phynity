# Project Infrastructure Status

This document mirrors the current CI/CD and infrastructure state for the Phynity project.

## CI/CD Workflows

### PR Gate (`pr-gate.yml`)
Triggers on pull requests to master/main/develop.

| Job | Platforms | Purpose |
|-----|-----------|---------|
| Build & Test | Windows, Linux, macOS | Compile + unit/validation tests |
| Format Check | Linux | clang-format 18.1.8 validation |
| Static Analysis | Linux | clang-tidy with warnings-as-errors |
| Coverage | Linux | gcovr report, 60% line threshold (non-blocking) |
| Sanitizers ASAN/UBSAN | Linux, macOS | Address + undefined behavior sanitizers |
| Sanitizers TSAN | Linux | Thread sanitizer |
| Flaky Budget Check | Linux | 14-day rolling window, threshold 3 (non-blocking) |
| Quality Gate Summary | Linux | Aggregates required checks |

### Nightly Quality (`nightly-quality.yml`)
Runs at 02:00 UTC daily.

| Job | Platforms | Purpose |
|-----|-----------|---------|
| ASAN+UBSAN | Windows, Linux, macOS | Extended sanitizer coverage |
| TSAN | Linux, macOS | Thread sanitizer (Windows excluded) |
| Determinism | Linux | Golden test suite + replay capture |
| Performance Regression | Windows, Linux, macOS | Benchmark + threshold comparison |
| Perf History Append | Linux | CSV history to perf-history branch |
| Flake Triage | Linux | Aggregate flaky test incidents |

### Performance Alert (`pr-performance-alert.yml`)
Triggers on PRs touching src/tests/tools/CMakeLists.

Runs performance-labeled tests and regression check (5% threshold) on all 3 platforms.

### Release (`release.yml`)
Triggers on `v*.*.*` tags.

Builds release-lto on all platforms, runs full test suite, creates GitHub release.

## Platform Matrix

| Platform | Runner | Compiler | Triplet |
|----------|--------|----------|---------|
| Windows | windows-2022 | clang-cl | x64-windows |
| Linux | ubuntu-22.04 | clang/clang++ | x64-linux |
| macOS | macos-14 | clang/clang++ | arm64-osx |

## Branch Protection

Required status checks (must match job names in `pr-gate.yml`):
- `Build & Test (windows)`
- `Build & Test (linux)`
- `Build & Test (macos)`
- `Format Check (clang-format)`
- `Static Analysis (clang-tidy)`
- `Sanitizers (linux)`
- `Sanitizers (macos)`
- `Sanitizers TSAN (linux)`
- `Quality Gate Summary`

## Performance History

- Stored on orphan `perf-history` branch
- CSV format with schema versioning
- HTML dashboard rendered by `tools/render_perf_dashboard.py`
- Flaky test history stored at `infra/history/flaky_test_history.csv`

## Known Gaps

- Coverage check is non-blocking (`continue-on-error: true`) - intended as informational
- No Windows TSAN support (LLVM limitation)
- macOS TSAN excluded from PR gate (reliability concerns), runs nightly only
- No automated coverage badge or PR comment integration
- Performance regression check in PR gate is separate from nightly history tracking

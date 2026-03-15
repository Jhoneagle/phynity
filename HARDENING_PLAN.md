# Roadmap Completion Checklist

This file tracks only the remaining work required to mark the three mid-term roadmap
items fully complete.

Legend:
- [ ] not started
- [~] in progress
- [x] done

---

## Track 1 - Data and Serialization (M)

### 1.1 Replace binary raw-struct casting with stable wire encoding

Problem:
- Binary serialization currently relies on raw struct casting in
	`src/core/serialization/snapshot_serializer.cpp`, which is sensitive to padding,
	alignment, and endianness differences.

Checklist:
- [x] Audit and remove `reinterpret_cast`-based writes/reads for snapshot payloads.
- [x] Implement explicit little-endian field-by-field encoding/decoding helpers.
- [x] Keep backward compatibility for existing binary format fixtures (v1 and v2 paths).
- [~] Re-run serialization validation tests on Windows, Linux, and macOS.

### 1.2 Implement allocator delta instrumentation

Problem:
- `src/platform/memory_usage.hpp` still returns `0` for `get_allocator_delta_bytes()`.

Checklist:
- [x] Add allocator instrumentation hooks (or equivalent allocation counters).
- [x] Return real per-run allocator delta instead of stub value.
- [x] Emit `allocator_delta_bytes` in performance output JSON.
- [x] Add tests validating non-zero delta under controlled allocations.

### 1.3 Finish serialization audit trail logging

Problem:
- `src/core/serialization/snapshot_helpers.cpp` contains TODO audit-log suppression.

Checklist:
- [x] Decide and document audit sink (diagnostics manager or dedicated log sink).
- [x] Replace TODO suppression with real logging call.
- [x] Add integration test asserting expected audit event is emitted.

### 1.4 Refresh stale implementation documentation

Problem:
- `PHASE3_IMPLEMENTATION.md` still lists already-completed serialization items as pending.

Checklist:
- [ ] Update all stale pending items to match current implemented state.
- [ ] Add a short dated summary of what is complete.
- [ ] Verify no other docs claim serialization stubs are still pending.

---

## Track 2 - Performance Monitoring (M)

### 2.1 Resolve unstable CV for `complex_deterministic_scene`

Problem:
- Current CV remains above configured threshold and triggers `CV_REGRESSION`.

Checklist:
- [ ] Choose one path: reduce benchmark variance or raise threshold with evidence.
- [ ] If reducing variance: isolate warmup/transients and stabilize measurement loop.
- [ ] If raising threshold: collect multi-run data and justify updated threshold.
- [ ] Update `tests/golden_outputs/performance/thresholds.json` accordingly.
- [ ] Verify nightly check no longer fails for this benchmark without masking real regressions.

### 2.2 Validate PR performance summary on all OS jobs

Problem:
- Per-OS summary output exists in workflow, but end-to-end verification is still pending.

Checklist:
- [~] Trigger a real PR/perf-alert run and confirm all three OS summaries appear.
- [ ] Fix summary write logic if any OS section is missing or malformed.

### 2.3 Complete memory trend pipeline with allocator delta

Blocked by Track 1 item 1.2.

Checklist:
- [ ] Confirm both `peak_rss_kb` and `allocator_delta_bytes` flow into `.current.json`.
- [ ] Confirm `tools/append_perf_history.py` stores memory fields in history rows.
- [ ] Confirm `tools/render_perf_dashboard.py` displays memory trend values.

---

## Track 3 - CI/CD Infrastructure (M)

### 3.1 Enforce branch protection in repository settings

Problem:
- Required checks are documented, but enforcement is a GitHub settings action not yet
	confirmed as active.

Checklist:
- [ ] Configure branch protection rule for `main` (or active default branch).
- [ ] Enable required status checks and up-to-date requirement.
- [ ] Disable bypass where policy requires it.
- [ ] Verify a failing required check blocks merge in a test PR.

### 3.2 Runtime-validate Windows ASAN/UBSAN nightly job

Problem:
- Workflow rows were added, but real CI runtime confirmation is pending.

Checklist:
- [ ] Run nightly sanitizer workflow including `windows-2022` row.
- [ ] Resolve any `clang-cl` sanitizer runtime/link issues discovered.
- [ ] Re-run until green or document a hard platform limitation with issue reference.

### 3.3 Runtime-validate release workflow on real tag

Problem:
- `release.yml` exists but has not been proven on a real `v*.*.*` tag push.

Checklist:
- [ ] Push a release-candidate tag and run full release workflow.
- [ ] Confirm all platform artifacts are generated and attached to GitHub Release.
- [ ] Fix packaging path/runtime issues and repeat until successful.

---

## Cross-Cutting Final Gate

Declare roadmap items fully complete only when all checks below pass:

- [ ] Build release test configuration successfully.
- [ ] Run unit + validation (non-performance) with retry harness and get zero failures.
- [ ] Run performance suite and get zero test failures.
- [ ] Run `tools/performance_regression_check.py` with strict orphan checking and get:
			zero `REGRESSION`, zero `MEMORY_REGRESSION`, zero `CV_REGRESSION`, zero
			`ORPHAN_CURRENT`.
- [ ] Confirm PR gate, nightly quality, and release workflows are all green in real CI.

---

## Completion Rule

All three roadmap tracks are "fully complete" only when:
- every checklist item in Tracks 1-3 is `[x]`, and
- the Cross-Cutting Final Gate is fully `[x]` in CI runtime (not file-only audit).

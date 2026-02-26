# Golden Test Outputs

This directory contains "golden files"—captured reference outputs from physics simulations that serve as regression test baselines.

## Structure

```
golden_outputs/
├── math/
│   ├── vectors/              # Vector operation outputs
│   ├── matrices/             # Matrix operation outputs
│   ├── quaternions/          # Quaternion operation outputs
│   ├── linear_algebra/       # Linear algebra solver outputs
│   └── calculus/             # Integrators and finite differences outputs
├── physics/
│   ├── particles/            # Particle system simulation trajectories
│   ├── integration/          # Multi-system integration scenes
│   ├── determinism/          # Deterministic replay captures
│   └── collision/            # Collision detection outputs
└── diagnostics/              # Energy/momentum/collision monitor outputs
```

## What Are Golden Files?

Golden files capture the exact output of a computation. When you run a golden test:

1. **Compare mode** (normal): Test runs simulation and compares output against golden file
2. **Capture mode** (regeneration): Test captures new output and saves it

## Workflow

### Running Tests (Compare Mode)

```bash
cmake ..
ctest -L golden
```

Tests compare actual output against golden files. Any difference triggers a test failure.

### Regenerating Goldens

When you intentionally change physics behavior (e.g., improve integrator), regenerate baselines:

```bash
cmake -DGOLDEN_CAPTURE_MODE=ON ..
ctest -L golden
git diff tests/golden_outputs/
```

Review the diffs to ensure changes are expected, then commit:

```bash
git add tests/golden_outputs/
```

## When to Update

✅ Update golden files when:
- Physics algorithm improves (better integrator, more stable)
- Bug fix changes numerical output
- Intentional behavior change
- Adding new golden test

❌ Do NOT update when:
- Code refactoring (same behavior)
- Optimizations (should produce same result)
- Logging/profiling changes
- Variable renames

## Format

Golden files are stored as JSON with 9-decimal precision for floating-point values:

```json
{
  "frame_number": 10,
  "elapsed_time": 0.160000000,
  "positions": [
    {"x": 1.234567890, "y": 0.000000000, "z": 0.000000000}
  ],
  "velocities": [
    {"x": 1.000000000, "y": -0.098000000, "z": 0.000000000}
  ],
  "masses": [1.0]
}
```

Trajectory files contain arrays of these snapshots for multi-frame captures.

## Debugging Failed Golden Tests

If a golden test fails:

1. Check the diff: `git diff tests/golden_outputs/expected.golden`
2. If change is unexpected, investigate the code
3. If change is desired, regenerate with capture mode
4. Verify diff carefully before committing

## Best Practices

- Commit golden files to version control
- Review diffs carefully before accepting changes
- Use determinism tests to catch subtle numerical changes
- Keep golden scenarios simple and focused
- Document what each golden file captures

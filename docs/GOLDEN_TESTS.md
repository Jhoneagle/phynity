# Golden Test Developer Guide

This guide explains how to write, run, and maintain golden tests for the Phynity physics engine.

## Quick Start

### Running Golden Tests

```bash
# Run all tests (including golden tests)
cmake ..
ctest

# Run only golden tests
ctest -L golden

# Regenerate golden baselines (after intentional physics changes)
cmake -DGOLDEN_CAPTURE_MODE=ON ..
ctest -L golden
git diff tests/golden_outputs/
git add tests/golden_outputs/
```

## What Are Golden Tests?

Golden tests capture the **exact output** of a physics simulation and compare subsequent runs against that snapshot. They detect:

- ✅ Unintended numerical regressions
- ✅ Determinism violations (same input → different output)
- ✅ Subtle changes from compiler flags or refactoring
- ❌ But NOT intentional physics improvements (golden files must update)

### Compare vs. Capture Mode

| Mode | Purpose | Command |
|------|---------|---------|
| **Compare** | Normal testing (default) | `ctest -L golden` |
| **Capture** | Create/update golden files | `cmake -DGOLDEN_CAPTURE_MODE=ON ..` |

## Writing a Golden Test

### 1. Basic Structure

```cpp
#include <catch2/catch_test_macros.hpp>
#include <tests/test_utils/golden_serializer.hpp>
#include <core/physics/micro/particle_system.hpp>

using namespace phynity::test;

TEST_CASE("My scenario - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");
    
    // Setup physics scenario
    ParticleSystem system;
    system.spawn({0, 0, 0}, {1, 0, 0}, 1.0f);
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0, -9.81f, 0)));
    
    // Simulate and capture
    std::vector<SerializedState> trajectory;
    for (int frame = 0; frame < 100; ++frame) {
        system.update(0.016f);
        trajectory.push_back(snapshot_system(system, frame + 1, (frame + 1) * 0.016f));
    }
    
#ifdef GOLDEN_CAPTURE_MODE
    // Capture mode: save golden baseline
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/my_scenario.golden"
    );
    SUCCEED("Golden captured");
#else
    // Compare mode: verify against baseline
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/my_scenario.golden"
    );
    
    // Assertions on trajectory...
    REQUIRE(trajectory.size() == 100);
#endif
}
```

### 2. Key Helpers

```cpp
// Snapshot current system state
SerializedState state = snapshot_system(
    system,
    frame_number,    // uint64_t
    elapsed_time     // float in seconds
);

// Compare two states with tolerance
bool matches = state1.equals(state2, tolerance /*1e-6f*/);

// Load and save golden files
auto json = GoldenSerializer::load_golden_file("path/to/file.golden");
GoldenSerializer::save_trajectory_golden(trajectory, "path/to/file.golden");
```

### 3. Directory Naming Convention

```
golden_outputs/
├── math/
│   ├── vectors/           # Vector operations (normalize, dot product, etc)
│   ├── matrices/          # Matrix operations
│   ├── quaternions/       # Quaternion operations
│   ├── linear_algebra/    # Linear algebra solver outputs
│   └── calculus/          # Integrators and finite differences outputs
├── physics/
│   ├── particles/         # Particle system trajectories
│   ├── integration/       # Multi-system integration scenes
│   ├── determinism/       # Determinism verification captures
│   └── collision/         # Broadphase/narrowphase captures
└── diagnostics/           # Energy/momentum/collision monitor outputs
```

Entity names: `{scenario}_{duration_or_count}.golden`
- Examples:
  - `gravity_simple_100frames.golden` (single particle under gravity)
  - `three_particle_600frames.golden` (multi-particle scenario)
  - `determinism_replay_1000frames.golden` (determinism check)

## Golden Test Workflow

### Creating a New Golden Test

```bash
# 1. Write test code in tests/ValidationTests/
# 2. CMakeLists.txt automatically picks it up
# 3. Capture baseline:
cmake -DGOLDEN_CAPTURE_MODE=ON -B build/debug
cd build/debug
ctest -R your_test_name

# 4. Golden files created in tests/golden_outputs/
# 5. Review files:
git diff tests/golden_outputs/

# 6. Commit to repository
git add tests/golden_outputs/
git commit -m "Add golden test: scenario name"
```

### After Intentional Physics Changes

```bash
# 1. Make physics changes (new integrator, bug fix, algorithm improvement)
# 2. Build normally to check compilation
cmake -B build/debug
cmake --build build/debug

# 3. Regenerate goldens
cmake -DGOLDEN_CAPTURE_MODE=ON -B build/debug_golden
cd build/debug_golden
ctest -L golden

# 4. Review what changed (CRITICAL STEP)
git diff tests/golden_outputs/

# 5. Verify changes are expected (e.g., "semi-implicit is more stable")
# 6. Commit new baselines
git add tests/golden_outputs/
git commit -m "Update golden tests after implementation of XYZ"
```

## Golden File Format

Files are JSON with 9-decimal precision for physics values:

### Single Frame Snapshot

```json
{
  "frame_number": 10,
  "elapsed_time": 0.160000000,
  "positions": [
    {"x": 1.234567890, "y": -0.150000000, "z": 0.000000000}
  ],
  "velocities": [
    {"x": 1.000000000, "y": -0.150000000, "z": 0.000000000}
  ],
  "masses": [1.0]
}
```

### Trajectory (Multiple Frames)

```json
{
  "trajectory": [
    { "frame_number": 1, "elapsed_time": 0.016, "positions": [...], ... },
    { "frame_number": 2, "elapsed_time": 0.032, "positions": [...], ... },
    ...
  ]
}
```

## Debugging Failed Golden Tests

### Test Fails on Expected Failure

```bash
ctest -L golden  # Test fails

# Option 1: Is this an intentional change?
git diff tests/golden_outputs/  # Review diff
# If yes, regenerate:
cmake -DGOLDEN_CAPTURE_MODE=ON ..
ctest -L golden
git add tests/golden_outputs/

# Option 2: Is this a regression?
# Investigate what changed in physics code
git log -p src/core/physics/  # Recent changes
```

### Test Fails on Unintended Regression

```bash
# Golden test caught a subtle regression!
# Example: Compiler flags changed numerical output

# 1. Check what changed
git diff tests/golden_outputs/particle_gravity.golden | head -30

# 2. Verify it's actually a problem (not just floating-point noise)
# Check if it violates physics invariants
grep -A2 "frame_number" tests/golden_outputs/particle_gravity.golden

# 3. If regression is real, fix the code
# If it's acceptable numerical difference, update guidelines

# 4. Once fixed, regenerate golden
cmake -DGOLDEN_CAPTURE_MODE=ON ..
ctest -L golden
git add tests/golden_outputs/
```

## Best Practices

### ✅ DO

- **Keep scenarios simple and focused**: One test = one physics feature
- **Document assumptions**: Comments in test code explaining setup
- **Review diffs carefully**: Always inspect golden changes before committing
- **Version control goldens**: Commit `.golden` files to git
- **Use tolerance wisely**: 
  - Determinism tests: 0.0f (exact match)
  - Physics tests: 1e-6f to 1e-4f depending on scenario
- **Tag tests**: Add `LABELS golden` so they're easy to filter

### ❌ DON'T

- **Update goldens without reviewing**: Automated regeneration without inspection
- **Use goldens for validation logic**: Validation tests check "properties", goldens check "regression"
- **Mix long & short scenarios**: Keep test duration consistent
- **Store binary-only**: Use JSON so diffs are readable
- **Ignore unexpected changes**: If golden changed unexpectedly, investigate

## Integration with CI

The CMake setup already integrates with CI:

```cmake
# Tests tagged with "golden" can be run separately
ctest -L golden
ctest -L ^golden  # everything except golden (fast smoke test)
```

For GitHub Actions, add to workflow:

```yaml
- name: Run all tests
  run: ctest

- name: Run smoke tests (exclude golden)
  run: ctest -L ^golden
```

## Common Scenarios

### Scenario: Single Particle Under Gravity

```cpp
TEST_CASE("Single particle gravity trajectory - golden") {
    // Initial: particle at origin, v=10 m/s upward
    // Expected: rises to ~5m, falls back down
    // Duration: 100 frames @ 16ms = 1.6s
}
```

### Scenario: Determinism Verification

```cpp
TEST_CASE("Three-particle determinism - golden") {
    // Run identical scenario twice
    // Compare frame-by-frame outputs (zero tolerance)
    // Ensures scheduling/ordering doesn't affect results
}
```

### Scenario: Field Composition

```cpp
TEST_CASE("Particles in gravity + drag - golden") {
    // Multiple particles with same drag preset
    // Mix of field types in one simulation
    // Duration: 60 frames (rapid convergence to terminal velocity)
}
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Golden file not found` | Ensure golden directory exists, check path in source |
| `Test passes in capture mode, fails in compare` | New golden files aren't committed; `git add golden_outputs/` |
| `Diff is huge but code change was small` | Floating-point accumulation over many frames; review if expected |
| `CMake can't find golden_serializer.hpp` | Check include paths in CMakeLists.txt match those in physics tests |
| `GOLDEN_CAPTURE_MODE undefined` | Rebuild cache: `rm -rf build/` then `cmake ..` |

## Future Enhancements

Potential improvements as the project scales:

- **Binary format**: Switch to Protobuf/FlatBuffers for large trajectory files
- **Visualization**: Frame-by-frame replay tool to compare goldens
- **Relative tolerance**: Per-component tolerance (position vs velocity vs acceleration)
- **Frame diffing**: Show which frames diverged first in regression
- **Archiving**: Golden file history/versioning

For now, JSON + git diffs provide excellent clarity for a growing physics engine.

---

**Quick Links:**
- [Golden Outputs Directory](../golden_outputs/)
- [Golden Serializer Header](../test_utils/golden_serializer.hpp)
- [Physics Test Helpers](../test_utils/physics_test_helpers.hpp)
- [Example: Particle Golden Test](../ValidationTests/physics/particles/particle_golden_test.cpp)
- [Example: Integration Scenes Golden Test](../ValidationTests/physics/particles/integration_scenes_golden_test.cpp)
- [Example: Math Golden Test](../ValidationTests/math/math_golden_test.cpp)
- [Example: Collision Golden Test](../ValidationTests/physics/collision/collision_golden_test.cpp)
- [Example: Diagnostics Golden Test](../ValidationTests/diagnostics/diagnostics_golden_test.cpp)

# Serialization Quick Start Guide

## Overview
The phynity serialization module enables:
- **Snapshot capture/restore**: Save and load physics simulation state
- **Golden testing**: Compare against expected baseline outputs
- **Determinism validation**: Verify reproducibility across platforms
- **Round-trip testing**: Save → Load → Compare workflow

## Module Location
- Headers: `src/core/serialization/`
- Implementation: `src/core/serialization/*.cpp`
- Tests: `tests/ValidationTests/serialization/`

## Core Concepts

### SchemaVersion
```cpp
SchemaVersion version{1, 0, 0};  // major.minor.patch
if (old_version.is_compatible_with(new_version)) {
    // Can safely load old snapshots
}
```

### Snapshot Types
- **ParticleSnapshot**: Single particle state (position, velocity, mass, etc.)
- **PhysicsSnapshot**: Complete system state (metadata + particle array)

### Serialization Formats
- **Binary** (.bin): Compact, optimized for storage and speed
- **JSON** (.json): Human-readable, easily debuggable

## Common Usage Patterns

### Pattern 1: Golden Testing
```cpp
#include <core/serialization/snapshot_helpers.hpp>

// In your test setup, capture golden
PhysicsSnapshot baseline = SnapshotHelpers::capture_particle_system(
    system, frame_count, sim_time, rng_seed
);
SnapshotHelpers::save_golden(baseline, "tests/golden_outputs/physics/drop_sphere.bin");

// In your test case, compare against golden
PhysicsSnapshot golden;
SnapshotHelpers::load_golden("tests/golden_outputs/physics/drop_sphere.bin", golden);

bool match = SnapshotHelpers::snapshots_equal_with_tolerance(golden, current);
REQUIRE(match);
```

### Pattern 2: Round-Trip Validation
```cpp
#include <core/serialization/snapshot_serializer.hpp>

// Create snapshot
PhysicsSnapshot original = create_test_snapshot();

// Save to file
auto save_result = SnapshotSerializer::save_binary(original, "temp.bin");
REQUIRE(save_result.is_success());

// Load back
PhysicsSnapshot loaded;
auto load_result = SnapshotSerializer::load_binary("temp.bin", loaded);
REQUIRE(load_result.is_success());

// Verify round-trip
bool round_trip_ok = SnapshotHelpers::snapshots_equal_with_tolerance(
    original, loaded
);
REQUIRE(round_trip_ok);
```

### Pattern 3: Determinism Check with Diagnostics
```cpp
std::string error_report;
bool deterministic = SnapshotHelpers::snapshots_equal_with_tolerance(
    expected,           // First run
    actual,             // Second run
    1e-6f,              // Position/velocity tolerance
    1e-4f,              // Impulse/force tolerance
    &error_report       // Optional: detailed mismatch info
);

if (!deterministic) {
    std::cerr << "Determinism violation:\n" << error_report << "\n";
    std::cerr << SnapshotHelpers::generate_diff_report(expected, actual);
}
```

### Pattern 4: Archival (Backup Both Formats)
```cpp
// Save both binary and JSON automatically
auto result = SnapshotSerializer::save_both(snapshot, "backup");
// Creates: backup.bin (primary) and backup.json (auxiliary)
```

## Tolerance Parameters

The serialization module uses two tolerance budgets matching the physics engine contract:

| Category | Tolerance | Use Case |
|----------|-----------|----------|
| Position/Velocity | 1e-6 | Particle position, velocity, acceleration |
| Impulse/Force | 1e-4 | Constraint forces, impulses, reactions |

These can be customized per comparison:
```cpp
// Stricter tolerance for tight determinism requirements
snapshots_equal_with_tolerance(s1, s2, 1e-7f, 1e-5f);

// Looser tolerance for numerical stability checks
snapshots_equal_with_tolerance(s1, s2, 1e-5f, 1e-3f);
```

## Error Handling

All serialization operations return structured results:

```cpp
struct SerializationResult {
    SerializationError error;      // Error code
    std::string error_message;     // Detailed message
    size_t bytes_processed;        // I/O stats
    
    bool is_success() const;       // Convenience check
};

// Usage
auto result = SnapshotSerializer::save_binary(snapshot, path);
if (!result.is_success()) {
    std::cerr << "Save failed: " << result.error_message << "\n";
    return false;
}
```

## Schema Version Compatibility

When loading snapshots:

```cpp
// Automatic compatibility checking
if (!snapshot.schema_version.is_compatible_with(current_schema)) {
    // Cannot load this snapshot - incompatible major version
    // Handle migration or error
}

// Or use helper
if (SnapshotSerializer::can_migrate(old_version, new_version)) {
    // Safe to migrate
}
```

**Compatibility Rules:**
- Same major version + equal/lower minor = compatible (additions only)
- Different major version = incompatible
- Migration required for major version bumps

## Integration with ParticleSystem

Current implementation supports ParticleSystem through:

```cpp
// Capture from live system
PhysicsSnapshot snapshot = SnapshotHelpers::capture_particle_system(
    system,              // ParticleSystem reference
    frame_42,            // Frame number for metadata
    42.5,                // Simulation time
    seed_12345           // RNG seed for determinism tracking
);

// Restore to clean system
ParticleSystem restored;
bool success = SnapshotHelpers::restore_particle_system(snapshot, restored);
```

**Note:** Requires public accessors or friend declarations on ParticleSystem for particle enumeration.

## Debugging Tips

### Inspect Snapshot Content
```cpp
std::cout << SnapshotSerializer::describe_snapshot(snapshot);
// Prints: schema version, frame count, particle count, size estimate
```

### Generate Detailed Diff
```cpp
std::string diff = SnapshotHelpers::generate_diff_report(expected, actual);
std::cout << diff;  // Shows particle-by-particle differences
```

### Validate Golden Files
```cpp
bool valid = SnapshotHelpers::validate_golden_exists(
    "tests/golden_outputs/scene.bin",
    {1, 0, 0}  // Expected schema version
);
```

## File Format Guide

### Binary Format
- Best for: Performance, storage efficiency, CI artifacts
- Extension: `.bin`
- Header includes magic number (0x5F435953) and version validation
- POD struct layout for fast I/O

### JSON Format
```json
{
  "schema_version": {
    "major": 1,
    "minor": 0,
    "patch": 0
  },
  "metadata": {
    "frame_number": 42,
    "simulated_time": 0.5,
    "timestep": 0.016667,
    "rng_seed": 12345
  },
  "particles": [
    {
      "position": [1.0, 2.0, 3.0],
      "velocity": [0.5, -1.0, 1.5],
      "mass": 2.0,
      "radius": 0.5,
      "active": true
    },
    ...
  ]
}
```

## Next Steps for Integration

1. **Extend to RigidBodySystem**: Add `RigidBodySnapshot` struct and capture/restore methods
2. **Generate golden files**: Use capture mode in test suite to create baselines
3. **Determinism CI job**: Integrate snapshot comparison into nightly workflow
4. **JSON loading**: Implement `load_json()` for flexible golden file format
5. **Migration utilities**: Add code generation for schema migrations

## See Also
- [Phase 3 Implementation](PHASE3_IMPLEMENTATION.md) - Detailed technical spec
- [Project Infrastructure](docs/project-infra.txt) - Serialization contract and policy
- [Golden Testing](docs/golden_tests.md) - Golden test workflow
- [Testing Best Practices](docs/testing_best_practices.md) - Test organization

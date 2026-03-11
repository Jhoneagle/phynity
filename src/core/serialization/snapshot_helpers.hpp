#pragma once

#include <core/serialization/replay_writer.hpp>
#include <core/serialization/snapshot_schema.hpp>
#include <core/serialization/snapshot_serializer.hpp>

namespace phynity::physics
{
class ParticleSystem;
class RigidBodySystem;
} // namespace phynity::physics

namespace phynity::serialization
{

/// Utilities for capturing and restoring physics simulation snapshots
/// Used by golden tests and replay validation
class SnapshotHelpers
{
public:
    // ========================================================================
    // Replay Capture: Multi-frame snapshot stream
    // ========================================================================

    /// Start writing replay frames to path.
    static SerializationResult start_replay_capture(const std::string &replay_file);

    /// Append an already captured snapshot frame into active replay capture.
    static SerializationResult append_replay_frame(const PhysicsSnapshot &snapshot);

    /// Capture and append particle-system state in one call.
    static SerializationResult append_replay_frame(const phynity::physics::ParticleSystem &system,
                                                   uint64_t frame_number,
                                                   double simulated_time,
                                                   uint32_t rng_seed);

    /// Capture and append rigid-body-system state in one call.
    static SerializationResult append_replay_frame(const phynity::physics::RigidBodySystem &system,
                                                   uint64_t frame_number,
                                                   double simulated_time,
                                                   uint32_t rng_seed);

    /// Finish replay capture and flush file to disk.
    static SerializationResult stop_replay_capture();

    // ========================================================================
    // Snapshot Capture: Live System -> Snapshot
    // ========================================================================

    /// Capture current state of a ParticleSystem into a snapshot
    /// @param system Active ParticleSystem to snapshot
    /// @param frame_number Frame count for metadata
    /// @param simulated_time Simulation clock for metadata
    /// @param rng_seed RNG seed used by system (for round-trip verification)
    /// @return PhysicsSnapshot with current system state
    static PhysicsSnapshot capture_particle_system(const phynity::physics::ParticleSystem &system,
                                                   uint64_t frame_number,
                                                   double simulated_time,
                                                   uint32_t rng_seed);

    /// Capture current state of a RigidBodySystem into a snapshot
    static PhysicsSnapshot capture_rigid_body_system(const phynity::physics::RigidBodySystem &system,
                                                     uint64_t frame_number,
                                                     double simulated_time,
                                                     uint32_t rng_seed);

    // ========================================================================
    // Snapshot Restoration: Snapshot -> Live System
    // ========================================================================

    /// Restore a ParticleSystem from a snapshot
    /// Clears existing particles and restores from snapshot state
    /// @param snapshot Source snapshot
    /// @param system Target ParticleSystem to restore (will be cleared first)
    /// @return true if restoration successful
    static bool restore_particle_system(const PhysicsSnapshot &snapshot, phynity::physics::ParticleSystem &system);

    /// Restore a RigidBodySystem from snapshot rigid body entries.
    static bool restore_rigid_body_system(const PhysicsSnapshot &snapshot, phynity::physics::RigidBodySystem &system);

    // ========================================================================
    // Comparison: Tolerance-based snapshot validation
    // ========================================================================

    /// Compare two snapshots with floating-point tolerance
    /// Used for determinism and replay validation
    /// @param expected Golden baseline snapshot
    /// @param actual Current simulation result
    /// @param position_tolerance Allowed difference in position/velocity (default: 1e-6)
    /// @param impulse_tolerance Allowed difference in impulse/force (default: 1e-4)
    /// @param error_report Output: detailed mismatch description if non-null
    /// @return true if snapshots match within tolerance
    static bool snapshots_equal_with_tolerance(const PhysicsSnapshot &expected,
                                               const PhysicsSnapshot &actual,
                                               float position_tolerance = 1e-6f,
                                               float impulse_tolerance = 1e-4f,
                                               std::string *error_report = nullptr);

    // ========================================================================
    // Golden File Management: Load/save golden baselines
    // ========================================================================

    /// Load golden baseline for a given scene/test
    /// @param golden_file Path to golden snapshot (binary or JSON)
    /// @param snapshot Output snapshot
    /// @return SerializationResult indicating success/failure
    static SerializationResult load_golden(const std::string &golden_file, PhysicsSnapshot &snapshot);

    /// Save new golden baseline after validating determinism
    /// Creates backup of existing golden as file.golden.bak if it exists
    /// @param snapshot Snapshot to save as new baseline
    /// @param golden_file Path to output golden file
    /// @return SerializationResult indicating success/failure
    static SerializationResult save_golden(const PhysicsSnapshot &snapshot, const std::string &golden_file);

    /// Update golden baseline during test regeneration
    /// Explicitly declares intent to overwrite golden file for approved changes
    /// Used with --update-golden test flag
    /// @param snapshot New snapshot to save
    /// @param golden_file Path to golden file
    /// @param log_message Human-readable reason for update
    /// @return SerializationResult
    static SerializationResult
    update_golden(const PhysicsSnapshot &snapshot, const std::string &golden_file, const std::string &log_message);

    // ========================================================================
    // Debugging and Diagnostics
    // ========================================================================

    /// Generate detailed diff report between two snapshots
    /// @param expected Golden snapshot
    /// @param actual Current snapshot
    /// @return Multi-line string with particle-by-particle diff
    static std::string generate_diff_report(const PhysicsSnapshot &expected, const PhysicsSnapshot &actual);

    /// Validate golden file exists and is readable
    /// @param golden_file Path to golden file
    /// @param schema_version Expected schema version
    /// @return true if file exists, readable, and schema version compatible
    static bool validate_golden_exists(const std::string &golden_file, const SchemaVersion &schema_version);
};

} // namespace phynity::serialization

#pragma once

#include <core/serialization/snapshot_schema.hpp>

#include <string>
#include <vector>

namespace phynity::serialization
{

/// Errors that can occur during serialization/deserialization
enum class SerializationError
{
    Success = 0,
    FileNotFound,
    InvalidFileFormat,
    SchemaVersionMismatch,
    CorruptedData,
    WriteError,
    IOError,
    JsonError,
    Unknown
};

/// Contains error details
struct SerializationResult
{
    SerializationError error = SerializationError::Success;
    std::string error_message;
    size_t bytes_processed = 0;

    bool is_success() const
    {
        return error == SerializationError::Success;
    }
    std::string to_string() const;
};

/// Binary and JSON serialization for PhysicsSnapshot
class SnapshotSerializer
{
public:
    // ========================================================================
    // Binary Format: Optimized for storage and round-trip accuracy
    // ========================================================================

    /// Save snapshot to binary file
    /// Format: [SnapshotFileHeader][metadata][ParticleSnapshot*num_particles][RigidBodySnapshot*num_rigid_bodies]
    /// @param snapshot Physics state to save
    /// @param file_path Output file path
    /// @return SaveResult with success status and bytes written
    static SerializationResult save_binary(const PhysicsSnapshot &snapshot, const std::string &file_path);

    /// Load snapshot from binary file
    /// @param file_path Input binary file
    /// @param snapshot Output snapshot (populated if successful)
    /// @return LoadResult with success status and error message if failed
    static SerializationResult load_binary(const std::string &file_path, PhysicsSnapshot &snapshot);

    // ========================================================================
    // JSON Format: Human-readable, debuggable
    // ========================================================================

    /// Save snapshot as pretty-printed JSON
    /// @param snapshot Physics state to save
    /// @param file_path Output file path
    /// @param pretty_print If true, indent/format for readability
    /// @return SerializationResult
    static SerializationResult
    save_json(const PhysicsSnapshot &snapshot, const std::string &file_path, bool pretty_print = true);

    /// Load snapshot from JSON file
    /// @param file_path Input JSON file
    /// @param snapshot Output snapshot (populated if successful)
    /// @return SerializationResult
    static SerializationResult load_json(const std::string &file_path, PhysicsSnapshot &snapshot);

    // ========================================================================
    // Archival: Save both binary + JSON in a single operation
    // ========================================================================

    /// Save snapshot in both binary and JSON formats for redundancy
    /// Creates: file_path.bin and file_path.json
    /// @param snapshot Physics state to save
    /// @param file_path_base Base path (extensions .bin and .json will be added)
    /// @return SerializationResult with status; only the binary write error is reported
    static SerializationResult save_both(const PhysicsSnapshot &snapshot, const std::string &file_path_base);

    // ========================================================================
    // Utility: Schema migration and validation
    // ========================================================================

    /// Check if legacy version can be loaded by current schema
    /// @param legacy_version Version from file
    /// @param current_version Current schema version
    /// @return true if legacy can be converted to current
    static bool can_migrate(const SchemaVersion &legacy_version, const SchemaVersion &current_version);

    /// Get human-readable description of snapshot for debugging
    /// @param snapshot Snapshot to describe
    /// @return Multi-line string describing contents
    static std::string describe_snapshot(const PhysicsSnapshot &snapshot);
};

} // namespace phynity::serialization

#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/physics/common/material.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace phynity::serialization
{

using phynity::math::vectors::Vec3f;

enum class SnapshotShapeType : uint32_t
{
    Unknown = 0,
    Sphere = 1,
    Box = 2,
    Capsule = 3,
};

/// Semantic versioning for snapshot schema
/// Bump major on breaking changes, minor on additions
struct SchemaVersion
{
    uint32_t major = 1;
    uint32_t minor = 0;
    uint32_t patch = 0;

    bool operator==(const SchemaVersion &other) const = default;
    bool operator!=(const SchemaVersion &other) const = default;

    /// Check if this version is forward-compatible with target version
    /// (target can load snapshots from this version)
    bool is_compatible_with(const SchemaVersion &target) const
    {
        // Major version mismatch = incompatible
        if (major != target.major)
            return false;
        // Same major, this.minor <= target.minor = compatible (additions only)
        return minor <= target.minor;
    }

    /// Get version as string for diagnostics
    std::string to_string() const;
};

constexpr SchemaVersion current_schema_version()
{
    return SchemaVersion{1, 1, 0};
}

/// Snapshot of a single particle's state (suitable for serialization)
struct ParticleSnapshot
{
    Vec3f position{0.0f};
    Vec3f velocity{0.0f};
    Vec3f acceleration{0.0f};
    Vec3f force_accumulator{0.0f};
    float radius = 0.5f;

    // Material
    float mass = 1.0f;
    float restitution = 0.1f;
    float friction = 0.5f;
    float linear_damping = 0.01f;
    float angular_damping = 0.01f;
    float drag_coefficient = 0.0f;

    // Lifecycle
    float lifetime = -1.0f;
    bool active = true;

    /// Compare with tolerance for floating-point
    /// @param tolerance Maximum allowed difference (e.g., 1e-6 for position)
    bool equals_with_tolerance(const ParticleSnapshot &other, float tolerance = 1e-6f) const;
};

/// Snapshot of a single rigid body's state.
struct RigidBodySnapshot
{
    Vec3f position{0.0f};
    Vec3f velocity{0.0f};
    Vec3f force_accumulator{0.0f};

    float orientation_w = 1.0f;
    float orientation_x = 0.0f;
    float orientation_y = 0.0f;
    float orientation_z = 0.0f;

    Vec3f angular_velocity{0.0f};
    Vec3f torque_accumulator{0.0f};

    SnapshotShapeType shape_type = SnapshotShapeType::Unknown;
    Vec3f shape_local_center{0.0f};
    float shape_radius = 0.5f;
    Vec3f shape_half_extents{0.5f};
    float shape_half_height = 0.5f;

    float collision_radius = 0.5f;

    float mass = 1.0f;
    float restitution = 0.1f;
    float friction = 0.5f;
    float linear_damping = 0.01f;
    float angular_damping = 0.01f;
    float drag_coefficient = 0.0f;

    bool active = true;
    int id = -1;
    float lifetime = -1.0f;
};

/// Complete snapshot of ParticleSystem state
/// Includes all active particles, RNG seed, and simulation metadata
struct PhysicsSnapshot
{
    SchemaVersion schema_version = current_schema_version();

    // Metadata
    uint64_t frame_number = 0;
    double simulated_time = 0.0;
    float timestep = 1.0f / 60.0f;

    // RNG state (for determinism)
    uint32_t rng_seed = 42;

    // Particle system state
    std::vector<ParticleSnapshot> particles;

    // Rigid body system state
    std::vector<RigidBodySnapshot> rigid_bodies;

    // Future: Rigid body snapshots, constraints, etc.

    /// Total size of snapshot in bytes (for diagnostics)
    size_t serialized_size() const;

    /// Verify snapshot integrity before round-trip test
    bool is_valid() const;
};

/// Snapshot header written before all snapshots (binary format)
struct SnapshotFileHeader
{
    static constexpr uint32_t MAGIC_NUMBER = 0x5F435953; // "_CYS" in hex (Phynity Snapshot)
    static constexpr uint32_t FORMAT_VERSION = 2; // File format, distinct from schema version

    uint32_t magic = MAGIC_NUMBER;
    uint32_t format_version = FORMAT_VERSION;
    uint32_t schema_major = 1;
    uint32_t schema_minor = 1;
    uint32_t schema_patch = 0;
    uint32_t num_particles = 0;
    uint32_t num_rigid_bodies = 0;
    uint32_t metadata_json_bytes = 0;
    uint64_t file_size = 0; // Total size including this header
};

} // namespace phynity::serialization

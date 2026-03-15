#include <core/serialization/snapshot_migration.hpp>

#include <array>

namespace
{

using phynity::serialization::PhysicsSnapshot;
using phynity::serialization::SchemaVersion;
using phynity::serialization::SerializationError;
using phynity::serialization::SerializationResult;

constexpr SchemaVersion SCHEMA_V0_9_0{0, 9, 0};
constexpr SchemaVersion SCHEMA_V1_0_0{1, 0, 0};
constexpr SchemaVersion SCHEMA_V1_1_0{1, 1, 0};

using MigrationFn = void (*)(PhysicsSnapshot &snapshot);

struct MigrationStep
{
    SchemaVersion from_version;
    SchemaVersion to_version;
    const char *name;
    MigrationFn apply;
};

void migrate_v0_9_to_v1_0(PhysicsSnapshot &snapshot)
{
    // Legacy 0.9 snapshots relied on parser defaults for fields added in 1.0.
    snapshot.schema_version = SCHEMA_V1_0_0;
}

void migrate_v1_0_to_v1_1(PhysicsSnapshot &snapshot)
{
    // 1.1 formalized rigid body snapshot support; legacy particle snapshots carry forward unchanged.
    snapshot.schema_version = SCHEMA_V1_1_0;
}

constexpr std::array<MigrationStep, 2> MIGRATION_STEPS{{
    {SCHEMA_V0_9_0, SCHEMA_V1_0_0, "migrate_v0_9_to_v1_0", &migrate_v0_9_to_v1_0},
    {SCHEMA_V1_0_0, SCHEMA_V1_1_0, "migrate_v1_0_to_v1_1", &migrate_v1_0_to_v1_1},
}};

const MigrationStep *find_migration_step(const SchemaVersion &from_version)
{
    for (const auto &step : MIGRATION_STEPS)
    {
        if (step.from_version == from_version)
        {
            return &step;
        }
    }
    return nullptr;
}

SerializationResult build_missing_path_error(const SchemaVersion &from_version, const SchemaVersion &target_version)
{
    SerializationResult result;
    result.error = SerializationError::SchemaVersionMismatch;
    result.error_message =
        "No migration path from schema " + from_version.to_string() + " to " + target_version.to_string();
    return result;
}

} // namespace

namespace phynity::serialization
{

bool can_migrate_snapshot_schema(const SchemaVersion &from_version, const SchemaVersion &to_version)
{
    if (from_version == to_version)
    {
        return true;
    }

    SchemaVersion current = from_version;
    for (size_t hop = 0; hop < MIGRATION_STEPS.size(); ++hop)
    {
        const MigrationStep *step = find_migration_step(current);
        if (step == nullptr)
        {
            return false;
        }

        current = step->to_version;
        if (current == to_version)
        {
            return true;
        }
    }

    return false;
}

SerializationResult migrate_snapshot_to_schema(PhysicsSnapshot &snapshot, const SchemaVersion &target_version)
{
    if (snapshot.schema_version == target_version)
    {
        return SerializationResult{};
    }

    if (!can_migrate_snapshot_schema(snapshot.schema_version, target_version))
    {
        return build_missing_path_error(snapshot.schema_version, target_version);
    }

    SerializationResult result;
    for (size_t hop = 0; hop < MIGRATION_STEPS.size() && snapshot.schema_version != target_version; ++hop)
    {
        const MigrationStep *step = find_migration_step(snapshot.schema_version);
        if (step == nullptr)
        {
            return build_missing_path_error(snapshot.schema_version, target_version);
        }

        step->apply(snapshot);
        result.bytes_processed += 1;
    }

    if (snapshot.schema_version != target_version)
    {
        return build_missing_path_error(snapshot.schema_version, target_version);
    }

    result.error = SerializationError::Success;
    return result;
}

} // namespace phynity::serialization
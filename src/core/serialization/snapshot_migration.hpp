#pragma once

#include <core/serialization/snapshot_serializer.hpp>

namespace phynity::serialization
{

bool can_migrate_snapshot_schema(const SchemaVersion &from_version, const SchemaVersion &to_version);

SerializationResult migrate_snapshot_to_schema(PhysicsSnapshot &snapshot, const SchemaVersion &target_version);

} // namespace phynity::serialization
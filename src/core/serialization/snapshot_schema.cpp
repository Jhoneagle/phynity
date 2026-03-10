#include <core/serialization/snapshot_schema.hpp>

#include <sstream>

namespace phynity::serialization
{

std::string SchemaVersion::to_string() const
{
    std::ostringstream oss;
    oss << major << "." << minor << "." << patch;
    return oss.str();
}

bool ParticleSnapshot::equals_with_tolerance(const ParticleSnapshot &other, float tolerance) const
{
    auto vec_equal = [tolerance](const Vec3f &a, const Vec3f &b) { return (a - b).length() <= tolerance; };

    return vec_equal(position, other.position) && vec_equal(velocity, other.velocity) &&
           vec_equal(acceleration, other.acceleration) && vec_equal(force_accumulator, other.force_accumulator) &&
           std::abs(radius - other.radius) <= tolerance && std::abs(mass - other.mass) <= tolerance &&
           std::abs(restitution - other.restitution) <= tolerance && std::abs(friction - other.friction) <= tolerance &&
           std::abs(linear_damping - other.linear_damping) <= tolerance &&
           std::abs(angular_damping - other.angular_damping) <= tolerance &&
           std::abs(drag_coefficient - other.drag_coefficient) <= tolerance &&
           std::abs(lifetime - other.lifetime) <= tolerance && active == other.active;
}

size_t PhysicsSnapshot::serialized_size() const
{
    // Header + particles + rigid bodies + metadata
    size_t size = sizeof(SnapshotFileHeader);
    size += particles.size() * sizeof(ParticleSnapshot);
    size += rigid_bodies.size() * sizeof(RigidBodySnapshot);
    // Add overhead for schema version and metadata
    size += 256; // Conservative estimate for metadata overhead
    return size;
}

bool PhysicsSnapshot::is_valid() const
{
    // Basic validity checks
    if (schema_version.major == 0)
        return false;
    if (particles.empty() && rigid_bodies.empty() && frame_number > 0)
        return false; // Non-zero frame but no simulation entities is suspicious
    return true;
}

} // namespace phynity::serialization

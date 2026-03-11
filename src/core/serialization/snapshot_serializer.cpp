#include <core/serialization/snapshot_migration.hpp>
#include <core/serialization/snapshot_serializer.hpp>

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace
{

using phynity::math::vectors::Vec3f;
using phynity::serialization::ParticleSnapshot;
using phynity::serialization::PhysicsSnapshot;
using phynity::serialization::RigidBodySnapshot;
using phynity::serialization::SchemaVersion;
using phynity::serialization::SerializationError;
using phynity::serialization::SerializationResult;
using phynity::serialization::SnapshotFileHeader;
using phynity::serialization::SnapshotShapeType;

struct SnapshotFileHeaderV1
{
    uint32_t magic = SnapshotFileHeader::MAGIC_NUMBER;
    uint32_t format_version = 1;
    uint32_t schema_major = 1;
    uint32_t schema_minor = 0;
    uint32_t schema_patch = 0;
    uint32_t num_particles = 0;
    uint64_t file_size = 0;
};

class JsonCursor
{
public:
    explicit JsonCursor(const std::string &input) : input_(input)
    {
    }

    void skip_whitespace()
    {
        while (pos_ < input_.size() && std::isspace(static_cast<unsigned char>(input_[pos_])) != 0)
        {
            ++pos_;
        }
    }

    bool eof()
    {
        skip_whitespace();
        return pos_ >= input_.size();
    }

    bool consume(char expected)
    {
        skip_whitespace();
        if (pos_ < input_.size() && input_[pos_] == expected)
        {
            ++pos_;
            return true;
        }
        return false;
    }

    bool peek(char expected)
    {
        skip_whitespace();
        return pos_ < input_.size() && input_[pos_] == expected;
    }

    bool consume_literal(const char *literal)
    {
        skip_whitespace();
        const size_t length = std::strlen(literal);
        if (input_.compare(pos_, length, literal) == 0)
        {
            pos_ += length;
            return true;
        }
        return false;
    }

    bool expect(char expected, SerializationResult &result, const char *message)
    {
        if (!consume(expected))
        {
            result.error = SerializationError::JsonError;
            result.error_message = message;
            return false;
        }
        return true;
    }

    std::string parse_string(SerializationResult &result)
    {
        skip_whitespace();
        if (pos_ >= input_.size() || input_[pos_] != '"')
        {
            result.error = SerializationError::JsonError;
            result.error_message = "Expected JSON string";
            return {};
        }

        ++pos_;
        std::string parsed;
        while (pos_ < input_.size())
        {
            const char current = input_[pos_++];
            if (current == '"')
            {
                return parsed;
            }

            if (current == '\\')
            {
                if (pos_ >= input_.size())
                {
                    break;
                }

                const char escaped = input_[pos_++];
                switch (escaped)
                {
                    case '"':
                    case '\\':
                    case '/':
                        parsed.push_back(escaped);
                        break;
                    case 'b':
                        parsed.push_back('\b');
                        break;
                    case 'f':
                        parsed.push_back('\f');
                        break;
                    case 'n':
                        parsed.push_back('\n');
                        break;
                    case 'r':
                        parsed.push_back('\r');
                        break;
                    case 't':
                        parsed.push_back('\t');
                        break;
                    default:
                        result.error = SerializationError::JsonError;
                        result.error_message = "Unsupported JSON escape sequence";
                        return {};
                }
                continue;
            }

            parsed.push_back(current);
        }

        result.error = SerializationError::JsonError;
        result.error_message = "Unterminated JSON string";
        return {};
    }

    double parse_number(SerializationResult &result)
    {
        skip_whitespace();
        const char *begin = input_.c_str() + pos_;
        char *end = nullptr;
        const double value = std::strtod(begin, &end);
        if (begin == end)
        {
            result.error = SerializationError::JsonError;
            result.error_message = "Expected JSON number";
            return 0.0;
        }

        pos_ = static_cast<size_t>(end - input_.c_str());
        return value;
    }

    bool parse_bool(SerializationResult &result)
    {
        if (consume_literal("true"))
        {
            return true;
        }
        if (consume_literal("false"))
        {
            return false;
        }

        result.error = SerializationError::JsonError;
        result.error_message = "Expected JSON boolean";
        return false;
    }

    void skip_value(SerializationResult &result)
    {
        skip_whitespace();
        if (pos_ >= input_.size())
        {
            result.error = SerializationError::JsonError;
            result.error_message = "Unexpected end of JSON value";
            return;
        }

        if (consume('{'))
        {
            if (consume('}'))
            {
                return;
            }

            do
            {
                parse_string(result);
                if (!result.is_success() || !expect(':', result, "Expected ':' after object key"))
                {
                    return;
                }
                skip_value(result);
                if (!result.is_success())
                {
                    return;
                }
            } while (consume(','));

            expect('}', result, "Expected '}' at end of object");
            return;
        }

        if (consume('['))
        {
            if (consume(']'))
            {
                return;
            }

            do
            {
                skip_value(result);
                if (!result.is_success())
                {
                    return;
                }
            } while (consume(','));

            expect(']', result, "Expected ']' at end of array");
            return;
        }

        if (peek('"'))
        {
            parse_string(result);
            return;
        }

        if (consume_literal("true") || consume_literal("false") || consume_literal("null"))
        {
            return;
        }

        parse_number(result);
    }

private:
    const std::string &input_;
    size_t pos_ = 0;
};

bool parse_vec3(JsonCursor &cursor, Vec3f &value, SerializationResult &result)
{
    if (!cursor.expect('[', result, "Expected '[' for Vec3f"))
    {
        return false;
    }

    value.x = static_cast<float>(cursor.parse_number(result));
    if (!result.is_success() || !cursor.expect(',', result, "Expected ',' after x component"))
    {
        return false;
    }

    value.y = static_cast<float>(cursor.parse_number(result));
    if (!result.is_success() || !cursor.expect(',', result, "Expected ',' after y component"))
    {
        return false;
    }

    value.z = static_cast<float>(cursor.parse_number(result));
    if (!result.is_success() || !cursor.expect(']', result, "Expected ']' after z component"))
    {
        return false;
    }

    return true;
}

bool parse_schema_version(JsonCursor &cursor, SchemaVersion &version, SerializationResult &result)
{
    if (!cursor.expect('{', result, "Expected '{' for schema_version"))
    {
        return false;
    }

    if (cursor.consume('}'))
    {
        return true;
    }

    do
    {
        const std::string key = cursor.parse_string(result);
        if (!result.is_success() || !cursor.expect(':', result, "Expected ':' after schema_version key"))
        {
            return false;
        }

        if (key == "major")
        {
            version.major = static_cast<uint32_t>(cursor.parse_number(result));
        }
        else if (key == "minor")
        {
            version.minor = static_cast<uint32_t>(cursor.parse_number(result));
        }
        else if (key == "patch")
        {
            version.patch = static_cast<uint32_t>(cursor.parse_number(result));
        }
        else
        {
            cursor.skip_value(result);
        }

        if (!result.is_success())
        {
            return false;
        }
    } while (cursor.consume(','));

    return cursor.expect('}', result, "Expected '}' after schema_version");
}

bool parse_metadata(JsonCursor &cursor, PhysicsSnapshot &snapshot, SerializationResult &result)
{
    if (!cursor.expect('{', result, "Expected '{' for metadata"))
    {
        return false;
    }

    if (cursor.consume('}'))
    {
        return true;
    }

    do
    {
        const std::string key = cursor.parse_string(result);
        if (!result.is_success() || !cursor.expect(':', result, "Expected ':' after metadata key"))
        {
            return false;
        }

        if (key == "frame_number")
        {
            snapshot.frame_number = static_cast<uint64_t>(cursor.parse_number(result));
        }
        else if (key == "simulated_time")
        {
            snapshot.simulated_time = cursor.parse_number(result);
        }
        else if (key == "timestep")
        {
            snapshot.timestep = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "rng_seed")
        {
            snapshot.rng_seed = static_cast<uint32_t>(cursor.parse_number(result));
        }
        else
        {
            cursor.skip_value(result);
        }

        if (!result.is_success())
        {
            return false;
        }
    } while (cursor.consume(','));

    return cursor.expect('}', result, "Expected '}' after metadata");
}

bool parse_particle(JsonCursor &cursor, ParticleSnapshot &particle, SerializationResult &result)
{
    if (!cursor.expect('{', result, "Expected '{' for particle"))
    {
        return false;
    }

    if (cursor.consume('}'))
    {
        return true;
    }

    do
    {
        const std::string key = cursor.parse_string(result);
        if (!result.is_success() || !cursor.expect(':', result, "Expected ':' after particle key"))
        {
            return false;
        }

        if (key == "position")
        {
            if (!parse_vec3(cursor, particle.position, result))
            {
                return false;
            }
        }
        else if (key == "velocity")
        {
            if (!parse_vec3(cursor, particle.velocity, result))
            {
                return false;
            }
        }
        else if (key == "acceleration")
        {
            if (!parse_vec3(cursor, particle.acceleration, result))
            {
                return false;
            }
        }
        else if (key == "force_accumulator")
        {
            if (!parse_vec3(cursor, particle.force_accumulator, result))
            {
                return false;
            }
        }
        else if (key == "radius")
        {
            particle.radius = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "mass")
        {
            particle.mass = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "restitution")
        {
            particle.restitution = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "friction")
        {
            particle.friction = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "linear_damping")
        {
            particle.linear_damping = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "angular_damping")
        {
            particle.angular_damping = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "drag_coefficient")
        {
            particle.drag_coefficient = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "lifetime")
        {
            particle.lifetime = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "active")
        {
            particle.active = cursor.parse_bool(result);
        }
        else
        {
            cursor.skip_value(result);
        }

        if (!result.is_success())
        {
            return false;
        }
    } while (cursor.consume(','));

    return cursor.expect('}', result, "Expected '}' after particle object");
}

bool parse_particles(JsonCursor &cursor, PhysicsSnapshot &snapshot, SerializationResult &result)
{
    if (!cursor.expect('[', result, "Expected '[' for particles"))
    {
        return false;
    }

    snapshot.particles.clear();
    if (cursor.consume(']'))
    {
        return true;
    }

    do
    {
        ParticleSnapshot particle;
        if (!parse_particle(cursor, particle, result))
        {
            return false;
        }
        snapshot.particles.push_back(particle);
    } while (cursor.consume(','));

    return cursor.expect(']', result, "Expected ']' after particles array");
}

bool parse_shape_type(JsonCursor &cursor, SnapshotShapeType &shape_type, SerializationResult &result)
{
    cursor.skip_whitespace();
    if (cursor.peek('"'))
    {
        const std::string value = cursor.parse_string(result);
        if (!result.is_success())
        {
            return false;
        }

        if (value == "sphere")
        {
            shape_type = SnapshotShapeType::Sphere;
        }
        else if (value == "box")
        {
            shape_type = SnapshotShapeType::Box;
        }
        else if (value == "capsule")
        {
            shape_type = SnapshotShapeType::Capsule;
        }
        else
        {
            shape_type = SnapshotShapeType::Unknown;
        }
        return true;
    }

    const auto raw = static_cast<uint32_t>(cursor.parse_number(result));
    if (!result.is_success())
    {
        return false;
    }

    if (raw == static_cast<uint32_t>(SnapshotShapeType::Sphere))
    {
        shape_type = SnapshotShapeType::Sphere;
    }
    else if (raw == static_cast<uint32_t>(SnapshotShapeType::Box))
    {
        shape_type = SnapshotShapeType::Box;
    }
    else if (raw == static_cast<uint32_t>(SnapshotShapeType::Capsule))
    {
        shape_type = SnapshotShapeType::Capsule;
    }
    else
    {
        shape_type = SnapshotShapeType::Unknown;
    }

    return true;
}

bool parse_rigid_body(JsonCursor &cursor, RigidBodySnapshot &rigid_body, SerializationResult &result)
{
    if (!cursor.expect('{', result, "Expected '{' for rigid body"))
    {
        return false;
    }

    if (cursor.consume('}'))
    {
        return true;
    }

    do
    {
        const std::string key = cursor.parse_string(result);
        if (!result.is_success() || !cursor.expect(':', result, "Expected ':' after rigid body key"))
        {
            return false;
        }

        if (key == "position")
        {
            if (!parse_vec3(cursor, rigid_body.position, result))
            {
                return false;
            }
        }
        else if (key == "velocity")
        {
            if (!parse_vec3(cursor, rigid_body.velocity, result))
            {
                return false;
            }
        }
        else if (key == "force_accumulator")
        {
            if (!parse_vec3(cursor, rigid_body.force_accumulator, result))
            {
                return false;
            }
        }
        else if (key == "orientation")
        {
            if (!cursor.expect('[', result, "Expected '[' for orientation"))
            {
                return false;
            }
            rigid_body.orientation_w = static_cast<float>(cursor.parse_number(result));
            if (!result.is_success() || !cursor.expect(',', result, "Expected ',' after orientation w"))
            {
                return false;
            }
            rigid_body.orientation_x = static_cast<float>(cursor.parse_number(result));
            if (!result.is_success() || !cursor.expect(',', result, "Expected ',' after orientation x"))
            {
                return false;
            }
            rigid_body.orientation_y = static_cast<float>(cursor.parse_number(result));
            if (!result.is_success() || !cursor.expect(',', result, "Expected ',' after orientation y"))
            {
                return false;
            }
            rigid_body.orientation_z = static_cast<float>(cursor.parse_number(result));
            if (!result.is_success() || !cursor.expect(']', result, "Expected ']' after orientation"))
            {
                return false;
            }
        }
        else if (key == "angular_velocity")
        {
            if (!parse_vec3(cursor, rigid_body.angular_velocity, result))
            {
                return false;
            }
        }
        else if (key == "torque_accumulator")
        {
            if (!parse_vec3(cursor, rigid_body.torque_accumulator, result))
            {
                return false;
            }
        }
        else if (key == "shape_type")
        {
            if (!parse_shape_type(cursor, rigid_body.shape_type, result))
            {
                return false;
            }
        }
        else if (key == "shape_local_center")
        {
            if (!parse_vec3(cursor, rigid_body.shape_local_center, result))
            {
                return false;
            }
        }
        else if (key == "shape_radius")
        {
            rigid_body.shape_radius = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "shape_half_extents")
        {
            if (!parse_vec3(cursor, rigid_body.shape_half_extents, result))
            {
                return false;
            }
        }
        else if (key == "shape_half_height")
        {
            rigid_body.shape_half_height = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "collision_radius")
        {
            rigid_body.collision_radius = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "mass")
        {
            rigid_body.mass = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "restitution")
        {
            rigid_body.restitution = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "friction")
        {
            rigid_body.friction = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "linear_damping")
        {
            rigid_body.linear_damping = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "angular_damping")
        {
            rigid_body.angular_damping = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "drag_coefficient")
        {
            rigid_body.drag_coefficient = static_cast<float>(cursor.parse_number(result));
        }
        else if (key == "active")
        {
            rigid_body.active = cursor.parse_bool(result);
        }
        else if (key == "id")
        {
            rigid_body.id = static_cast<int>(cursor.parse_number(result));
        }
        else if (key == "lifetime")
        {
            rigid_body.lifetime = static_cast<float>(cursor.parse_number(result));
        }
        else
        {
            cursor.skip_value(result);
        }

        if (!result.is_success())
        {
            return false;
        }
    } while (cursor.consume(','));

    return cursor.expect('}', result, "Expected '}' after rigid body object");
}

bool parse_rigid_bodies(JsonCursor &cursor, PhysicsSnapshot &snapshot, SerializationResult &result)
{
    if (!cursor.expect('[', result, "Expected '[' for rigid_bodies"))
    {
        return false;
    }

    snapshot.rigid_bodies.clear();
    if (cursor.consume(']'))
    {
        return true;
    }

    do
    {
        RigidBodySnapshot rigid_body;
        if (!parse_rigid_body(cursor, rigid_body, result))
        {
            return false;
        }
        snapshot.rigid_bodies.push_back(rigid_body);
    } while (cursor.consume(','));

    return cursor.expect(']', result, "Expected ']' after rigid_bodies array");
}

} // namespace

namespace phynity::serialization
{

std::string SerializationResult::to_string() const
{
    if (error == SerializationError::Success)
    {
        return "Success (" + std::to_string(bytes_processed) + " bytes)";
    }
    return "Error: " + error_message;
}

SerializationResult SnapshotSerializer::save_binary(const PhysicsSnapshot &snapshot, const std::string &file_path)
{
    SerializationResult result;
    if (!snapshot.is_valid())
    {
        result.error = SerializationError::CorruptedData;
        result.error_message = "Snapshot failed validity check";
        return result;
    }

    std::ofstream file(file_path, std::ios::binary);
    if (!file.is_open())
    {
        result.error = SerializationError::WriteError;
        result.error_message = "Cannot open file for writing: " + file_path;
        return result;
    }

    try
    {
        SnapshotFileHeader header;
        header.schema_major = snapshot.schema_version.major;
        header.schema_minor = snapshot.schema_version.minor;
        header.schema_patch = snapshot.schema_version.patch;
        header.num_particles = static_cast<uint32_t>(snapshot.particles.size());
        header.num_rigid_bodies = static_cast<uint32_t>(snapshot.rigid_bodies.size());
        header.metadata_json_bytes = 0;
        header.file_size = snapshot.serialized_size();

        file.write(reinterpret_cast<const char *>(&header), sizeof(SnapshotFileHeader));
        result.bytes_processed += sizeof(SnapshotFileHeader);

        file.write(reinterpret_cast<const char *>(&snapshot.frame_number), sizeof(snapshot.frame_number));
        file.write(reinterpret_cast<const char *>(&snapshot.simulated_time), sizeof(snapshot.simulated_time));
        file.write(reinterpret_cast<const char *>(&snapshot.timestep), sizeof(snapshot.timestep));
        file.write(reinterpret_cast<const char *>(&snapshot.rng_seed), sizeof(snapshot.rng_seed));
        result.bytes_processed += sizeof(snapshot.frame_number) + sizeof(snapshot.simulated_time) +
                                  sizeof(snapshot.timestep) + sizeof(snapshot.rng_seed);

        for (const auto &particle : snapshot.particles)
        {
            file.write(reinterpret_cast<const char *>(&particle), sizeof(ParticleSnapshot));
            result.bytes_processed += sizeof(ParticleSnapshot);
        }

        for (const auto &rigid_body : snapshot.rigid_bodies)
        {
            file.write(reinterpret_cast<const char *>(&rigid_body), sizeof(RigidBodySnapshot));
            result.bytes_processed += sizeof(RigidBodySnapshot);
        }

        if (!file.good())
        {
            result.error = SerializationError::WriteError;
            result.error_message = "Error writing snapshot body to file";
            return result;
        }

        result.error = SerializationError::Success;
        return result;
    }
    catch (const std::exception &error)
    {
        result.error = SerializationError::IOError;
        result.error_message = std::string("Exception during write: ") + error.what();
        return result;
    }
}

SerializationResult SnapshotSerializer::load_binary(const std::string &file_path, PhysicsSnapshot &snapshot)
{
    SerializationResult result;
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open())
    {
        result.error = SerializationError::FileNotFound;
        result.error_message = "Cannot open file for reading: " + file_path;
        return result;
    }

    try
    {
        uint32_t magic = 0;
        uint32_t format_version = 0;
        file.read(reinterpret_cast<char *>(&magic), sizeof(magic));
        file.read(reinterpret_cast<char *>(&format_version), sizeof(format_version));
        result.bytes_processed += sizeof(magic) + sizeof(format_version);

        if (!file.good())
        {
            result.error = SerializationError::CorruptedData;
            result.error_message = "Unable to read snapshot file header";
            return result;
        }

        if (magic != SnapshotFileHeader::MAGIC_NUMBER)
        {
            result.error = SerializationError::InvalidFileFormat;
            result.error_message = "Invalid magic number in file header";
            return result;
        }

        uint32_t schema_major = 0;
        uint32_t schema_minor = 0;
        uint32_t schema_patch = 0;
        uint32_t num_particles = 0;
        uint32_t num_rigid_bodies = 0;

        if (format_version == 1)
        {
            SnapshotFileHeaderV1 legacy_header;
            legacy_header.magic = magic;
            legacy_header.format_version = format_version;

            file.read(reinterpret_cast<char *>(&legacy_header.schema_major),
                      sizeof(SnapshotFileHeaderV1) - (sizeof(uint32_t) * 2));
            result.bytes_processed += sizeof(SnapshotFileHeaderV1) - (sizeof(uint32_t) * 2);

            if (!file.good())
            {
                result.error = SerializationError::CorruptedData;
                result.error_message = "Incomplete legacy snapshot header";
                return result;
            }

            schema_major = legacy_header.schema_major;
            schema_minor = legacy_header.schema_minor;
            schema_patch = legacy_header.schema_patch;
            num_particles = legacy_header.num_particles;
            num_rigid_bodies = 0;
        }
        else if (format_version == SnapshotFileHeader::FORMAT_VERSION)
        {
            SnapshotFileHeader header;
            header.magic = magic;
            header.format_version = format_version;
            file.read(reinterpret_cast<char *>(&header.schema_major),
                      sizeof(SnapshotFileHeader) - (sizeof(uint32_t) * 2));
            result.bytes_processed += sizeof(SnapshotFileHeader) - (sizeof(uint32_t) * 2);

            if (!file.good())
            {
                result.error = SerializationError::CorruptedData;
                result.error_message = "Incomplete snapshot header";
                return result;
            }

            schema_major = header.schema_major;
            schema_minor = header.schema_minor;
            schema_patch = header.schema_patch;
            num_particles = header.num_particles;
            num_rigid_bodies = header.num_rigid_bodies;
        }
        else
        {
            result.error = SerializationError::SchemaVersionMismatch;
            result.error_message = "Unsupported file format version: " + std::to_string(format_version);
            return result;
        }

        snapshot = PhysicsSnapshot{};
        snapshot.schema_version.major = schema_major;
        snapshot.schema_version.minor = schema_minor;
        snapshot.schema_version.patch = schema_patch;

        file.read(reinterpret_cast<char *>(&snapshot.frame_number), sizeof(snapshot.frame_number));
        file.read(reinterpret_cast<char *>(&snapshot.simulated_time), sizeof(snapshot.simulated_time));
        file.read(reinterpret_cast<char *>(&snapshot.timestep), sizeof(snapshot.timestep));
        file.read(reinterpret_cast<char *>(&snapshot.rng_seed), sizeof(snapshot.rng_seed));
        result.bytes_processed += sizeof(snapshot.frame_number) + sizeof(snapshot.simulated_time) +
                                  sizeof(snapshot.timestep) + sizeof(snapshot.rng_seed);

        snapshot.particles.resize(num_particles);
        for (uint32_t index = 0; index < num_particles; ++index)
        {
            file.read(reinterpret_cast<char *>(&snapshot.particles[index]), sizeof(ParticleSnapshot));
            result.bytes_processed += sizeof(ParticleSnapshot);
        }

        snapshot.rigid_bodies.resize(num_rigid_bodies);
        for (uint32_t index = 0; index < num_rigid_bodies; ++index)
        {
            file.read(reinterpret_cast<char *>(&snapshot.rigid_bodies[index]), sizeof(RigidBodySnapshot));
            result.bytes_processed += sizeof(RigidBodySnapshot);
        }

        if (!file.good() && !file.eof())
        {
            result.error = SerializationError::CorruptedData;
            result.error_message = "Incomplete read of snapshot data";
            return result;
        }

        const auto migration_result = migrate_snapshot_to_schema(snapshot, current_schema_version());
        if (!migration_result.is_success())
        {
            return migration_result;
        }
        result.bytes_processed += migration_result.bytes_processed;

        if (!snapshot.is_valid())
        {
            result.error = SerializationError::CorruptedData;
            result.error_message = "Loaded binary snapshot failed validity check";
            return result;
        }

        result.error = SerializationError::Success;
        return result;
    }
    catch (const std::exception &error)
    {
        result.error = SerializationError::IOError;
        result.error_message = std::string("Exception during read: ") + error.what();
        return result;
    }
}

SerializationResult
SnapshotSerializer::save_json(const PhysicsSnapshot &snapshot, const std::string &file_path, bool pretty_print)
{
    SerializationResult result;
    if (!snapshot.is_valid())
    {
        result.error = SerializationError::CorruptedData;
        result.error_message = "Snapshot failed validity check";
        return result;
    }

    std::ofstream file(file_path);
    if (!file.is_open())
    {
        result.error = SerializationError::WriteError;
        result.error_message = "Cannot open JSON file for writing: " + file_path;
        return result;
    }

    try
    {
        std::ostringstream json_stream;
        json_stream << std::setprecision(std::numeric_limits<float>::max_digits10);
        json_stream << "{";
        if (pretty_print)
        {
            json_stream << "\n";
        }

        if (pretty_print)
        {
            json_stream << "  ";
        }
        json_stream << "\"schema_version\": {";
        if (pretty_print)
        {
            json_stream << "\n    ";
        }
        else
        {
            json_stream << " ";
        }
        json_stream << "\"major\": " << snapshot.schema_version.major << ",";
        if (pretty_print)
        {
            json_stream << "\n    ";
        }
        else
        {
            json_stream << " ";
        }
        json_stream << "\"minor\": " << snapshot.schema_version.minor << ",";
        if (pretty_print)
        {
            json_stream << "\n    ";
        }
        else
        {
            json_stream << " ";
        }
        json_stream << "\"patch\": " << snapshot.schema_version.patch << "\n";
        if (pretty_print)
        {
            json_stream << "  ";
        }
        json_stream << "},\n";

        if (pretty_print)
        {
            json_stream << "  ";
        }
        json_stream << "\"metadata\": {";
        if (pretty_print)
        {
            json_stream << "\n    ";
        }
        else
        {
            json_stream << " ";
        }
        json_stream << "\"frame_number\": " << snapshot.frame_number << ",";
        if (pretty_print)
        {
            json_stream << "\n    ";
        }
        else
        {
            json_stream << " ";
        }
        json_stream << "\"simulated_time\": " << snapshot.simulated_time << ",";
        if (pretty_print)
        {
            json_stream << "\n    ";
        }
        else
        {
            json_stream << " ";
        }
        json_stream << "\"timestep\": " << snapshot.timestep << ",";
        if (pretty_print)
        {
            json_stream << "\n    ";
        }
        else
        {
            json_stream << " ";
        }
        json_stream << "\"rng_seed\": " << snapshot.rng_seed << "\n";
        if (pretty_print)
        {
            json_stream << "  ";
        }
        json_stream << "},\n";

        if (pretty_print)
        {
            json_stream << "  ";
        }
        json_stream << "\"particles\": [";
        if (pretty_print && !snapshot.particles.empty())
        {
            json_stream << "\n";
        }

        for (size_t index = 0; index < snapshot.particles.size(); ++index)
        {
            const auto &particle = snapshot.particles[index];
            if (pretty_print)
            {
                json_stream << "    ";
            }
            json_stream << "{";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"position\": [" << particle.position.x << ", " << particle.position.y << ", "
                        << particle.position.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"velocity\": [" << particle.velocity.x << ", " << particle.velocity.y << ", "
                        << particle.velocity.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"acceleration\": [" << particle.acceleration.x << ", " << particle.acceleration.y << ", "
                        << particle.acceleration.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"force_accumulator\": [" << particle.force_accumulator.x << ", "
                        << particle.force_accumulator.y << ", " << particle.force_accumulator.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"radius\": " << particle.radius << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"mass\": " << particle.mass << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"restitution\": " << particle.restitution << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"friction\": " << particle.friction << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"linear_damping\": " << particle.linear_damping << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"angular_damping\": " << particle.angular_damping << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"drag_coefficient\": " << particle.drag_coefficient << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"lifetime\": " << particle.lifetime << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"active\": " << (particle.active ? "true" : "false") << "\n";
            if (pretty_print)
            {
                json_stream << "    ";
            }
            json_stream << "}";
            if (index + 1 < snapshot.particles.size())
            {
                json_stream << ",";
            }
            if (pretty_print)
            {
                json_stream << "\n";
            }
        }

        if (pretty_print && !snapshot.particles.empty())
        {
            json_stream << "  ";
        }
        json_stream << "],";
        if (pretty_print)
        {
            json_stream << "\n  ";
        }
        else
        {
            json_stream << " ";
        }

        json_stream << "\"rigid_bodies\": [";
        if (pretty_print && !snapshot.rigid_bodies.empty())
        {
            json_stream << "\n";
        }

        for (size_t index = 0; index < snapshot.rigid_bodies.size(); ++index)
        {
            const auto &rigid = snapshot.rigid_bodies[index];
            if (pretty_print)
            {
                json_stream << "    ";
            }
            json_stream << "{";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"position\": [" << rigid.position.x << ", " << rigid.position.y << ", " << rigid.position.z
                        << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"velocity\": [" << rigid.velocity.x << ", " << rigid.velocity.y << ", " << rigid.velocity.z
                        << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"force_accumulator\": [" << rigid.force_accumulator.x << ", " << rigid.force_accumulator.y
                        << ", " << rigid.force_accumulator.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"orientation\": [" << rigid.orientation_w << ", " << rigid.orientation_x << ", "
                        << rigid.orientation_y << ", " << rigid.orientation_z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"angular_velocity\": [" << rigid.angular_velocity.x << ", " << rigid.angular_velocity.y
                        << ", " << rigid.angular_velocity.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"torque_accumulator\": [" << rigid.torque_accumulator.x << ", "
                        << rigid.torque_accumulator.y << ", " << rigid.torque_accumulator.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"shape_type\": " << static_cast<uint32_t>(rigid.shape_type) << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"shape_local_center\": [" << rigid.shape_local_center.x << ", "
                        << rigid.shape_local_center.y << ", " << rigid.shape_local_center.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"shape_radius\": " << rigid.shape_radius << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"shape_half_extents\": [" << rigid.shape_half_extents.x << ", "
                        << rigid.shape_half_extents.y << ", " << rigid.shape_half_extents.z << "],";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"shape_half_height\": " << rigid.shape_half_height << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"collision_radius\": " << rigid.collision_radius << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"mass\": " << rigid.mass << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"restitution\": " << rigid.restitution << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"friction\": " << rigid.friction << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"linear_damping\": " << rigid.linear_damping << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"angular_damping\": " << rigid.angular_damping << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"drag_coefficient\": " << rigid.drag_coefficient << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"active\": " << (rigid.active ? "true" : "false") << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"id\": " << rigid.id << ",";
            if (pretty_print)
            {
                json_stream << "\n      ";
            }
            else
            {
                json_stream << " ";
            }

            json_stream << "\"lifetime\": " << rigid.lifetime << "\n";
            if (pretty_print)
            {
                json_stream << "    ";
            }

            json_stream << "}";
            if (index + 1 < snapshot.rigid_bodies.size())
            {
                json_stream << ",";
            }
            if (pretty_print)
            {
                json_stream << "\n";
            }
        }

        if (pretty_print && !snapshot.rigid_bodies.empty())
        {
            json_stream << "  ";
        }
        json_stream << "]\n}";

        file << json_stream.str();
        result.bytes_processed = json_stream.str().size();
        if (!file.good())
        {
            result.error = SerializationError::WriteError;
            result.error_message = "Error writing JSON data";
            return result;
        }

        result.error = SerializationError::Success;
        return result;
    }
    catch (const std::exception &error)
    {
        result.error = SerializationError::IOError;
        result.error_message = std::string("Exception during JSON write: ") + error.what();
        return result;
    }
}

SerializationResult SnapshotSerializer::load_json(const std::string &file_path, PhysicsSnapshot &snapshot)
{
    SerializationResult result;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        result.error = SerializationError::FileNotFound;
        result.error_message = "Cannot open JSON file for reading: " + file_path;
        return result;
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    const std::string content = buffer.str();

    JsonCursor cursor(content);
    snapshot = PhysicsSnapshot{};
    if (!cursor.expect('{', result, "Expected JSON root object"))
    {
        return result;
    }

    if (!cursor.consume('}'))
    {
        do
        {
            const std::string key = cursor.parse_string(result);
            if (!result.is_success() || !cursor.expect(':', result, "Expected ':' after root key"))
            {
                return result;
            }

            if (key == "schema_version")
            {
                if (!parse_schema_version(cursor, snapshot.schema_version, result))
                {
                    return result;
                }
            }
            else if (key == "metadata")
            {
                if (!parse_metadata(cursor, snapshot, result))
                {
                    return result;
                }
            }
            else if (key == "particles")
            {
                if (!parse_particles(cursor, snapshot, result))
                {
                    return result;
                }
            }
            else if (key == "rigid_bodies")
            {
                if (!parse_rigid_bodies(cursor, snapshot, result))
                {
                    return result;
                }
            }
            else
            {
                cursor.skip_value(result);
                if (!result.is_success())
                {
                    return result;
                }
            }
        } while (cursor.consume(','));

        if (!cursor.expect('}', result, "Expected '}' after JSON root object"))
        {
            return result;
        }
    }

    if (!cursor.eof())
    {
        result.error = SerializationError::JsonError;
        result.error_message = "Unexpected trailing content in JSON file";
        return result;
    }

    const auto migration_result = migrate_snapshot_to_schema(snapshot, current_schema_version());
    if (!migration_result.is_success())
    {
        return migration_result;
    }

    if (!snapshot.is_valid())
    {
        result.error = SerializationError::CorruptedData;
        result.error_message = "Loaded JSON snapshot failed validity check";
        return result;
    }

    result.error = SerializationError::Success;
    result.bytes_processed = content.size() + migration_result.bytes_processed;
    return result;
}

SerializationResult SnapshotSerializer::save_both(const PhysicsSnapshot &snapshot, const std::string &file_path_base)
{
    const auto binary_result = save_binary(snapshot, file_path_base + ".bin");
    if (!binary_result.is_success())
    {
        return binary_result;
    }

    const auto json_result = save_json(snapshot, file_path_base + ".json");
    if (!json_result.is_success())
    {
        return json_result;
    }

    return binary_result;
}

bool SnapshotSerializer::can_migrate(const SchemaVersion &legacy_version, const SchemaVersion &current_version)
{
    return can_migrate_snapshot_schema(legacy_version, current_version);
}

std::string SnapshotSerializer::describe_snapshot(const PhysicsSnapshot &snapshot)
{
    std::ostringstream description;
    description << "PhysicsSnapshot {\n";
    description << "  schema_version: " << snapshot.schema_version.to_string() << "\n";
    description << "  frame_number: " << snapshot.frame_number << "\n";
    description << "  simulated_time: " << snapshot.simulated_time << "s\n";
    description << "  timestep: " << snapshot.timestep << "s\n";
    description << "  rng_seed: " << snapshot.rng_seed << "\n";
    description << "  particles: " << snapshot.particles.size() << " active\n";
    description << "  rigid_bodies: " << snapshot.rigid_bodies.size() << " active\n";
    description << "  serialized_size: " << snapshot.serialized_size() << " bytes\n";
    description << "}\n";
    return description.str();
}

} // namespace phynity::serialization
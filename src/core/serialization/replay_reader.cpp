#include <core/serialization/replay_reader.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace phynity::serialization
{

namespace
{

std::filesystem::path make_temp_frame_path(size_t frame_index)
{
    const auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::ostringstream file_name;
    file_name << "phynity_replay_read_" << now << "_" << frame_index << ".bin";
    return std::filesystem::temp_directory_path() / file_name.str();
}

SerializationResult
deserialize_snapshot_from_bytes(const std::vector<uint8_t> &bytes, size_t frame_index, PhysicsSnapshot &snapshot)
{
    const auto temp_file = make_temp_frame_path(frame_index);

    {
        std::ofstream output(temp_file, std::ios::binary | std::ios::trunc);
        if (!output.is_open())
        {
            return {SerializationError::WriteError, "Cannot write temporary replay frame", 0};
        }

        if (!bytes.empty())
        {
            output.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        }

        if (!output.good())
        {
            std::filesystem::remove(temp_file);
            return {SerializationError::IOError, "Failed to write temporary replay frame bytes", 0};
        }
    }

    const auto load_result = SnapshotSerializer::load_binary(temp_file.string(), snapshot);
    std::filesystem::remove(temp_file);
    return load_result;
}

} // namespace

SerializationResult ReplayReader::open(const std::string &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
    {
        return {SerializationError::FileNotFound, "Cannot open replay file: " + path, 0};
    }

    input.seekg(0, std::ios::end);
    const std::streamoff file_size = input.tellg();
    input.seekg(0, std::ios::beg);

    if (file_size < static_cast<std::streamoff>(sizeof(ReplayFileHeader)))
    {
        return {SerializationError::InvalidFileFormat, "Replay file too small for header", 0};
    }

    ReplayFileHeader header{};
    input.read(reinterpret_cast<char *>(&header), sizeof(header));
    if (!input.good())
    {
        return {SerializationError::IOError, "Failed to read replay header", 0};
    }

    if (header.magic != replay_file_magic)
    {
        return {SerializationError::InvalidFileFormat, "Replay magic number mismatch", 0};
    }

    if (header.format_version != replay_format_version)
    {
        return {SerializationError::InvalidFileFormat, "Unsupported replay format version", 0};
    }

    const uint64_t frame_count_u64 = header.frame_count;
    if (frame_count_u64 > static_cast<uint64_t>(SIZE_MAX))
    {
        return {SerializationError::InvalidFileFormat, "Replay frame count exceeds platform limits", 0};
    }

    std::vector<ReplayFrameIndexEntry> index(static_cast<size_t>(frame_count_u64));
    if (!index.empty())
    {
        input.read(reinterpret_cast<char *>(index.data()),
                   static_cast<std::streamsize>(index.size() * sizeof(ReplayFrameIndexEntry)));
        if (!input.good())
        {
            return {SerializationError::IOError, "Failed to read replay frame index", 0};
        }
    }

    const uint64_t min_data_offset = static_cast<uint64_t>(sizeof(ReplayFileHeader)) +
                                     static_cast<uint64_t>(index.size() * sizeof(ReplayFrameIndexEntry));

    uint64_t previous_end = min_data_offset;
    for (size_t i = 0; i < index.size(); ++i)
    {
        const auto &entry = index[i];
        if (entry.offset < min_data_offset || entry.offset < previous_end)
        {
            return {SerializationError::InvalidFileFormat, "Replay frame index is not monotonic", 0};
        }

        const uint64_t end = entry.offset + entry.size;
        if (end < entry.offset || end > static_cast<uint64_t>(file_size))
        {
            return {SerializationError::InvalidFileFormat, "Replay frame entry exceeds file bounds", 0};
        }

        previous_end = end;
    }

    input_path_ = path;
    header_ = header;
    const size_t previous_capacity = index_.capacity();
    index_ = std::move(index);
    phynity::platform::track_vector_capacity_change(index_, previous_capacity);
    is_open_ = true;
    next_index_ = 0;

    return {SerializationError::Success, "", static_cast<size_t>(file_size)};
}

size_t ReplayReader::frame_count() const
{
    return index_.size();
}

SerializationResult ReplayReader::read_frame(size_t index, PhysicsSnapshot &snapshot) const
{
    if (!is_open_)
    {
        return {SerializationError::IOError, "ReplayReader is not open", 0};
    }

    if (index >= index_.size())
    {
        return {SerializationError::InvalidFileFormat, "Replay frame index out of range", 0};
    }

    const auto &entry = index_[index];

    std::ifstream input(input_path_, std::ios::binary);
    if (!input.is_open())
    {
        return {SerializationError::FileNotFound, "Cannot reopen replay file: " + input_path_, 0};
    }

    input.seekg(static_cast<std::streamoff>(entry.offset), std::ios::beg);
    if (!input.good())
    {
        return {SerializationError::IOError, "Failed to seek to replay frame", 0};
    }

    std::vector<uint8_t> bytes(static_cast<size_t>(entry.size));
    if (!bytes.empty())
    {
        input.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(entry.size));
        if (!input.good())
        {
            return {SerializationError::IOError, "Failed to read replay frame bytes", 0};
        }
    }

    return deserialize_snapshot_from_bytes(bytes, index, snapshot);
}

SerializationResult ReplayReader::read_next(PhysicsSnapshot &snapshot)
{
    const auto result = read_frame(next_index_, snapshot);
    if (result.is_success())
    {
        ++next_index_;
    }
    return result;
}

void ReplayReader::reset_iteration()
{
    next_index_ = 0;
}

} // namespace phynity::serialization

#include <core/serialization/replay_writer.hpp>

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
    file_name << "phynity_replay_write_" << now << "_" << frame_index << ".bin";
    return std::filesystem::temp_directory_path() / file_name.str();
}

SerializationResult
serialize_snapshot_to_bytes(const PhysicsSnapshot &snapshot, size_t frame_index, std::vector<uint8_t> &bytes)
{
    const auto temp_file = make_temp_frame_path(frame_index);
    const auto save_result = SnapshotSerializer::save_binary(snapshot, temp_file.string());
    if (!save_result.is_success())
    {
        return save_result;
    }

    std::ifstream input(temp_file, std::ios::binary);
    if (!input.is_open())
    {
        std::filesystem::remove(temp_file);
        return {SerializationError::IOError, "Cannot reopen temporary replay frame for reading", 0};
    }

    input.seekg(0, std::ios::end);
    const std::streamsize size = input.tellg();
    if (size < 0)
    {
        std::filesystem::remove(temp_file);
        return {SerializationError::IOError, "Failed to inspect temporary replay frame size", 0};
    }

    input.seekg(0, std::ios::beg);
    bytes.resize(static_cast<size_t>(size));
    if (!bytes.empty())
    {
        input.read(reinterpret_cast<char *>(bytes.data()), size);
    }

    const bool read_ok = input.good() || input.eof();
    input.close();
    std::filesystem::remove(temp_file);

    if (!read_ok)
    {
        return {SerializationError::IOError, "Failed to read temporary replay frame bytes", 0};
    }

    return {SerializationError::Success, "", static_cast<size_t>(size)};
}

} // namespace

SerializationResult ReplayWriter::open(const std::string &path)
{
    if (is_open_)
    {
        return {SerializationError::IOError, "ReplayWriter is already open", 0};
    }

    output_path_ = path;
    is_open_ = true;
    dt_locked_ = false;
    frame_dt_ = 1.0 / 60.0;
    frames_.clear();
    return {SerializationError::Success, "", 0};
}

SerializationResult ReplayWriter::append_frame(const PhysicsSnapshot &snapshot)
{
    if (!is_open_)
    {
        return {SerializationError::IOError, "ReplayWriter is not open", 0};
    }

    if (!dt_locked_)
    {
        frame_dt_ = snapshot.timestep > 0.0f ? static_cast<double>(snapshot.timestep) : (1.0 / 60.0);
        dt_locked_ = true;
    }

    std::vector<uint8_t> bytes;
    const auto serialize_result = serialize_snapshot_to_bytes(snapshot, frames_.size(), bytes);
    if (!serialize_result.is_success())
    {
        return serialize_result;
    }

    frames_.push_back(std::move(bytes));
    return {SerializationError::Success, "", serialize_result.bytes_processed};
}

SerializationResult ReplayWriter::close()
{
    if (!is_open_)
    {
        return {SerializationError::IOError, "ReplayWriter is not open", 0};
    }

    std::ofstream output(output_path_, std::ios::binary | std::ios::trunc);
    if (!output.is_open())
    {
        return {SerializationError::WriteError, "Cannot open replay file for writing: " + output_path_, 0};
    }

    ReplayFileHeader header;
    header.frame_count = static_cast<uint64_t>(frames_.size());
    header.frame_dt = frame_dt_;

    std::vector<ReplayFrameIndexEntry> frame_index(frames_.size());
    const uint64_t table_size = static_cast<uint64_t>(frame_index.size() * sizeof(ReplayFrameIndexEntry));
    uint64_t frame_offset = static_cast<uint64_t>(sizeof(ReplayFileHeader)) + table_size;

    for (size_t i = 0; i < frames_.size(); ++i)
    {
        frame_index[i].offset = frame_offset;
        frame_index[i].size = static_cast<uint64_t>(frames_[i].size());
        frame_offset += frame_index[i].size;
    }

    output.write(reinterpret_cast<const char *>(&header), sizeof(header));
    if (!frame_index.empty())
    {
        output.write(reinterpret_cast<const char *>(frame_index.data()), static_cast<std::streamsize>(table_size));
    }

    for (const auto &frame_bytes : frames_)
    {
        if (!frame_bytes.empty())
        {
            output.write(reinterpret_cast<const char *>(frame_bytes.data()),
                         static_cast<std::streamsize>(frame_bytes.size()));
        }
    }

    if (!output.good())
    {
        return {SerializationError::IOError, "I/O error while writing replay file: " + output_path_, 0};
    }

    const size_t bytes_written = static_cast<size_t>(frame_offset);

    output.close();
    frames_.clear();
    output_path_.clear();
    is_open_ = false;
    dt_locked_ = false;

    return {SerializationError::Success, "", bytes_written};
}

bool ReplayWriter::is_open() const
{
    return is_open_;
}

} // namespace phynity::serialization

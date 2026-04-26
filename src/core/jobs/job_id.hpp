#pragma once

#include <cstdint>
#include <functional>
#include <limits>

namespace phynity::jobs
{

/// Unified handle for jobs in the system. Replaces both JobHandle and TaskId.
/// index = pool slot (or graph-local index during graph construction).
/// generation = ABA protection for pool slot reuse.
struct JobId
{
    uint32_t index = invalid_index;
    uint32_t generation = 0;

    static constexpr uint32_t invalid_index = std::numeric_limits<uint32_t>::max();

    [[nodiscard]] bool valid() const noexcept
    {
        return index != invalid_index;
    }

    bool operator==(const JobId &other) const noexcept = default;

    bool operator<(const JobId &other) const noexcept
    {
        return index < other.index;
    }
};

/// Handle to a shared completion counter. Waiters block until the counter reaches zero.
struct CounterHandle
{
    uint32_t index = std::numeric_limits<uint32_t>::max();
    uint32_t generation = 0;

    [[nodiscard]] bool valid() const noexcept
    {
        return index != std::numeric_limits<uint32_t>::max();
    }

    bool operator==(const CounterHandle &other) const noexcept = default;
};

} // namespace phynity::jobs

template <> struct std::hash<phynity::jobs::JobId>
{
    size_t operator()(const phynity::jobs::JobId &id) const noexcept
    {
        return std::hash<uint64_t>{}(static_cast<uint64_t>(id.index) << 32 | id.generation);
    }
};

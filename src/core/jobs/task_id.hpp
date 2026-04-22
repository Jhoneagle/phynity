#pragma once

#include <cstdint>
#include <limits>

namespace phynity::jobs
{

struct TaskId
{
    uint32_t value = invalid_value;

    static constexpr uint32_t invalid_value = std::numeric_limits<uint32_t>::max();

    [[nodiscard]] bool valid() const noexcept
    {
        return value != invalid_value;
    }

    bool operator==(const TaskId &other) const noexcept = default;
    bool operator<(const TaskId &other) const noexcept
    {
        return value < other.value;
    }
};

} // namespace phynity::jobs

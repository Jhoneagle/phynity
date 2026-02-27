#pragma once

#include <cstdint>

namespace phynity::jobs {

struct JobHandle {
    uint32_t id = 0;
    uint32_t generation = 0;

    [[nodiscard]] bool valid() const noexcept {
        return id != 0 || generation != 0;
    }
};

}  // namespace phynity::jobs

#pragma once

#include <chrono>
#include <cstdint>
#include <thread>

namespace phynity::platform {

using Thread = std::thread;
using ThreadId = std::thread::id;

[[nodiscard]] inline uint32_t hardware_concurrency() noexcept {
    const unsigned int count = std::thread::hardware_concurrency();
    return count == 0 ? 1u : static_cast<uint32_t>(count);
}

[[nodiscard]] inline ThreadId current_thread_id() noexcept {
    return std::this_thread::get_id();
}

inline void yield_thread() noexcept {
    std::this_thread::yield();
}

template <typename Rep, typename Period>
inline void sleep_for(const std::chrono::duration<Rep, Period>& duration) {
    std::this_thread::sleep_for(duration);
}

}  // namespace phynity::platform

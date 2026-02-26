#pragma once

#include "timer.hpp"
#include <vector>
#include <string_view>
#include <cstdint>

namespace phynity::diagnostics {

/**
 * @brief Profile zone entry representing a timed execution scope.
 * 
 * Records the name, timing, and hierarchical depth of a profiled code section.
 */
struct ProfileZone {
    std::string_view name;          ///< Zone name (must remain valid during profiling)
    uint64_t start_time_us;         ///< Start time in microseconds
    uint64_t duration_us;           ///< Duration in microseconds
    uint32_t depth;                 ///< Nesting level (0 = root)
    uint32_t parent_index;          ///< Index of parent zone in zone list

    ProfileZone() noexcept = default;
    ProfileZone(std::string_view name_, uint32_t depth_, uint32_t parent_)
        : name(name_), start_time_us(0), duration_us(0), depth(depth_), parent_index(parent_)
    {}
};

/**
 * @brief Thread-local profiling context.
 * 
 * Stores profiling zones for a single thread. Each thread maintains
 * its own zone stack and storage to avoid synchronization overhead.
 */
struct ProfileContext {
    std::vector<ProfileZone> zones;         ///< Collected zones for current frame
    std::vector<uint32_t> scope_stack;      ///< Stack of active zone indices
    bool enabled = false;                    ///< Runtime enable/disable flag
    
    void clear() noexcept {
        zones.clear();
        scope_stack.clear();
    }
    
    void reserve_zones(size_t count) {
        zones.reserve(count);
        scope_stack.reserve(32);  // Typical max nesting depth
    }
};

/**
 * @brief Global profiler singleton.
 * 
 * Manages profiling state and provides access to thread-local contexts.
 * Call enable() to turn profiling on/off at runtime.
 */
class Profiler {
public:
    /**
     * @brief Enable or disable profiling globally.
     * 
     * When disabled, ProfileScope constructors/destructors become no-ops.
     * 
     * @param enabled True to enable profiling, false to disable
     */
    static void enable(bool enabled) noexcept {
        get_context().enabled = enabled;
    }

    /**
     * @brief Check if profiling is currently enabled.
     */
    [[nodiscard]] static bool is_enabled() noexcept {
        return get_context().enabled;
    }

    /**
     * @brief Get the thread-local profiling context.
     * 
     * Each thread has its own context to avoid synchronization.
     */
    [[nodiscard]] static ProfileContext& get_context() noexcept {
        thread_local ProfileContext context;
        return context;
    }

    /**
     * @brief Clear all zones from current thread's context.
     * 
     * Typically called at the beginning of each frame.
     */
    static void clear_frame() noexcept {
        get_context().clear();
    }

    /**
     * @brief Reserve zone storage for current thread.
     * 
     * Pre-allocates storage to avoid allocations during profiling.
     * 
     * @param zone_count Expected number of zones per frame
     */
    static void reserve(size_t zone_count) {
        get_context().reserve_zones(zone_count);
    }

    /**
     * @brief Get all zones collected in current frame.
     * 
     * @return Vector of ProfileZone entries
     */
    [[nodiscard]] static const std::vector<ProfileZone>& get_zones() noexcept {
        return get_context().zones;
    }

    /**
     * @brief Begin a new profile zone.
     * 
     * Called by ProfileScope constructor. Not intended for direct use.
     * 
     * @param name Zone name (must remain valid during scope)
     * @return Zone index for use by end_zone
     */
    [[nodiscard]] static uint32_t begin_zone(std::string_view name) noexcept {
        auto& ctx = get_context();
        
        const uint32_t depth = static_cast<uint32_t>(ctx.scope_stack.size());
        const uint32_t parent_index = ctx.scope_stack.empty() ? 
            static_cast<uint32_t>(-1) : ctx.scope_stack.back();
        
        const uint32_t zone_index = static_cast<uint32_t>(ctx.zones.size());
        ctx.zones.emplace_back(name, depth, parent_index);
        ctx.zones.back().start_time_us = static_cast<uint64_t>(Timer::Clock::now().time_since_epoch().count() / 1000);
        
        ctx.scope_stack.push_back(zone_index);
        
        return zone_index;
    }

    /**
     * @brief End a profile zone.
     * 
     * Called by ProfileScope destructor. Not intended for direct use.
     * 
     * @param zone_index Zone index returned by begin_zone
     */
    static void end_zone(uint32_t zone_index) noexcept {
        auto& ctx = get_context();
        
        if (zone_index >= ctx.zones.size()) {
            return;  // Invalid zone index
        }
        
        const uint64_t end_time_us = static_cast<uint64_t>(Timer::Clock::now().time_since_epoch().count() / 1000);
        ctx.zones[zone_index].duration_us = end_time_us - ctx.zones[zone_index].start_time_us;
        
        if (!ctx.scope_stack.empty()) {
            ctx.scope_stack.pop_back();
        }
    }
};

/**
 * @brief RAII profiling scope marker.
 * 
 * Automatically times the scope in which it is constructed.
 * Use the PROFILE_SCOPE macro for convenient usage.
 * 
 * Example:
 * ```cpp
 * void expensive_function() {
 *     ProfileScope scope("expensive_function");
 *     // ... code to profile
 * }
 * ```
 * 
 * On construction, records start time and zone info.
 * On destruction, records duration and pops scope.
 */
class ProfileScope {
public:
    /**
     * @brief Construct profile scope and begin timing.
     * 
     * @param name Zone name (must be string literal or remain valid during scope)
     * 
     * If profiling is disabled, this is a no-op (just stores a flag).
     */
    explicit ProfileScope(std::string_view name) noexcept
        : zone_index_(static_cast<uint32_t>(-1))
        , enabled_(Profiler::is_enabled())
    {
        if (enabled_) {
            zone_index_ = Profiler::begin_zone(name);
        }
    }

    /**
     * @brief Destruct and record zone duration.
     * 
     * Records elapsed time if profiling was enabled.
     */
    ~ProfileScope() noexcept {
        if (enabled_) {
            Profiler::end_zone(zone_index_);
        }
    }

    // Non-copyable, non-movable (RAII guard)
    ProfileScope(const ProfileScope&) = delete;
    ProfileScope& operator=(const ProfileScope&) = delete;
    ProfileScope(ProfileScope&&) = delete;
    ProfileScope& operator=(ProfileScope&&) = delete;

private:
    uint32_t zone_index_;
    bool enabled_;
};

} // namespace phynity::diagnostics

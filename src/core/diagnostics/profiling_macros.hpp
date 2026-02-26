#pragma once

#include "profiler.hpp"

/**
 * @file profiling_macros.hpp
 * @brief Convenience macros for profiling.
 * 
 * These macros provide easy-to-use profiling instrumentation:
 * - PROFILE_SCOPE(name): Profile a named scope
 * - PROFILE_FUNCTION(): Profile the current function (auto-named)
 * 
 * Profiling can be disabled at compile-time by defining PHYNITY_DISABLE_PROFILING.
 * When disabled, macros expand to nothing (zero runtime cost).
 */

#ifndef PHYNITY_DISABLE_PROFILING

/**
 * @brief Profile a named scope.
 * 
 * Usage:
 * ```cpp
 * void process_data() {
 *     PROFILE_SCOPE("process_data");
 *     // ... code to profile
 * }
 * ```
 * 
 * @param name Zone name (must be string literal or compile-time constant)
 */
#define PROFILE_SCOPE(name) \
    ::phynity::diagnostics::ProfileScope PHYNITY_PROFILE_CONCAT(_profile_scope_, __LINE__)(name)

/**
 * @brief Profile the current function using its name.
 * 
 * Usage:
 * ```cpp
 * void expensive_calculation() {
 *     PROFILE_FUNCTION();
 *     // ... function body
 * }
 * ```
 * 
 * The zone will be named with the function name (e.g., "expensive_calculation").
 */
#define PROFILE_FUNCTION() \
    PROFILE_SCOPE(__FUNCTION__)

/**
 * @brief Internal helper macro for generating unique variable names.
 * 
 * Creates unique identifiers by concatenating prefix with line number.
 */
#define PHYNITY_PROFILE_CONCAT_IMPL(a, b) a ## b
#define PHYNITY_PROFILE_CONCAT(a, b) PHYNITY_PROFILE_CONCAT_IMPL(a, b)

#else  // PHYNITY_DISABLE_PROFILING

// When profiling is disabled, macros expand to nothing (zero cost)
#define PROFILE_SCOPE(name) ((void)0)
#define PROFILE_FUNCTION() ((void)0)

#endif  // PHYNITY_DISABLE_PROFILING

/**
 * @brief Enable or disable profiling at runtime.
 * 
 * Usage:
 * ```cpp
 * PROFILER_ENABLE(true);   // Enable profiling
 * // ... run code with profiling
 * PROFILER_ENABLE(false);  // Disable profiling
 * ```
 * 
 * Note: This only has effect if profiling is not disabled at compile-time.
 */
#define PROFILER_ENABLE(enabled) \
    ::phynity::diagnostics::Profiler::enable(enabled)

/**
 * @brief Clear all profiling zones for the current frame.
 * 
 * Usage:
 * ```cpp
 * void game_loop() {
 *     while (running) {
 *         PROFILER_CLEAR_FRAME();  // Clear previous frame's zones
 *         update();
 *         render();
 *         // ... analyze profiling data
 *     }
 * }
 * ```
 */
#define PROFILER_CLEAR_FRAME() \
    ::phynity::diagnostics::Profiler::clear_frame()

/**
 * @brief Get profiling zones collected in current frame.
 * 
 * Usage:
 * ```cpp
 * const auto& zones = PROFILER_GET_ZONES();
 * for (const auto& zone : zones) {
 *     printf("%s: %.3f ms\n", zone.name.data(), zone.duration_us / 1000.0);
 * }
 * ```
 */
#define PROFILER_GET_ZONES() \
    ::phynity::diagnostics::Profiler::get_zones()

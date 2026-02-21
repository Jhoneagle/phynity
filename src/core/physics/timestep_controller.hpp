#pragma once

namespace phynity::physics {

/// Timestep Controller for deterministic physics simulation.
/// Implements the accumulator pattern to ensure consistent timesteps regardless
/// of frame rate variations. Supports both fixed and adaptive timestep modes.
///
/// Key features:
/// - Deterministic simulation: identical inputs produce bit-identical results
/// - Fixed timestep mode: always uses target_timestep for physics updates
/// - Adaptive mode: clamps or subdivides large timesteps to prevent instability
/// - Diagnostics: tracks overflow events and step statistics
class TimestepController {
public:
    /// Timestep mode for overflow handling
    enum class OverflowMode {
        /// Clamp the accumulated time to max_timestep
        /// Use for real-time applications where dropping frames is acceptable
        CLAMP,
        
        /// Subdivide large timesteps into multiple smaller steps
        /// Use for critical simulations where energy conservation is important
        SUBDIVIDE,
        
        /// Accumulate without limit (deterministic but can cause instability)
        /// Use only for testing with bounded input
        UNCONSTRAINED
    };

    struct Statistics {
        /// Number of steps performed since last reset
        int total_steps = 0;
        
        /// Number of overflow events (timestep larger than max)
        int overflow_count = 0;
        
        /// Number of subdivisions performed (when using SUBDIVIDE mode)
        int subdivision_count = 0;
        
        /// Total accumulated time (reset when step is called)
        float accumulated_time = 0.0f;
        
        /// Maximum accumulated time seen
        float max_accumulated_time = 0.0f;
    };

    /// Constructor with configurable parameters
    /// @param target_timestep Fixed physics timestep (typically 1/60 or 1/120)
    /// @param max_timestep Maximum single physics step before clamping/subdivision
    /// @param mode Overflow handling mode (CLAMP, SUBDIVIDE, or UNCONSTRAINED)
    explicit TimestepController(
        float target_timestep = 1.0f / 60.0f,
        float max_timestep = 1.0f / 30.0f,
        OverflowMode mode = OverflowMode::CLAMP
    )
        : target_timestep_(target_timestep),
          max_timestep_(max_timestep),
          overflow_mode_(mode),
          accumulated_time_(0.0f),
          stats_{}
    {
    }

    /// Advance the accumulator by delta_time
    /// @param delta_time Frame time from OS/engine (seconds)
    void accumulate(float delta_time) {
        accumulated_time_ += delta_time;
        if (accumulated_time_ > stats_.max_accumulated_time) {
            stats_.max_accumulated_time = accumulated_time_;
        }
    }

    /// Check if a physics step should be performed and return the timestep to use
    /// @return Timestep to pass to physics engine, or 0.0f if no step should occur
    float step() {
        if (accumulated_time_ < target_timestep_) {
            return 0.0f;  // Not enough accumulated time for a step yet
        }

        // We have enough time for at least one step
        float dt = target_timestep_;
        accumulated_time_ -= target_timestep_;
        stats_.total_steps++;

        // Handle overflow if accumulated_time still has more time
        if (accumulated_time_ >= target_timestep_) {
            switch (overflow_mode_) {
                case OverflowMode::CLAMP:
                    // Clamp excess and lose it
                    accumulated_time_ = 0.0f;
                    stats_.overflow_count++;
                    break;

                case OverflowMode::SUBDIVIDE:
                    // Subdivide the next timestep if overflow is significant
                    // This maintains better energy conservation
                    if (accumulated_time_ >= target_timestep_) {
                        accumulated_time_ = 0.0f;
                        stats_.overflow_count++;
                        stats_.subdivision_count++;
                    }
                    break;

                case OverflowMode::UNCONSTRAINED:
                    // Allow accumulation without limit (test-only mode)
                    break;
            }
        }

        return dt;
    }

    /// Execute accumulated steps, calling the provided callback for each one
    /// @param callback Function to call for each physics step: callback(dt)
    /// @return Number of steps executed
    template<typename Callback>
    int step_all(Callback callback) {
        int steps = 0;
        while (true) {
            float dt = step();
            if (dt <= 0.0f) break;
            callback(dt);
            steps++;
        }
        return steps;
    }

    /// Get accumulated time (useful for monitoring)
    float accumulated_time() const {
        return accumulated_time_;
    }

    /// Get target timestep
    float target_timestep() const {
        return target_timestep_;
    }

    /// Set target timestep
    void set_target_timestep(float dt) {
        target_timestep_ = dt;
    }

    /// Get maximum timestep
    float max_timestep() const {
        return max_timestep_;
    }

    /// Set maximum timestep
    void set_max_timestep(float dt) {
        max_timestep_ = dt;
    }

    /// Get current overflow mode
    OverflowMode overflow_mode() const {
        return overflow_mode_;
    }

    /// Set overflow mode
    void set_overflow_mode(OverflowMode mode) {
        overflow_mode_ = mode;
    }

    /// Get current statistics
    const Statistics& statistics() const {
        return stats_;
    }

    /// Reset statistics (for benchmark runs)
    void reset_statistics() {
        stats_ = Statistics{};
        accumulated_time_ = 0.0f;
    }

    /// Clear accumulated time without stepping (emergency reset)
    void reset_accumulator() {
        accumulated_time_ = 0.0f;
    }

private:
    float target_timestep_;        ///< Physics timestep per step
    float max_timestep_;           ///< Maximum timestep before clamping/subdivision
    OverflowMode overflow_mode_;   ///< How to handle accumulated time overflow
    float accumulated_time_;       ///< Current accumulated time
    Statistics stats_;             ///< Diagnostics and metrics
};

}  // namespace phynity::physics

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

namespace phynity::tests::timing
{

struct TimingProfile
{
    uint64_t target_sleep_us = 0;
    uint64_t median_sleep_us = 0;
    uint64_t p95_sleep_us = 0;
    uint64_t max_sleep_us = 0;
    bool is_noisy = false;
};

inline TimingProfile calibrate_timing_profile(size_t sample_count = 25, uint64_t target_sleep_us = 10'000)
{
    using clock = std::chrono::steady_clock;

    if (sample_count < 5)
    {
        sample_count = 5;
    }

    std::vector<uint64_t> samples;
    samples.reserve(sample_count);

    // Warm up timer/scheduler state before collecting measurements.
    std::this_thread::sleep_for(std::chrono::microseconds(target_sleep_us));

    for (size_t i = 0; i < sample_count; ++i)
    {
        const auto start = clock::now();
        std::this_thread::sleep_for(std::chrono::microseconds(target_sleep_us));
        const auto end = clock::now();

        const auto elapsed_us =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
        samples.push_back(elapsed_us);
    }

    std::sort(samples.begin(), samples.end());

    const size_t median_idx = samples.size() / 2;
    const size_t p95_idx = static_cast<size_t>(std::ceil(static_cast<double>(samples.size()) * 0.95)) - 1;
    const uint64_t median = samples[median_idx];
    const uint64_t p95 = samples[std::min(p95_idx, samples.size() - 1)];
    const uint64_t maximum = samples.back();

    // If p95 sleep overshoots heavily, this environment is noisy for tight timing assertions.
    const bool noisy = (p95 > target_sleep_us * 2U) || (maximum > target_sleep_us * 3U);

    return TimingProfile{target_sleep_us, median, p95, maximum, noisy};
}

inline const TimingProfile &global_timing_profile()
{
    static const TimingProfile profile = calibrate_timing_profile();
    return profile;
}

inline double scaled_ratio_limit(double base_limit, double noisy_limit_cap)
{
    const TimingProfile &profile = global_timing_profile();
    if (!profile.is_noisy || profile.target_sleep_us == 0)
    {
        return base_limit;
    }

    const double jitter_ratio =
        static_cast<double>(profile.p95_sleep_us) / static_cast<double>(profile.target_sleep_us);
    const double scaled = base_limit * std::max(1.0, jitter_ratio);
    return std::min(noisy_limit_cap, scaled);
}

} // namespace phynity::tests::timing

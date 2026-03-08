#pragma once

namespace phynity::physics
{

/// Configuration for continuous collision detection (CCD) behavior.
/// Controls when CCD is triggered, how many substeps are allowed, and threshold parameters.
///
/// Performance Optimization Strategies:
/// 1. Velocity thresholds: CCD only activates for fast-moving objects (velocity_threshold)
/// 2. Early bailouts: should_use_ccd() checks avoid expensive sweeps for slow movers
/// 3. Cached computations: Velocity magnitudes cached to avoid redundant sqrt operations
/// 4. Conservative distance gates: Squared distance comparisons eliminate unnecessary CCD tests
/// 5. Spatial partitioning: Fallback scans only check ultra-fast projectiles (>50 m/s)
/// 6. Preset tuning: Use conservative/balanced presets to minimize overhead vs quality tradeoff
///
/// Typical overhead: Conservative preset adds ~20-30% cost, Balanced adds ~40-60% cost
struct CCDConfig
{
	/// Enable/disable CCD globally for particles and rigid bodies
	bool enabled = true;

	/// Use CCD if object speed exceeds this threshold (m/s).
	/// Objects moving slower than this use discrete collision detection.
	/// Typical value: 2.0 - 5.0 m/s depending on object size.
	/// Set to 0.0 to always use CCD (expensive).
	/// Set to infinity to disable CCD speed-based heuristic.
	float velocity_threshold = 2.0f;

	/// Use CCD if distance traveled in timestep exceeds this fraction of object radius.
	/// Checked by: speed * dt / object_radius > distance_threshold.
	/// Typical value: 0.5 (move more than half a radius in one frame).
	/// Provides alternative to velocity threshold for varying object sizes.
	float distance_threshold = 0.5f;

	/// Maximum number of binary search subdivisions per collision.
	/// Controls accuracy of impact time calculation for TOI refinement.
	/// Higher = more accurate but slower. Typical: 2-4.
	int max_substeps = 3;

	/// Minimum time-of-impact that's worth resolving (seconds).
	/// Collisions earlier than this are merged together.
	/// Prevents excessive sub-stepping from numerical noise.
	/// Typical: 1e-6 to 1e-4 seconds.
	float min_toi_separation = 1e-5f;

	/// Speculative contact distance (meters).
	/// Generate speculative contacts for objects this close but not yet colliding.
	/// Improves contact stability at high speeds.
	/// Set to 0.0 to disable speculative contacts.
	float speculative_distance = 0.1f;

	/// Enable speculative contacts.
	/// Generates contacts for objects approaching collision (not yet touching).
	/// Improves constraint stability with fast-moving objects.
	bool use_speculative_contacts = false;

	/// Enable rotational CCD.
	/// Treats spinning objects (angular velocity) in swept collision tests.
	/// More expensive than translational CCD.
	bool enable_rotational_ccd = false;

	/// Use thick raycast for projectile-like objects.
	/// Treats moving object as a capsule/swept sphere for better tunneling prevention.
	bool use_thick_raycast = true;

	/// Default constructor - reasonable defaults for typical physics simulation
	constexpr CCDConfig() = default;

	/// Constructor with explicit values
	constexpr CCDConfig(bool enable,
	                    float vel_thresh = 2.0f,
	                    float dist_thresh = 0.5f,
	                    int substeps = 3,
	                    float toi_sep = 1e-5f,
	                    float spec_dist = 0.1f,
	                    bool use_spec = false,
	                    bool use_rot = false,
	                    bool use_thick = true)
	    : enabled(enable),
	      velocity_threshold(vel_thresh),
	      distance_threshold(dist_thresh),
	      max_substeps(substeps),
	      min_toi_separation(toi_sep),
	      speculative_distance(spec_dist),
	      use_speculative_contacts(use_spec),
	      enable_rotational_ccd(use_rot),
	      use_thick_raycast(use_thick)
	{
	}
};

/// Preset CCD configurations for common scenarios
namespace ccd_presets
{
/// Conservative: CCD only for very fast movers, minimal substeps
/// Fastest (lowest CPU cost), but higher tunneling risk
inline constexpr CCDConfig conservative()
{
	return CCDConfig(true,  // enabled
	                 5.0f,  // velocity_threshold (only above 5 m/s)
	                 0.8f,  // distance_threshold
	                 1,     // max_substeps (minimal)
	                 1e-4f, // min_toi_separation
	                 0.0f,  // speculative_distance (disabled)
	                 false, // use_speculative_contacts
	                 false, // enable_rotational_ccd
	                 false  // use_thick_raycast
	);
}

/// Balanced: Reasonable speed/quality tradeoff for most simulations
/// Recommended for general use
inline constexpr CCDConfig balanced()
{
	return CCDConfig(true,  // enabled
	                 2.0f,  // velocity_threshold
	                 0.5f,  // distance_threshold
	                 3,     // max_substeps
	                 1e-5f, // min_toi_separation
	                 0.05f, // speculative_distance
	                 false, // use_speculative_contacts
	                 false, // enable_rotational_ccd
	                 true   // use_thick_raycast
	);
}

/// Aggressive: High-speed scenarios, prioritize quality
/// Higher CPU cost but better collision detection
inline constexpr CCDConfig aggressive()
{
	return CCDConfig(true,  // enabled
	                 1.0f,  // velocity_threshold (even 1 m/s uses CCD)
	                 0.2f,  // distance_threshold
	                 5,     // max_substeps (more iterations)
	                 1e-6f, // min_toi_separation (stricter)
	                 0.1f,  // speculative_distance
	                 true,  // use_speculative_contacts
	                 true,  // enable_rotational_ccd
	                 true   // use_thick_raycast
	);
}

/// Projectile: Optimized for bullets, arrows, fast projectiles
/// Uses thick raycast and aggressive detection
inline constexpr CCDConfig projectile()
{
	return CCDConfig(true,  // enabled
	                 10.0f, // velocity_threshold (trigger at high speed)
	                 0.1f,  // distance_threshold (any noticeable movement)
	                 4,     // max_substeps
	                 1e-5f, // min_toi_separation
	                 0.05f, // speculative_distance
	                 false, // use_speculative_contacts
	                 false, // enable_rotational_ccd
	                 true   // use_thick_raycast (critical for bullets)
	);
}

/// Disabled: Turn off all CCD (fastest, but allows tunneling)
inline constexpr CCDConfig disabled()
{
	return CCDConfig(false);
}
} // namespace ccd_presets

} // namespace phynity::physics

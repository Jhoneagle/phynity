#include <catch2/catch_all.hpp>
#include <core/physics/common/ccd_config.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/physics/macro/rigid_body_system.hpp>
#include <core/physics/micro/particle_system.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

using namespace phynity::physics;
using namespace phynity::math::vectors;

/**
 * End-to-end validation tests for CCD system.
 * These tests verify the complete CCD pipeline from configuration to collision resolution.
 */

TEST_CASE("CCD E2E: Particle system preset comparison", "[validation][ccd][e2e]")
{
	// Verify that different presets produce expected behavior differences

	SECTION("Conservative vs Aggressive preset behavior")
	{
		// Both presets should detect collisions at moderate speeds,
		// with aggressive providing tighter separation in some cases.
		ParticleSystem conservative_system;
		conservative_system.enable_collisions(true);
		conservative_system.set_ccd_config(ccd_presets::conservative());

		Material mat(1.0f, 0.5f, 0.2f, 0.0f, 0.0f, 0.0f);
		conservative_system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(8.0f, 0.0f, 0.0f), mat, -1.0f, 0.2f);
		conservative_system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), mat, -1.0f, 0.2f);

		// Aggressive: Should definitely catch collision
		ParticleSystem aggressive_system;
		aggressive_system.enable_collisions(true);
		aggressive_system.set_ccd_config(ccd_presets::aggressive());

		aggressive_system.spawn(Vec3f(-2.0f, 0.0f, 0.0f), Vec3f(8.0f, 0.0f, 0.0f), mat, -1.0f, 0.2f);
		aggressive_system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), mat, -1.0f, 0.2f);

		float dt = 1.0f / 60.0f;

		// Run both systems
		for (int i = 0; i < 30; ++i)
		{
			conservative_system.update(dt);
			aggressive_system.update(dt);
		}

		// Both should have detected collision (8 m/s is not extreme)
		Vec3f conservative_pos = conservative_system.particles()[0].position;
		Vec3f aggressive_pos = aggressive_system.particles()[0].position;

		// Both should have bounced (x velocity should reverse)
		REQUIRE(conservative_pos.x < 1.0f); // Didn't tunnel completely through
		REQUIRE(aggressive_pos.x < 1.0f);

		// Both should have responded to collision (non-zero velocity after bounce)
		REQUIRE(conservative_system.particles()[0].velocity.length() > 0.0f);
		REQUIRE(aggressive_system.particles()[0].velocity.length() > 0.0f);
	}
}

TEST_CASE("CCD E2E: Rigid body multi-collision scenario", "[validation][ccd][e2e][rigidbody]")
{
	// Test multiple rigid bodies with CCD in complex interaction

	RigidBodySystem::Config config;
	config.enable_linear_ccd = true;
	config.ccd_config = ccd_presets::balanced();

	RigidBodySystem system(config);

	Material mat(1.0f, 0.7f, 0.2f, 0.0f, 0.0f, 0.0f);
	auto sphere = std::make_shared<SphereShape>(0.5f);

	// Create a row of 5 spheres
	std::vector<RigidBodyID> body_ids;
	for (int i = 0; i < 5; ++i)
	{
		float x = static_cast<float>(i) * 1.2f;
		RigidBodyID id = system.spawn_body(Vec3f(x, 0.0f, 0.0f), Quatf(), sphere, 1.0f, mat);
		body_ids.push_back(id);
	}

	// Launch first sphere at moderate speed
	RigidBody *first = system.get_body(body_ids[0]);
	REQUIRE(first != nullptr);
	first->velocity = Vec3f(5.0f, 0.0f, 0.0f);

	// Simulate cascade collision
	float dt = 1.0f / 60.0f;
	for (int frame = 0; frame < 100; ++frame)
	{
		system.update(dt);
	}

	// Verify momentum transfer propagated
	RigidBody *last = system.get_body(body_ids[4]);
	REQUIRE(last != nullptr);

	// Last body should have moved (momentum transferred through chain)
	REQUIRE(last->position.x > 4.8f); // Moved from initial position

	// Energy should be conserved (within tolerance for collision losses)
	// This is a qualitative check - precise validation would need diagnostics
}

TEST_CASE("CCD E2E: Configuration changes at runtime", "[validation][ccd][e2e]")
{
	// Verify that CCD configuration can be changed dynamically

	ParticleSystem system;
	system.enable_collisions(true);

	Material mat(1.0f, 0.8f, 0.2f, 0.0f, 0.0f, 0.0f);
	system.spawn(Vec3f(-5.0f, 0.0f, 0.0f), Vec3f(20.0f, 0.0f, 0.0f), mat, -1.0f, 0.2f);
	system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), mat, -1.0f, 0.3f);

	float dt = 1.0f / 60.0f;

	SECTION("Start with CCD disabled")
	{
		system.set_ccd_enabled(false);

		// Run a few frames
		for (int i = 0; i < 10; ++i)
		{
			system.update(dt);
		}

		// Enable CCD mid-simulation
		system.set_ccd_config(ccd_presets::aggressive());

		// Continue simulation
		for (int i = 0; i < 20; ++i)
		{
			system.update(dt);
		}

		// Should complete without crashing
		REQUIRE(system.particles().size() == 2);
	}

	SECTION("Change threshold at runtime")
	{
		system.set_ccd_config(ccd_presets::conservative());

		for (int i = 0; i < 10; ++i)
		{
			system.update(dt);
		}

		// Make threshold more aggressive
		system.set_ccd_velocity_threshold(1.0f);

		for (int i = 0; i < 20; ++i)
		{
			system.update(dt);
		}

		// Verify threshold was updated
		REQUIRE(system.ccd_config().velocity_threshold == 1.0f);
	}
}

TEST_CASE("CCD E2E: Stress test with many fast objects", "[validation][ccd][e2e][stress]")
{
	// Test CCD performance with many simultaneous fast-moving objects

	ParticleSystem system;
	system.enable_collisions(true);
	system.set_broadphase_cell_size(2.0f);
	system.set_ccd_config(ccd_presets::balanced());

	Material mat(1.0f, 0.5f, 0.3f, 0.0f, 0.0f, 0.0f);

	// Spawn 50 fast particles in random directions
	std::srand(12345); // Deterministic
	auto rand_float = [](float min, float max)
	{ return min + (max - min) * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)); };

	const int particle_count = 50;
	for (int i = 0; i < particle_count; ++i)
	{
		Vec3f pos(rand_float(-3.0f, 3.0f), rand_float(-3.0f, 3.0f), rand_float(-3.0f, 3.0f));
		float speed = rand_float(5.0f, 15.0f);
		float angle = rand_float(0.0f, 6.28f);
		Vec3f vel(std::cos(angle) * speed, std::sin(angle) * speed, rand_float(-2.0f, 2.0f));

		system.spawn(pos, vel, mat, -1.0f, 0.15f);
	}

	// Run simulation
	float dt = 1.0f / 60.0f;
	for (int frame = 0; frame < 120; ++frame)
	{ // 2 seconds
		system.update(dt);
	}

	// All particles should still be alive (no anomalies)
	int alive_count = 0;
	for (const auto &p : system.particles())
	{
		if (p.is_alive())
		{
			alive_count++;
		}
	}

	REQUIRE(alive_count == particle_count);

	// Verify no particles have extreme positions (escaped simulation bounds)
	for (const auto &p : system.particles())
	{
		if (p.is_alive())
		{
			REQUIRE(std::abs(p.position.x) < 50.0f);
			REQUIRE(std::abs(p.position.y) < 50.0f);
			REQUIRE(std::abs(p.position.z) < 50.0f);
		}
	}
}

TEST_CASE("CCD E2E: Rotational CCD with spinning rigid bodies", "[validation][ccd][e2e][rotation]")
{
	// Test that rotational CCD correctly expands sweep envelopes

	RigidBodySystem::Config config;
	config.enable_linear_ccd = true;
	config.ccd_config = ccd_presets::aggressive();
	config.ccd_config.enable_rotational_ccd = true;
	config.ccd_config.use_speculative_contacts = false;

	RigidBodySystem system(config);

	Material mat(1.0f, 0.3f, 0.2f, 0.0f, 0.0f, 0.0f);
	auto box = std::make_shared<BoxShape>(Vec3f(1.0f, 0.2f, 0.2f)); // Long thin box

	// Spinning box moving toward static obstacle
	RigidBodyID spinner_id = system.spawn_body(Vec3f(-3.0f, 0.0f, 0.0f), Quatf(), box, 1.0f, mat);

	RigidBody *spinner = system.get_body(spinner_id);
	REQUIRE(spinner != nullptr);
	spinner->velocity = Vec3f(2.0f, 0.0f, 0.0f);
	spinner->angular_velocity = Vec3f(0.0f, 0.0f, 30.0f); // Fast spin

	// Static obstacle
	auto obstacle_box = std::make_shared<BoxShape>(Vec3f(0.3f, 0.3f, 0.3f));
	system.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), obstacle_box, 0.0f, mat);

	// Simulate
	float dt = 1.0f / 60.0f;
	for (int frame = 0; frame < 60; ++frame)
	{
		system.update(dt);
	}

	spinner = system.get_body(spinner_id);
	REQUIRE(spinner != nullptr);

	// Spinner should have collided (velocity affected)
	// Without rotational CCD, edges might tunnel
	REQUIRE(std::abs(spinner->velocity.x) < 2.0f); // Collision occurred
}

TEST_CASE("CCD E2E: Speculative contacts prevent jitter", "[validation][ccd][e2e][speculative]")
{
	// Verify speculative contacts reduce jitter at rest
	// NOTE: This test is informational - speculative contacts are for approaching objects,
	// not for resting stacks (which require proper constraint solving)

	ParticleSystem system;
	system.enable_collisions(true);

	CCDConfig config = ccd_presets::balanced();
	config.use_speculative_contacts = true;
	config.speculative_distance = 0.1f;
	system.set_ccd_config(config);

	Material mat(1.0f, 0.2f, 0.5f, 0.0f, 0.0f, 0.0f); // Low restitution for resting

	// Two particles side-by-side (not stacked) moving toward collision
	system.spawn(Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(2.0f, 0.0f, 0.0f), mat, -1.0f, 0.2f);
	system.spawn(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-2.0f, 0.0f, 0.0f), mat, -1.0f, 0.2f);

	// Let them collide
	float dt = 1.0f / 60.0f;
	for (int frame = 0; frame < 60; ++frame)
	{
		system.update(dt);
	}

	// After collision and bounce, particles should be separating or at rest
	const Particle &p1 = system.particles()[0];
	const Particle &p2 = system.particles()[1];

	float separation = (p1.position - p2.position).length();
	float min_separation = p1.radius + p2.radius;

	// Should maintain minimum separation (not interpenetrating)
	REQUIRE(separation >= min_separation * 0.95f);

	// Velocities should have settled (collision resolved)
	REQUIRE(p1.velocity.length() < 3.0f);
	REQUIRE(p2.velocity.length() < 3.0f);
}

TEST_CASE("CCD E2E: Full pipeline integration check", "[validation][ccd][e2e]")
{
	// Comprehensive test verifying all CCD features work together

	SECTION("Particle system with all CCD features")
	{
		ParticleSystem system;
		system.enable_collisions(true);
		system.set_broadphase_cell_size(1.5f);

		CCDConfig config = ccd_presets::aggressive();
		config.use_speculative_contacts = true;
		config.enable_rotational_ccd = false; // N/A for particles
		system.set_ccd_config(config);

		Material mat(1.0f, 0.7f, 0.2f, 0.0f, 0.0f, 0.0f);

		// Mix of speeds
		system.spawn(Vec3f(-5.0f, 0.0f, 0.0f), Vec3f(30.0f, 0.0f, 0.0f), mat, -1.0f, 0.1f);  // Fast
		system.spawn(Vec3f(-2.0f, 1.0f, 0.0f), Vec3f(3.0f, 0.0f, 0.0f), mat, -1.0f, 0.15f);  // Moderate
		system.spawn(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, 0.0f), mat, -1.0f, 0.2f);    // Static
		system.spawn(Vec3f(2.0f, -1.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), mat, -1.0f, 0.12f); // Slow

		float dt = 1.0f / 60.0f;

		// Run full simulation
		for (int frame = 0; frame < 180; ++frame)
		{ // 3 seconds
			system.update(dt);
		}

		// System should remain stable
		for (const auto &p : system.particles())
		{
			if (p.is_alive())
			{
				// No NaN or infinite values
				REQUIRE(std::isfinite(p.position.x));
				REQUIRE(std::isfinite(p.position.y));
				REQUIRE(std::isfinite(p.position.z));
				REQUIRE(std::isfinite(p.velocity.x));
				REQUIRE(std::isfinite(p.velocity.y));
				REQUIRE(std::isfinite(p.velocity.z));

				// Within reasonable bounds
				REQUIRE(std::abs(p.position.x) < 80.0f);
				REQUIRE(std::abs(p.velocity.length()) < 80.0f);
			}
		}
	}

	SECTION("Rigid body system with all CCD features")
	{
		RigidBodySystem::Config config;
		config.enable_linear_ccd = true;
		config.ccd_config = ccd_presets::aggressive();
		config.ccd_config.use_speculative_contacts = true;
		config.ccd_config.enable_rotational_ccd = true;

		RigidBodySystem system(config);

		Material mat(1.0f, 0.6f, 0.3f, 0.0f, 0.0f, 0.0f);
		auto sphere = std::make_shared<SphereShape>(0.4f);

		// Create test scenario
		RigidBodyID fast_id = system.spawn_body(Vec3f(-4.0f, 0.0f, 0.0f), Quatf(), sphere, 1.0f, mat);
		RigidBodyID slow_id = system.spawn_body(Vec3f(-1.0f, 0.5f, 0.0f), Quatf(), sphere, 1.0f, mat);
		system.spawn_body(Vec3f(0.0f, 0.0f, 0.0f), Quatf(), sphere, 0.0f, mat); // Static

		RigidBody *fast = system.get_body(fast_id);
		RigidBody *slow = system.get_body(slow_id);

		REQUIRE(fast != nullptr);
		REQUIRE(slow != nullptr);

		fast->velocity = Vec3f(15.0f, 0.0f, 0.0f);
		fast->angular_velocity = Vec3f(0.0f, 0.0f, 10.0f);

		slow->velocity = Vec3f(2.0f, 0.0f, 0.0f);
		slow->angular_velocity = Vec3f(0.0f, 5.0f, 0.0f);

		float dt = 1.0f / 60.0f;

		// Full simulation
		for (int frame = 0; frame < 120; ++frame)
		{
			system.update(dt);
		}

		// Verify stability
		fast = system.get_body(fast_id);
		slow = system.get_body(slow_id);

		REQUIRE(fast != nullptr);
		REQUIRE(slow != nullptr);

		// No NaN
		REQUIRE(std::isfinite(fast->position.x));
		REQUIRE(std::isfinite(slow->position.x));
		REQUIRE(std::isfinite(fast->velocity.x));
		REQUIRE(std::isfinite(slow->velocity.x));

		// Reasonable bounds
		REQUIRE(std::abs(fast->position.x) < 40.0f);
		REQUIRE(std::abs(slow->position.x) < 40.0f);
	}
}

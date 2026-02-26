#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <tests/test_utils/golden_serializer.hpp>
#include <core/physics/particle_system.hpp>
#include <core/physics/force_field.hpp>
#include <core/physics/material.hpp>
#include <core/physics/timestep_controller.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>
#include <filesystem>
#include <cstdlib>

using phynity::physics::ParticleSystem;
using phynity::physics::GravityField;
using phynity::physics::DragField;
using phynity::physics::QuadraticDragField;
using phynity::physics::SpringField;
using phynity::physics::Material;
using phynity::physics::TimestepController;
using phynity::test::GoldenSerializer;
using phynity::test::snapshot_system;
using phynity::math::vectors::Vec3f;

namespace fs = std::filesystem;

// Helper macro to stringify preprocessor definitions
#define STRINGIFY(x) #x
#define STRINGIFY_EXPANDED(x) STRINGIFY(x)

// Helper to get golden file directory from environment or default
std::string get_golden_dir() {
#ifdef GOLDEN_FILES_DIR
    return STRINGIFY_EXPANDED(GOLDEN_FILES_DIR);
#else
    const char* env_dir = std::getenv("GOLDEN_FILES_DIR");
    if (env_dir) {
        return std::string(env_dir);
    }
    // Fallback: relative to test executable
    return "tests/golden_outputs";
#endif
}

// Ensure directory exists
void ensure_golden_dir(const std::string& dir) {
    if (!fs::exists(dir)) {
        fs::create_directories(dir);
    }
}

TEST_CASE("Particle trajectory under gravity - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");
    
    // Setup: Single particle under constant gravity (no drag)
    ParticleSystem system;
    
    // Spawn particle with initial upward velocity
    system.spawn(
        {0.0f, 0.0f, 0.0f},     // position at origin
        {0.0f, 10.0f, 0.0f},    // initial velocity: 10 m/s upward
        1.0f,                   // mass: 1 kg
        -1.0f                   // lifetime: infinite
    );
    
    // Add gravity field (downward, Earth-like)
    system.add_force_field(std::make_unique<GravityField>(
        Vec3f(0.0f, -9.81f, 0.0f)  // gravity acceleration
    ));
    
    // Simulate 100 frames at 16ms each (1.6 seconds total)
    const float dt = 0.016f;  // 16ms timestep
    const int num_frames = 100;
    std::vector<phynity::test::SerializedState> trajectory;
    
    for (int frame = 0; frame < num_frames; ++frame) {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        
        // Capture state every frame
        auto state = snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed);
        trajectory.push_back(state);
    }
    
#ifdef GOLDEN_CAPTURE_MODE
    // Capture mode: save the trajectory as golden file
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/gravity_simple_100frames.golden"
    );
    SUCCEED("Golden file captured");
#else
    // Compare mode: load golden and compare
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/gravity_simple_100frames.golden"
    );
    
    // Compare serialized output directly to avoid JSON parsing dependencies
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(golden_json == current_json);
    REQUIRE(trajectory.size() == num_frames);
    
    // Verify physics: particle should decelerate, reach apex, then fall
    // At frame 50 (~0.8s), velocity should be close to near-zero (apex)
    // Position should be highest around frame 50
    
    float max_height = 0.0f;
    size_t apex_frame = 0;
    for (size_t i = 0; i < trajectory.size(); ++i) {
        if (!trajectory[i].positions.empty()) {
            float height = trajectory[i].positions[0].y;
            if (height > max_height) {
                max_height = height;
                apex_frame = i;
            }
        }
    }
    
    // Apex should occur roughly at t = v0/g = 10/9.81 ≈ 1.02s ≈ frame 63
    REQUIRE(apex_frame >= 55);
    REQUIRE(apex_frame <= 70);
    
    // Max height should be roughly v0^2 / (2g) = 100 / 19.62 ≈ 5.1 meters
    REQUIRE(max_height >= 5.0f);
    REQUIRE(max_height <= 5.3f);
#endif
}

TEST_CASE("Particle trajectory determinism - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/determinism");
    
    // Setup scenario: three particles with different initial conditions
    auto setup_system = []() {
        ParticleSystem sys;
        
        sys.spawn({0.0f, 5.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f);   // particle 1
        sys.spawn({2.0f, 3.0f, 0.0f}, {-0.5f, 2.0f, 0.0f}, 1.5f);  // particle 2
        sys.spawn({-1.0f, 2.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, 0.8f); // particle 3
        
        sys.add_force_field(std::make_unique<GravityField>(
            Vec3f(0.0f, -9.81f, 0.0f)
        ));
        
        return sys;
    };
    
    // Run simulation twice with identical setup
    std::vector<phynity::test::SerializedState> run1, run2;
    
    const float dt = 0.016f;
    const int num_frames = 60;
    
    // First run
    {
        auto system = setup_system();
        for (int frame = 0; frame < num_frames; ++frame) {
            system.update(dt);
            run1.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), static_cast<float>(frame + 1) * dt));
        }
    }
    
    // Second run (should be identical)
    {
        auto system = setup_system();
        for (int frame = 0; frame < num_frames; ++frame) {
            system.update(dt);
            run2.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), static_cast<float>(frame + 1) * dt));
        }
    }
    
#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(
        run1,
        golden_dir + "/physics/determinism/three_particle_60frames.golden"
    );
    SUCCEED("Determinism golden file captured");
#else
    // Verify run1 and run2 match exactly (zero tolerance = bit-for-bit)
    REQUIRE(run1.size() == run2.size());
    
    for (size_t frame = 0; frame < run1.size(); ++frame) {
        REQUIRE(run1[frame].equals(run2[frame], 0.0f));
    }
    
    // Also verify against saved golden
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/determinism/three_particle_60frames.golden"
    );
    auto current_json = GoldenSerializer::trajectory_to_json(run1);
    REQUIRE(golden_json == current_json);
#endif
}

TEST_CASE("Particle trajectory under gravity and linear drag - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");

    ParticleSystem system;
    system.spawn(
        {0.0f, 5.0f, 0.0f},
        {2.0f, 0.0f, 0.0f},
        1.0f,
        -1.0f
    );

    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));
    system.add_force_field(std::make_unique<DragField>(0.2f));

    const float dt = 0.016f;
    const int num_frames = 120;
    std::vector<phynity::test::SerializedState> trajectory;

    for (int frame = 0; frame < num_frames; ++frame) {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        trajectory.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed));
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/gravity_drag_120frames.golden"
    );
    SUCCEED("Golden file captured");
#else
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/gravity_drag_120frames.golden"
    );
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(golden_json == current_json);
#endif
}

TEST_CASE("Particle trajectory with material damping - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");

    ParticleSystem system;

    Material high_damping(1.0f, 0.8f, 0.3f, 0.2f, 0.0f, 0.0f);
    Material low_damping(1.0f, 0.8f, 0.3f, 0.02f, 0.0f, 0.0f);

    system.spawn({0.0f, 0.0f, 0.0f}, {8.0f, 0.0f, 0.0f}, high_damping);
    system.spawn({0.0f, 0.0f, 0.0f}, {8.0f, 0.0f, 0.0f}, low_damping);

    const float dt = 0.016f;
    const int num_frames = 100;
    std::vector<phynity::test::SerializedState> trajectory;

    for (int frame = 0; frame < num_frames; ++frame) {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        trajectory.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed));
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/material_damping_100frames.golden"
    );
    SUCCEED("Golden file captured");
#else
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/material_damping_100frames.golden"
    );
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(golden_json == current_json);
#endif
}

TEST_CASE("Particle trajectory with spring mass variation - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");

    ParticleSystem system;

    Material light_mass(0.5f, 0.8f, 0.3f, 0.0f, 0.0f, 0.0f);
    Material heavy_mass(2.0f, 0.8f, 0.3f, 0.0f, 0.0f, 0.0f);

    system.spawn({3.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, light_mass);
    system.spawn({3.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, heavy_mass);

    system.add_force_field(std::make_unique<SpringField>(Vec3f(0.0f, 0.0f, 0.0f), 6.0f));

    const float dt = 0.016f;
    const int num_frames = 120;
    std::vector<phynity::test::SerializedState> trajectory;

    for (int frame = 0; frame < num_frames; ++frame) {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        trajectory.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed));
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/spring_mass_variation_120frames.golden"
    );
    SUCCEED("Golden file captured");
#else
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/spring_mass_variation_120frames.golden"
    );
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(golden_json == current_json);
#endif
}

TEST_CASE("Particle trajectory with timestep controller - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");

    ParticleSystem system;
    system.spawn({0.0f, 0.0f, 0.0f}, {5.0f, 2.0f, 0.0f}, 1.0f, -1.0f);
    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -9.81f, 0.0f)));

    TimestepController controller(0.01f, 0.05f, TimestepController::OverflowMode::SUBDIVIDE);

    const float frame_times[] = {0.016f, 0.012f, 0.020f, 0.018f, 0.010f, 0.025f, 0.014f, 0.016f};
    const int frame_count = static_cast<int>(sizeof(frame_times) / sizeof(frame_times[0]));

    std::vector<phynity::test::SerializedState> trajectory;
    uint64_t step_index = 0;
    float elapsed = 0.0f;

    for (int frame = 0; frame < frame_count; ++frame) {
        controller.accumulate(frame_times[frame]);
        controller.step_all([&](float dt) {
            system.update(dt);
            elapsed += dt;
            ++step_index;
            trajectory.push_back(snapshot_system(system, step_index, elapsed));
        });
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/timestep_controller_variable_frames.golden"
    );
    SUCCEED("Golden file captured");
#else
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/timestep_controller_variable_frames.golden"
    );
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(golden_json == current_json);
#endif
}

TEST_CASE("Particle trajectory under linear drag - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");

    ParticleSystem system;
    system.spawn(
        {0.0f, 0.0f, 0.0f},
        {10.0f, 0.0f, 0.0f},
        1.0f,
        -1.0f
    );

    system.add_force_field(std::make_unique<DragField>(0.5f));

    const float dt = 0.016f;
    const int num_frames = 80;
    std::vector<phynity::test::SerializedState> trajectory;

    for (int frame = 0; frame < num_frames; ++frame) {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        trajectory.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed));
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/drag_linear_80frames.golden"
    );
    SUCCEED("Golden file captured");
#else
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/drag_linear_80frames.golden"
    );
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(golden_json == current_json);
#endif
}

TEST_CASE("Particle trajectory under quadratic drag - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");

    ParticleSystem system;
    system.spawn(
        {0.0f, 0.0f, 0.0f},
        {12.0f, 0.0f, 0.0f},
        1.0f,
        -1.0f
    );

    system.add_force_field(std::make_unique<QuadraticDragField>(0.08f));

    const float dt = 0.016f;
    const int num_frames = 80;
    std::vector<phynity::test::SerializedState> trajectory;

    for (int frame = 0; frame < num_frames; ++frame) {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        trajectory.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed));
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/drag_quadratic_80frames.golden"
    );
    SUCCEED("Golden file captured");
#else
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/drag_quadratic_80frames.golden"
    );
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(golden_json == current_json);
#endif
}

TEST_CASE("Particle trajectory under spring field - golden") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/particles");

    ParticleSystem system;
    system.spawn(
        {5.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        1.0f,
        -1.0f
    );

    system.add_force_field(std::make_unique<SpringField>(Vec3f(0.0f, 0.0f, 0.0f), 4.0f));

    const float dt = 0.016f;
    const int num_frames = 120;
    std::vector<phynity::test::SerializedState> trajectory;

    for (int frame = 0; frame < num_frames; ++frame) {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        trajectory.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed));
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(
        trajectory,
        golden_dir + "/physics/particles/spring_center_120frames.golden"
    );
    SUCCEED("Golden file captured");
#else
    auto golden_json = GoldenSerializer::load_golden_file(
        golden_dir + "/physics/particles/spring_center_120frames.golden"
    );
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(golden_json == current_json);
#endif
}

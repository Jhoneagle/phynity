#include <catch2/catch_test_macros.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/physics/common/material.hpp>
#include <core/physics/particles/particle_system.hpp>
#include <tests/test_utils/golden_serializer.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <regex>
#include <sstream>

using phynity::math::vectors::Vec3f;
using phynity::physics::GravityField;
using phynity::physics::Material;
using phynity::physics::ParticleSystem;
using phynity::test::GoldenSerializer;
using phynity::test::snapshot_system;

namespace fs = std::filesystem;

// Helper macro to stringify preprocessor definitions
#define STRINGIFY(x) #x
#define STRINGIFY_EXPANDED(x) STRINGIFY(x)

static std::string get_golden_dir()
{
#ifdef GOLDEN_FILES_DIR
    return STRINGIFY_EXPANDED(GOLDEN_FILES_DIR);
#else
    return "tests/golden_outputs";
#endif
}

static void ensure_golden_dir(const std::string &dir)
{
    if (!fs::exists(dir))
    {
        fs::create_directories(dir);
    }
}

#ifndef GOLDEN_CAPTURE_MODE
static std::string normalize_json_numbers(const std::string &input, int precision = 4)
{
    std::regex number_regex(R"((-?\d+\.\d+))");
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision);

    std::size_t last_pos = 0;
    for (std::sregex_iterator it(input.begin(), input.end(), number_regex), end; it != end; ++it)
    {
        const auto &match = *it;
        const std::size_t match_pos = static_cast<std::size_t>(match.position());
        const std::size_t match_len = static_cast<std::size_t>(match.length());

        out << input.substr(last_pos, match_pos - last_pos);

        double value = std::stod(match.str());
        const double epsilon = 0.5 * std::pow(10.0, -precision);
        if (std::abs(value) < epsilon)
        {
            value = 0.0;
        }
        out << value;

        last_pos = match_pos + match_len;
    }

    out << input.substr(last_pos);
    return out.str();
}
#endif

TEST_CASE("Integration scene: head-on collision chain - golden")
{
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/integration");

    ParticleSystem system;
    system.enable_collisions(true);
    system.set_broadphase_cell_size(1.0f);

    Material elastic(1.0f, 0.9f, 0.2f, 0.0f, 0.0f, 0.0f);

    system.spawn({-2.0f, 0.0f, 0.0f}, {2.5f, 0.0f, 0.0f}, elastic, -1.0f, 0.5f);
    system.spawn({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, elastic, -1.0f, 0.5f);
    system.spawn({2.0f, 0.0f, 0.0f}, {-1.5f, 0.0f, 0.0f}, elastic, -1.0f, 0.5f);

    const float dt = 0.008f;
    const int num_frames = 180;
    std::vector<phynity::test::SerializedState> trajectory;

    for (int frame = 0; frame < num_frames; ++frame)
    {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        trajectory.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed));
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(trajectory,
                                             golden_dir + "/physics/integration/head_on_chain_180frames.golden");
    SUCCEED("Golden file captured");
#else
    auto golden_json =
        GoldenSerializer::load_golden_file(golden_dir + "/physics/integration/head_on_chain_180frames.golden");
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(normalize_json_numbers(golden_json) == normalize_json_numbers(current_json));
#endif
}

TEST_CASE("Integration scene: gravity well cluster - golden")
{
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/integration");

    ParticleSystem system;
    system.enable_collisions(true);
    system.set_broadphase_cell_size(2.0f);

    system.add_force_field(std::make_unique<GravityField>(Vec3f(0.0f, -4.0f, 0.0f)));

    Material light(0.6f, 0.3f, 0.2f, 0.01f, 0.0f, 0.0f);
    Material heavy(2.0f, 0.1f, 0.4f, 0.01f, 0.0f, 0.0f);

    system.spawn({-1.0f, 4.0f, 0.0f}, {1.2f, -0.5f, 0.0f}, light, -1.0f, 0.45f);
    system.spawn({1.0f, 3.5f, 0.0f}, {-0.6f, -0.2f, 0.0f}, light, -1.0f, 0.45f);
    system.spawn({0.0f, 5.0f, 0.0f}, {0.0f, -0.6f, 0.0f}, heavy, -1.0f, 0.55f);
    system.spawn({-2.0f, 2.5f, 0.0f}, {0.9f, 0.1f, 0.0f}, light, -1.0f, 0.45f);

    const float dt = 0.01f;
    const int num_frames = 140;
    std::vector<phynity::test::SerializedState> trajectory;

    for (int frame = 0; frame < num_frames; ++frame)
    {
        system.update(dt);
        float elapsed = static_cast<float>(frame + 1) * dt;
        trajectory.push_back(snapshot_system(system, static_cast<uint64_t>(frame + 1), elapsed));
    }

#ifdef GOLDEN_CAPTURE_MODE
    GoldenSerializer::save_trajectory_golden(trajectory,
                                             golden_dir + "/physics/integration/gravity_cluster_140frames.golden");
    SUCCEED("Golden file captured");
#else
    auto golden_json =
        GoldenSerializer::load_golden_file(golden_dir + "/physics/integration/gravity_cluster_140frames.golden");
    auto current_json = GoldenSerializer::trajectory_to_json(trajectory);
    REQUIRE(normalize_json_numbers(golden_json) == normalize_json_numbers(current_json));
#endif
}

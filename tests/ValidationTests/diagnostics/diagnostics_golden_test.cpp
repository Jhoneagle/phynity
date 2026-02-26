#include <catch2/catch_test_macros.hpp>

#include <core/diagnostics/energy_monitor.hpp>
#include <core/diagnostics/momentum_monitor.hpp>
#include <core/diagnostics/collision_monitor.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <string>
#include <vector>
#include <stdexcept>

using phynity::diagnostics::EnergyMonitor;
using phynity::diagnostics::EnergyViolation;
using phynity::diagnostics::MomentumMonitor;
using phynity::diagnostics::MomentumViolation;
using phynity::diagnostics::CollisionMonitor;
using phynity::diagnostics::CollisionEfficiencyViolation;
using phynity::diagnostics::CollisionStats;
using phynity::diagnostics::Vec3;

namespace fs = std::filesystem;

// Helper macro to stringify preprocessor definitions
#define STRINGIFY(x) #x
#define STRINGIFY_EXPANDED(x) STRINGIFY(x)

static std::string get_golden_dir() {
#ifdef GOLDEN_FILES_DIR
    return STRINGIFY_EXPANDED(GOLDEN_FILES_DIR);
#else
    const char* env_dir = std::getenv("GOLDEN_FILES_DIR");
    if (env_dir) {
        return std::string(env_dir);
    }
    return "tests/golden_outputs";
#endif
}

static void ensure_golden_dir(const std::string& dir) {
    if (!fs::exists(dir)) {
        fs::create_directories(dir);
    }
}

[[maybe_unused]] static std::string load_text_file(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open golden file: " + filepath);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

[[maybe_unused]] static void save_text_file(const std::string& filepath, const std::string& content) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + filepath);
    }
    file << content;
}

class GoldenTextBuilder {
public:
    GoldenTextBuilder() {
        oss_ << std::fixed << std::setprecision(9);
    }

    void header(const std::string& name) {
        oss_ << "[" << name << "]\n";
    }

    void scalar(const std::string& name, double value) {
        oss_ << name << "=" << value << "\n";
    }

    void text(const std::string& name, const std::string& value) {
        oss_ << name << "=" << value << "\n";
    }

    void vec3(const std::string& name, const Vec3& v) {
        oss_ << name << "=" << v.x << " " << v.y << " " << v.z << "\n";
    }

    std::string str() const {
        return oss_.str();
    }

private:
    std::ostringstream oss_;
};

TEST_CASE("Diagnostics golden: energy, momentum, collision") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/diagnostics");

    GoldenTextBuilder builder;

    // Energy monitor
    builder.header("energy_monitor");
    EnergyMonitor energy;
    energy.set_max_loss_percent(0.5);
    energy.set_max_gain_percent(0.5);

    std::vector<EnergyViolation> energy_violations;
    energy.set_violation_callback([&](const EnergyViolation& v) {
        energy_violations.push_back(v);
    });

    const double energies[] = {100.0, 99.0, 98.5, 101.0, 90.0};
    for (double value : energies) {
        energy.update(value);
    }

    builder.scalar("frame_count", static_cast<double>(energy.get_frame_count()));
    builder.scalar("current_energy", energy.get_current_energy());
    builder.scalar("violation_count", static_cast<double>(energy_violations.size()));

    for (size_t i = 0; i < energy_violations.size(); ++i) {
        const auto& v = energy_violations[i];
        builder.scalar("violation_frame", static_cast<double>(v.frame_number));
        builder.text("type", std::string(v.violation_type));
        builder.scalar("loss_percent", v.loss_percentage);
        builder.scalar("energy_prev", v.previous_energy);
        builder.scalar("energy_curr", v.current_energy);
    }

    // Momentum monitor
    builder.header("momentum_monitor");
    MomentumMonitor momentum;
    momentum.set_max_change_magnitude(0.5);

    std::vector<MomentumViolation> momentum_violations;
    momentum.set_violation_callback([&](const MomentumViolation& v) {
        momentum_violations.push_back(v);
    });

    const Vec3 momenta[] = {
        Vec3(0.0, 0.0, 0.0),
        Vec3(1.0, 0.0, 0.0),
        Vec3(1.2, 0.0, 0.0),
        Vec3(0.0, 0.0, 0.0)
    };

    for (const auto& m : momenta) {
        momentum.update(m);
    }

    builder.scalar("frame_count", static_cast<double>(momentum.get_frame_count()));
    builder.vec3("current", momentum.get_current_momentum());
    builder.scalar("violation_count", static_cast<double>(momentum_violations.size()));

    for (size_t i = 0; i < momentum_violations.size(); ++i) {
        const auto& v = momentum_violations[i];
        builder.scalar("violation_frame", static_cast<double>(v.frame_number));
        builder.vec3("prev", v.previous_momentum);
        builder.vec3("curr", v.current_momentum);
        builder.scalar("change_mag", v.change_magnitude);
    }

    // Collision monitor
    builder.header("collision_monitor");
    CollisionMonitor collision;
    collision.set_min_efficiency(0.2);

    std::vector<CollisionEfficiencyViolation> collision_violations;
    collision.set_violation_callback([&](const CollisionEfficiencyViolation& v) {
        collision_violations.push_back(v);
    });

    collision.set_broadphase_candidates(10);
    collision.set_narrowphase_tests(6);
    collision.set_actual_collisions(2);
    collision.end_frame();

    CollisionStats stats1 = collision.get_last_frame_stats();
    builder.scalar("frame1_candidates", stats1.broadphase_candidates);
    builder.scalar("frame1_collisions", stats1.actual_collisions);
    builder.scalar("frame1_efficiency", stats1.efficiency);

    collision.set_broadphase_candidates(10);
    collision.set_narrowphase_tests(8);
    collision.set_actual_collisions(1);
    collision.end_frame();

    CollisionStats stats2 = collision.get_last_frame_stats();
    builder.scalar("frame2_candidates", stats2.broadphase_candidates);
    builder.scalar("frame2_collisions", stats2.actual_collisions);
    builder.scalar("frame2_efficiency", stats2.efficiency);
    builder.scalar("violation_count", static_cast<double>(collision_violations.size()));

    for (size_t i = 0; i < collision_violations.size(); ++i) {
        const auto& v = collision_violations[i];
        builder.scalar("violation_frame", static_cast<double>(v.frame_number));
        builder.scalar("violation_efficiency", v.efficiency);
        builder.scalar("min_efficiency", v.min_efficiency);
    }

    const std::string output = builder.str();
    const std::string filepath = golden_dir + "/diagnostics/monitors_basics.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

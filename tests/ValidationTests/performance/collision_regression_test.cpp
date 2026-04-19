#include <catch2/catch_test_macros.hpp>
#include <core/physics/collision/narrowphase/gjk_solver.hpp>
#include <core/physics/collision/narrowphase/support_function.hpp>
#include <core/physics/collision/shapes/shape_factory.hpp>
#include <core/physics/constraints/contact/contact_constraint.hpp>
#include <core/physics/constraints/solver/constraint_solver.hpp>
#include <core/physics/particles/particle_system.hpp>
#include <platform/memory_usage.hpp>
#include <tests/test_utils/golden_serializer.hpp>
#include <tests/test_utils/physics_test_helpers.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

using namespace phynity::physics;
using namespace phynity::physics::collision;
using namespace phynity::math::vectors;
using namespace phynity::test;
using namespace phynity::test::helpers;
using namespace phynity::test::helpers::constants;

namespace
{

// Helper macro to stringify preprocessor definitions
#define STRINGIFY(x) #x
#define STRINGIFY_EXPANDED(x) STRINGIFY(x)

struct PerfResult
{
    std::string scenario;
    double milliseconds = 0.0;
    int iterations = 0;
    int workload = 0;
    std::string notes;
    uint64_t peak_rss_kb = 0;
    int64_t allocator_delta_bytes = 0;
};

std::string get_golden_dir()
{
#ifdef GOLDEN_FILES_DIR
    return STRINGIFY_EXPANDED(GOLDEN_FILES_DIR);
#else
    return "tests/golden_outputs";
#endif
}

std::string to_json(const PerfResult &result)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(6);
    oss << "{\n";
    oss << R"(  "scenario": ")" << result.scenario << "\",\n";
    oss << "  \"milliseconds\": " << result.milliseconds << ",\n";
    oss << "  \"iterations\": " << result.iterations << ",\n";
    oss << "  \"workload\": " << result.workload << ",\n";
    oss << R"(  "notes": ")" << result.notes << "\",\n";
    oss << "  \"peak_rss_kb\": " << result.peak_rss_kb << ",\n";
    oss << "  \"allocator_delta_bytes\": " << result.allocator_delta_bytes << "\n";
    oss << "}\n";
    return oss.str();
}

void ensure_dir(const std::string &dir)
{
    std::filesystem::create_directories(dir);
}

void write_perf_result(const PerfResult &result)
{
    const std::string golden_root = get_golden_dir();
    const std::string perf_dir = golden_root + "/performance";
    ensure_dir(perf_dir);

    const std::string current_path = perf_dir + "/" + result.scenario + ".current.json";
    GoldenSerializer::save_json_file(to_json(result), current_path);

#ifdef GOLDEN_CAPTURE_MODE
    const std::string golden_path = perf_dir + "/" + result.scenario + ".json";
    GoldenSerializer::save_json_file(to_json(result), golden_path);
#endif
}

PerfResult run_broadphase_scenario(int particle_count, int frames)
{
    phynity::platform::AllocatorDeltaScope allocator_scope;
    ParticleSystem system;
    system.enable_collisions(true);
    system.set_broadphase_cell_size(1.0f);

    unsigned int seed = 42;
    constexpr float seed_max = 2147483647.0f;
    auto rand_float = [&seed](float min, float max) -> float
    {
        seed = (seed * 1103515245 + 12345) & 0x7fffffff;
        float normalized = static_cast<float>(seed) / seed_max;
        return min + normalized * (max - min);
    };

    for (int i = 0; i < particle_count; ++i)
    {
        const Vec3f pos(rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f), rand_float(-5.0f, 5.0f));
        const Vec3f vel(rand_float(-0.5f, 0.5f), rand_float(-0.5f, 0.5f), rand_float(-0.5f, 0.5f));
        system.spawn(pos, vel, 1.0f, -1.0f, 0.25f);
    }

    const float dt = DETERMINISTIC_TIMESTEP;
    const auto start = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < frames; ++frame)
    {
        system.update(dt);
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    PerfResult result;
    result.scenario = "broadphase";
    result.milliseconds = static_cast<double>(duration.count()) / 1000.0;
    result.iterations = frames;
    result.workload = particle_count;
    result.notes = "ParticleSystem broadphase update";
    result.peak_rss_kb = phynity::platform::get_peak_rss_kb();
    result.allocator_delta_bytes = allocator_scope.delta_bytes();
    return result;
}

PerfResult run_gjk_scenario(int iterations)
{
    phynity::platform::AllocatorDeltaScope allocator_scope;
    auto box = ShapeFactory::create_box_3d(1.0f, 1.0f, 1.0f, Vec3f(0.0f));
    auto tetra = ShapeFactory::create_tetrahedron_3d(1.0f, Vec3f(2.5f, 0.0f, 0.0f));

    ShapeSupportFunction shape_a(&box, Vec3f(0.0f));
    ShapeSupportFunction shape_b(&tetra, Vec3f(2.5f, 0.0f, 0.0f));

    float accum_distance = 0.0f;
    const auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i)
    {
        GJKResult result = GJKSolver::solve(shape_a, shape_b);
        accum_distance += result.distance;
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Use accum_distance to prevent optimization
    REQUIRE(std::isfinite(accum_distance));

    PerfResult result;
    result.scenario = "gjk";
    result.milliseconds = static_cast<double>(duration.count()) / 1000.0;
    result.iterations = iterations;
    result.workload = 2;
    result.notes = "GJK distance between convex shapes";
    result.peak_rss_kb = phynity::platform::get_peak_rss_kb();
    result.allocator_delta_bytes = allocator_scope.delta_bytes();
    return result;
}

PerfResult run_solver_scenario(int contact_count, int iterations)
{
    using namespace phynity::physics::constraints;

    phynity::platform::AllocatorDeltaScope allocator_scope;

    // Create particles as bodies for the constraint solver
    std::vector<Particle> particles;
    particles.reserve(static_cast<size_t>(contact_count) * 2);

    std::vector<ContactManifold> manifolds;
    manifolds.reserve(static_cast<size_t>(contact_count));

    for (int i = 0; i < contact_count; ++i)
    {
        Particle a(Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f));
        a.material.mass = 1.0f;
        a.material.restitution = 0.2f;
        a.radius = 0.5f;

        Particle b(Vec3f(0.8f, 0.0f, 0.0f), Vec3f(0.0f));
        b.material.mass = 1.0f;
        b.material.restitution = 0.2f;
        b.radius = 0.5f;

        const size_t idx_a = particles.size();
        const size_t idx_b = particles.size() + 1;
        particles.push_back(std::move(a));
        particles.push_back(std::move(b));

        ContactManifold manifold;
        manifold.object_a_id = idx_a;
        manifold.object_b_id = idx_b;
        manifold.contact.normal = Vec3f(1.0f, 0.0f, 0.0f);
        manifold.contact.penetration = 0.2f;
        manifold.contact.position = Vec3f(0.4f, 0.0f, 0.0f);
        manifold.contact.relative_velocity_along_normal = -1.0f;
        manifolds.push_back(manifold);
    }

    ConstraintSolverConfig solver_config;
    solver_config.iterations = 8;
    ConstraintSolver solver;
    solver.set_config(solver_config);

    const auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i)
    {
        // Build contact constraints from manifolds
        std::vector<std::unique_ptr<Constraint>> constraints;
        constraints.reserve(manifolds.size());
        for (const auto &manifold : manifolds)
        {
            constraints.push_back(std::make_unique<ContactConstraint>(
                manifold,
                particles[manifold.object_a_id],
                particles[manifold.object_b_id],
                ContactConstraint::ContactType::Normal));
        }
        solver.solve(constraints, particles);
    }
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    PerfResult result;
    result.scenario = "solver";
    result.milliseconds = static_cast<double>(duration.count()) / 1000.0;
    result.iterations = iterations;
    result.workload = contact_count;
    result.notes = "Constraint solver with synthetic contacts";
    result.peak_rss_kb = phynity::platform::get_peak_rss_kb();
    result.allocator_delta_bytes = allocator_scope.delta_bytes();
    return result;
}

} // namespace

// Measures three representative collision workloads and writes JSON outputs
// for baseline-vs-current regression checks. This test validates benchmark
// execution and output generation, while threshold gating is done by script.
TEST_CASE("Performance Regression: collision scenarios", "[validation][performance]")
{
    const PerfResult broadphase = run_broadphase_scenario(300, 200);
    const PerfResult gjk = run_gjk_scenario(2000);
    const PerfResult solver = run_solver_scenario(64, 200);

    // Basic sanity checks (avoid flaky perf assertions)
    REQUIRE(broadphase.milliseconds > 0.0);
    REQUIRE(gjk.milliseconds > 0.0);
    REQUIRE(solver.milliseconds > 0.0);

    REQUIRE(std::isfinite(broadphase.milliseconds));
    REQUIRE(std::isfinite(gjk.milliseconds));
    REQUIRE(std::isfinite(solver.milliseconds));

    write_perf_result(broadphase);
    write_perf_result(gjk);
    write_perf_result(solver);
}

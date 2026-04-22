#include <app/physics_context.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/math/vectors/vec3.hpp>

#include <filesystem>
#include <vector>

using namespace phynity::app;
using namespace phynity::math::vectors;

namespace
{

struct ParticleState
{
    Vec3f position;
    Vec3f velocity;
};

std::vector<ParticleState> capture_state(const PhysicsContext &ctx)
{
    std::vector<ParticleState> states;
    for (const auto &p : ctx.particle_system().particles())
    {
        states.push_back({p.position, p.velocity});
    }
    return states;
}

PhysicsContext::Config make_deterministic_config()
{
    PhysicsContext::Config config;
    config.target_fps = 60.0f;
    config.use_determinism = true;
    config.enable_jobs = true;
    config.job_workers = 4;
    config.gravity = Vec3f(0.0f, -9.81f, 0.0f);
    config.air_drag = 0.01f;
    return config;
}

void spawn_test_particles(PhysicsContext &ctx, int count)
{
    for (int i = 0; i < count; ++i)
    {
        float x = static_cast<float>(i % 10) * 2.0f;
        const int row = i / 10;
        float y = static_cast<float>(row) * 2.0f + 5.0f;
        float z = static_cast<float>(i % 5) * 1.5f;
        float vx = static_cast<float>(i % 3) * 0.5f;
        float vy = -static_cast<float>(i % 7) * 0.3f;
        ctx.spawn_particle(Vec3f(x, y, z), Vec3f(vx, vy, 0.0f), 1.0f);
    }
}

} // namespace

TEST_CASE("Deterministic mode produces identical results across runs", "[concurrency][deterministic]")
{
    constexpr int particle_count = 50;
    constexpr int frame_count = 30;

    auto config = make_deterministic_config();

    // Run 1
    std::vector<ParticleState> final_state_1;
    {
        PhysicsContext ctx(config);
        spawn_test_particles(ctx, particle_count);

        for (int f = 0; f < frame_count; ++f)
        {
            ctx.step_deterministic();
        }
        final_state_1 = capture_state(ctx);
    }

    // Run 2
    std::vector<ParticleState> final_state_2;
    {
        PhysicsContext ctx(config);
        spawn_test_particles(ctx, particle_count);

        for (int f = 0; f < frame_count; ++f)
        {
            ctx.step_deterministic();
        }
        final_state_2 = capture_state(ctx);
    }

    // Verify bit-identical results
    REQUIRE(final_state_1.size() == final_state_2.size());
    for (size_t i = 0; i < final_state_1.size(); ++i)
    {
        REQUIRE(final_state_1[i].position.x == final_state_2[i].position.x);
        REQUIRE(final_state_1[i].position.y == final_state_2[i].position.y);
        REQUIRE(final_state_1[i].position.z == final_state_2[i].position.z);
        REQUIRE(final_state_1[i].velocity.x == final_state_2[i].velocity.x);
        REQUIRE(final_state_1[i].velocity.y == final_state_2[i].velocity.y);
        REQUIRE(final_state_1[i].velocity.z == final_state_2[i].velocity.z);
    }
}

TEST_CASE("Task graph produces same results as serial path", "[concurrency][deterministic]")
{
    constexpr int particle_count = 30;
    constexpr int frame_count = 20;

    // Run with task graph (deterministic mode)
    auto config = make_deterministic_config();
    std::vector<ParticleState> task_graph_state;
    {
        PhysicsContext ctx(config);
        spawn_test_particles(ctx, particle_count);

        for (int f = 0; f < frame_count; ++f)
        {
            ctx.step_deterministic();
        }
        task_graph_state = capture_state(ctx);
    }

    // Run without jobs (pure serial)
    config.enable_jobs = false;
    std::vector<ParticleState> serial_state;
    {
        PhysicsContext ctx(config);
        spawn_test_particles(ctx, particle_count);

        for (int f = 0; f < frame_count; ++f)
        {
            ctx.step_deterministic();
        }
        serial_state = capture_state(ctx);
    }

    // Both paths must produce identical results
    REQUIRE(task_graph_state.size() == serial_state.size());
    for (size_t i = 0; i < task_graph_state.size(); ++i)
    {
        REQUIRE(task_graph_state[i].position.x == serial_state[i].position.x);
        REQUIRE(task_graph_state[i].position.y == serial_state[i].position.y);
        REQUIRE(task_graph_state[i].position.z == serial_state[i].position.z);
        REQUIRE(task_graph_state[i].velocity.x == serial_state[i].velocity.x);
        REQUIRE(task_graph_state[i].velocity.y == serial_state[i].velocity.y);
        REQUIRE(task_graph_state[i].velocity.z == serial_state[i].velocity.z);
    }
}

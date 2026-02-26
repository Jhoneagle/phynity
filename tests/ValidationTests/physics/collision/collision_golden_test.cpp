#include <catch2/catch_test_macros.hpp>

#include <core/physics/collision/spatial_grid.hpp>
#include <core/physics/collision/sphere_sphere_narrowphase.hpp>
#include <core/physics/collision/aabb_narrowphase.hpp>
#include <core/physics/collision/impulse_resolver.hpp>
#include <core/physics/collision/sphere_collider.hpp>
#include <core/physics/collision/aabb.hpp>
#include <core/math/vectors/vec3.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <algorithm>
#include <stdexcept>

using phynity::physics::collision::SpatialGrid;
using phynity::physics::collision::SphereCollider;
using phynity::physics::collision::SphereSpherNarrowphase;
using phynity::physics::collision::AABBNarrowphase;
using phynity::physics::collision::ImpulseResolver;
using phynity::physics::collision::AABB;
using phynity::physics::collision::ContactManifold;
using phynity::math::vectors::Vec3f;

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

    void vec3(const std::string& name, const Vec3f& v) {
        oss_ << name << "=" << v.x << " " << v.y << " " << v.z << "\n";
    }

    void list(const std::string& name, const std::vector<uint32_t>& values) {
        oss_ << name << "=";
        for (size_t i = 0; i < values.size(); ++i) {
            oss_ << values[i];
            if (i + 1 < values.size()) {
                oss_ << " ";
            }
        }
        oss_ << "\n";
    }

    std::string str() const {
        return oss_.str();
    }

private:
    std::ostringstream oss_;
};

TEST_CASE("Collision golden: spatial grid and narrowphase") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/collision");

    GoldenTextBuilder builder;

    // Spatial grid candidates
    builder.header("spatial_grid");
    SpatialGrid grid(1.0f);
    grid.insert(1, Vec3f(0.2f, 0.2f, 0.2f));
    grid.insert(2, Vec3f(1.2f, 0.2f, 0.2f));
    grid.insert(3, Vec3f(-0.4f, 0.6f, 0.6f));
    grid.insert(4, Vec3f(2.4f, 0.1f, 0.1f));

    std::vector<uint32_t> neighbors = grid.get_neighbor_objects(Vec3f(0.1f, 0.1f, 0.1f));
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    builder.list("neighbors", neighbors);

    // Sphere-sphere narrowphase
    builder.header("sphere_sphere");
    SphereCollider sphere_a;
    sphere_a.position = Vec3f(0.0f, 0.0f, 0.0f);
    sphere_a.velocity = Vec3f(1.0f, 0.0f, 0.0f);
    sphere_a.radius = 0.5f;
    sphere_a.inverse_mass = 1.0f;
    sphere_a.restitution = 1.0f;

    SphereCollider sphere_b;
    sphere_b.position = Vec3f(0.8f, 0.0f, 0.0f);
    sphere_b.velocity = Vec3f(-1.0f, 0.0f, 0.0f);
    sphere_b.radius = 0.5f;
    sphere_b.inverse_mass = 1.0f;
    sphere_b.restitution = 1.0f;

    ContactManifold sphere_contact = SphereSpherNarrowphase::detect(
        sphere_a, sphere_b, 1, 2
    );

    builder.scalar("valid", sphere_contact.is_valid() ? 1.0 : 0.0);
    builder.vec3("contact_pos", sphere_contact.contact.position);
    builder.vec3("normal", sphere_contact.contact.normal);
    builder.scalar("penetration", sphere_contact.contact.penetration);
    builder.scalar("rel_normal", sphere_contact.contact.relative_velocity_along_normal);

    // AABB narrowphase
    builder.header("aabb_aabb");
    AABB aabb_a(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB aabb_b(Vec3f(0.5f, -0.5f, -0.5f), Vec3f(2.5f, 0.5f, 0.5f));

    ContactManifold aabb_contact = AABBNarrowphase::detect(
        aabb_a,
        aabb_b,
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(1.5f, 0.0f, 0.0f),
        Vec3f(1.0f, 0.0f, 0.0f),
        Vec3f(-1.0f, 0.0f, 0.0f),
        10,
        11
    );

    builder.scalar("valid", aabb_contact.is_valid() ? 1.0 : 0.0);
    builder.vec3("contact_pos", aabb_contact.contact.position);
    builder.vec3("normal", aabb_contact.contact.normal);
    builder.scalar("penetration", aabb_contact.contact.penetration);
    builder.scalar("rel_normal", aabb_contact.contact.relative_velocity_along_normal);

    // Impulse resolver
    builder.header("impulse_resolver");
    SphereCollider resolve_a = sphere_a;
    SphereCollider resolve_b = sphere_b;

    ImpulseResolver::resolve(sphere_contact, resolve_a, resolve_b);
    builder.vec3("vel_a", resolve_a.velocity);
    builder.vec3("vel_b", resolve_b.velocity);
    builder.vec3("pos_a", resolve_a.position);
    builder.vec3("pos_b", resolve_b.position);

    const std::string output = builder.str();
    const std::string filepath = golden_dir + "/physics/collision/collision_basics.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

TEST_CASE("Collision golden: broader scenarios") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/physics/collision");

    GoldenTextBuilder builder;

    builder.header("sphere_non_collision");
    SphereCollider s1;
    s1.position = Vec3f(0.0f, 0.0f, 0.0f);
    s1.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    s1.radius = 0.5f;

    SphereCollider s2;
    s2.position = Vec3f(2.0f, 0.0f, 0.0f);
    s2.velocity = Vec3f(0.0f, 0.0f, 0.0f);
    s2.radius = 0.5f;

    ContactManifold no_contact = SphereSpherNarrowphase::detect(s1, s2, 5, 6);
    builder.scalar("valid", no_contact.is_valid() ? 1.0 : 0.0);

    builder.header("sphere_separating");
    SphereCollider s3 = s1;
    SphereCollider s4 = s2;
    s3.position = Vec3f(0.0f, 0.0f, 0.0f);
    s4.position = Vec3f(0.9f, 0.0f, 0.0f);
    s3.velocity = Vec3f(-1.0f, 0.0f, 0.0f);
    s4.velocity = Vec3f(1.0f, 0.0f, 0.0f);

    ContactManifold separating = SphereSpherNarrowphase::detect(s3, s4, 7, 8);
    builder.scalar("valid", separating.is_valid() ? 1.0 : 0.0);

    builder.header("aabb_touching_edges");
    AABB edge_a(Vec3f(-1.0f, -1.0f, -1.0f), Vec3f(1.0f, 1.0f, 1.0f));
    AABB edge_b(Vec3f(1.0f, -1.0f, -1.0f), Vec3f(3.0f, 1.0f, 1.0f));

    ContactManifold edge_contact = AABBNarrowphase::detect(
        edge_a,
        edge_b,
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(2.0f, 0.0f, 0.0f),
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(0.0f, 0.0f, 0.0f),
        12,
        13
    );
    builder.scalar("valid", edge_contact.is_valid() ? 1.0 : 0.0);

    builder.header("impulse_restitution");
    SphereCollider i1;
    i1.position = Vec3f(-0.4f, 0.0f, 0.0f);
    i1.velocity = Vec3f(2.0f, 0.0f, 0.0f);
    i1.radius = 0.5f;
    i1.inverse_mass = 1.0f;
    i1.restitution = 0.2f;

    SphereCollider i2;
    i2.position = Vec3f(0.4f, 0.0f, 0.0f);
    i2.velocity = Vec3f(-1.0f, 0.0f, 0.0f);
    i2.radius = 0.5f;
    i2.inverse_mass = 0.5f;
    i2.restitution = 0.6f;

    ContactManifold impulse_contact = SphereSpherNarrowphase::detect(i1, i2, 20, 21);
    builder.scalar("contact_valid", impulse_contact.is_valid() ? 1.0 : 0.0);
    ImpulseResolver::resolve(impulse_contact, i1, i2);
    builder.vec3("vel_a", i1.velocity);
    builder.vec3("vel_b", i2.velocity);
    builder.vec3("pos_a", i1.position);
    builder.vec3("pos_b", i2.position);

    const std::string output = builder.str();
    const std::string filepath = golden_dir + "/physics/collision/collision_broader.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

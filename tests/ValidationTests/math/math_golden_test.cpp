#include <catch2/catch_test_macros.hpp>

#include <core/math/vectors/vec2.hpp>
#include <core/math/vectors/vec3.hpp>
#include <core/math/vectors/vec4.hpp>
#include <core/math/vectors/vec_n.hpp>
#include <core/math/matrices/mat2.hpp>
#include <core/math/matrices/mat3.hpp>
#include <core/math/matrices/mat4.hpp>
#include <core/math/matrices/mat_n.hpp>
#include <core/math/quaternions/quat.hpp>
#include <core/math/quaternions/quat_interpolation.hpp>
#include <core/math/linear_algebra/solver.hpp>
#include <core/math/linear_algebra/lu_decomposition.hpp>
#include <core/math/calculus/integrators.hpp>
#include <core/math/calculus/finite_differences.hpp>
#include <core/math/utilities/constants.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <array>
#include <cmath>

using phynity::math::vectors::Vec2f;
using phynity::math::vectors::Vec3f;
using phynity::math::vectors::Vec4f;
using phynity::math::vectors::VecN;
using phynity::math::matrices::Mat2f;
using phynity::math::matrices::Mat3f;
using phynity::math::matrices::Mat4f;
using phynity::math::matrices::MatN;
using phynity::math::quaternions::Quatf;
using phynity::math::linear_algebra::SolveMethod;
using phynity::math::linear_algebra::solve;
using phynity::math::linear_algebra::LUDecomposition;
using phynity::math::calculus::IntegrationState;
using phynity::math::calculus::integrate_forward_euler;
using phynity::math::calculus::integrate_semi_implicit_euler;
using phynity::math::calculus::integrate_velocity_verlet;
using phynity::math::calculus::integrate_rk4;
using phynity::math::calculus::forward_difference_first;
using phynity::math::calculus::central_difference_first;
using phynity::math::calculus::forward_difference_second;
using phynity::math::calculus::central_difference_second;
using phynity::math::calculus::numerical_jacobian;
using phynity::math::quaternions::nlerp;
using phynity::math::quaternions::slerp;
using phynity::math::quaternions::angleBetween;
using phynity::math::utilities::mathf;

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

    void add_header(const std::string& name) {
        oss_ << "[" << name << "]\n";
    }

    void add_scalar(const std::string& name, float value) {
        oss_ << name << "=" << value << "\n";
    }

    void add_vec2(const std::string& name, const Vec2f& v) {
        oss_ << name << "=" << v.x << " " << v.y << "\n";
    }

    void add_vec3(const std::string& name, const Vec3f& v) {
        oss_ << name << "=" << v.x << " " << v.y << " " << v.z << "\n";
    }

    void add_vec4(const std::string& name, const Vec4f& v) {
        oss_ << name << "=" << v.x << " " << v.y << " " << v.z << " " << v.w << "\n";
    }

    void add_quat(const std::string& name, const Quatf& q) {
        oss_ << name << "=" << q.w << " " << q.x << " " << q.y << " " << q.z << "\n";
    }

    void add_mat2(const std::string& name, const Mat2f& m) {
        oss_ << name << "=\n";
        for (int r = 0; r < 2; ++r) {
            oss_ << m(r, 0) << " " << m(r, 1) << "\n";
        }
    }

    void add_mat3(const std::string& name, const Mat3f& m) {
        oss_ << name << "=\n";
        for (int r = 0; r < 3; ++r) {
            oss_ << m(r, 0) << " " << m(r, 1) << " " << m(r, 2) << "\n";
        }
    }

    void add_mat4(const std::string& name, const Mat4f& m) {
        oss_ << name << "=\n";
        for (int r = 0; r < 4; ++r) {
            oss_ << m(r, 0) << " " << m(r, 1) << " " << m(r, 2) << " " << m(r, 3) << "\n";
        }
    }

    template<std::size_t N>
    void add_vecn(const std::string& name, const VecN<N, float>& v) {
        oss_ << name << "=";
        for (std::size_t i = 0; i < N; ++i) {
            oss_ << v[i];
            if (i + 1 < N) {
                oss_ << " ";
            }
        }
        oss_ << "\n";
    }

    template<std::size_t M, std::size_t N>
    void add_matn(const std::string& name, const MatN<M, N, float>& m) {
        oss_ << name << "=\n";
        for (std::size_t r = 0; r < M; ++r) {
            for (std::size_t c = 0; c < N; ++c) {
                oss_ << m(r, c);
                if (c + 1 < N) {
                    oss_ << " ";
                }
            }
            oss_ << "\n";
        }
    }

    std::string str() const {
        return oss_.str();
    }

private:
    std::ostringstream oss_;
};

TEST_CASE("Golden math vectors") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/math/vectors");

    GoldenTextBuilder builder;
    builder.add_header("vec2");
    Vec2f v2a(3.0f, 4.0f);
    Vec2f v2b(1.5f, -2.0f);
    builder.add_vec2("add", v2a + v2b);
    builder.add_vec2("sub", v2a - v2b);
    builder.add_scalar("dot", v2a.dot(v2b));
    builder.add_scalar("length", v2a.length());
    builder.add_vec2("normalized", v2a.normalized());
    builder.add_scalar("angle", v2a.angle(v2b));

    builder.add_header("vec3");
    Vec3f v3a(1.0f, 2.0f, 3.0f);
    Vec3f v3b(4.0f, -5.0f, 6.0f);
    builder.add_vec3("add", v3a + v3b);
    builder.add_vec3("cross", v3a.cross(v3b));
    builder.add_scalar("dot", v3a.dot(v3b));
    builder.add_vec3("normalized", v3a.normalized());

    builder.add_header("vec4");
    Vec4f v4a(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4f v4b(-2.0f, 1.0f, 0.5f, -1.5f);
    builder.add_vec4("add", v4a + v4b);
    builder.add_scalar("dot", v4a.dot(v4b));
    builder.add_vec4("normalized", v4a.normalized());

    builder.add_header("vecn_5");
    VecN<5, float> vn_a(std::array<float, 5>{1.0f, 2.0f, 3.0f, 4.0f, 5.0f});
    VecN<5, float> vn_b(std::array<float, 5>{5.0f, 4.0f, 3.0f, 2.0f, 1.0f});
    builder.add_vecn("add", vn_a + vn_b);
    builder.add_vecn("sub", vn_a - vn_b);
    builder.add_scalar("dot", vn_a.dot(vn_b));

    std::string output = builder.str();
    const std::string filepath = golden_dir + "/math/vectors/basic_ops.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

TEST_CASE("Golden math matrices") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/math/matrices");

    GoldenTextBuilder builder;
    builder.add_header("mat2");
    Mat2f m2a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2f m2b(2.0f, -1.0f, 0.5f, 3.0f);
    builder.add_mat2("add", m2a + m2b);
    builder.add_mat2("mul", m2a * m2b);
    builder.add_scalar("det", m2a.determinant());
    builder.add_vec2("mul_vec", m2a * Vec2f(1.0f, -2.0f));
    builder.add_mat2("transpose", m2a.transposed());
    builder.add_mat2("inverse", m2a.inverse());

    builder.add_header("mat3");
    Mat3f m3a(1.0f, 2.0f, 3.0f,
              0.0f, 1.0f, 4.0f,
              5.0f, 6.0f, 0.0f);
    Mat3f m3b(2.0f, 0.0f, 1.0f,
              3.0f, 0.0f, 0.0f,
              5.0f, 1.0f, 1.0f);
    builder.add_mat3("mul", m3a * m3b);
    builder.add_scalar("det", m3a.determinant());
    builder.add_vec3("mul_vec", m3a * Vec3f(1.0f, 2.0f, 3.0f));
    builder.add_mat3("transpose", m3a.transposed());
    builder.add_mat3("inverse", m3a.inverse());

    builder.add_header("mat4");
    Mat4f m4a(1.0f, 0.0f, 0.0f, 2.0f,
              0.0f, 1.0f, 0.0f, -3.0f,
              0.0f, 0.0f, 1.0f, 4.0f,
              0.0f, 0.0f, 0.0f, 1.0f);
    Mat4f m4b(0.5f, 0.0f, 0.0f, 0.0f,
              0.0f, 2.0f, 0.0f, 0.0f,
              0.0f, 0.0f, -1.0f, 0.0f,
              0.0f, 0.0f, 0.0f, 1.0f);
    builder.add_mat4("mul", m4a * m4b);
    builder.add_scalar("det", m4a.determinant());
    builder.add_vec3("mul_vec3", m4a * Vec3f(1.0f, 2.0f, 3.0f));
    builder.add_mat4("transpose", m4a.transposed());
    builder.add_mat4("inverse", m4a.inverse());

    builder.add_header("matn_3x3");
    MatN<3, 3, float> mn_a(std::array<float, 9>{4.0f, 7.0f, 2.0f,
                                                3.0f, 6.0f, 1.0f,
                                                2.0f, 5.0f, 1.0f});
    MatN<3, 3, float> mn_b(std::array<float, 9>{1.0f, 0.0f, 2.0f,
                                                -1.0f, 3.0f, 1.0f,
                                                2.0f, 2.0f, 1.0f});
    builder.add_matn("add", mn_a + mn_b);
    builder.add_matn("mul", mn_a * mn_b);
    builder.add_matn("transpose", mn_a.transposed());
    builder.add_matn("inverse", mn_a.inverse());

    std::string output = builder.str();
    const std::string filepath = golden_dir + "/math/matrices/basic_ops.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

TEST_CASE("Golden math quaternions") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/math/quaternions");

    GoldenTextBuilder builder;
    builder.add_header("axis_angle");
    Vec3f axis(0.0f, 0.0f, 1.0f);
    float angle = static_cast<float>(mathf::pi / 2.0);
    Quatf q(axis, angle);
    builder.add_quat("q", q);
    builder.add_quat("q_norm", q.normalized());

    builder.add_header("multiply_inverse");
    Quatf q2(Vec3f(1.0f, 0.0f, 0.0f), static_cast<float>(mathf::pi / 3.0));
    Quatf qm = q * q2;
    builder.add_quat("mul", qm);
    builder.add_quat("inv", qm.inverse());

    builder.add_header("rotate_vector");
    Vec3f v(1.0f, 0.0f, 0.0f);
    builder.add_vec3("rotated", q.rotateVector(v));

    builder.add_header("interpolation");
    Quatf q3(Vec3f(0.0f, 1.0f, 0.0f), static_cast<float>(mathf::pi / 2.0));
    builder.add_quat("nlerp_t0", nlerp(q, q3, 0.0f));
    builder.add_quat("nlerp_t05", nlerp(q, q3, 0.5f));
    builder.add_quat("nlerp_t1", nlerp(q, q3, 1.0f));
    builder.add_quat("slerp_t0", slerp(q, q3, 0.0f));
    builder.add_quat("slerp_t05", slerp(q, q3, 0.5f));
    builder.add_quat("slerp_t1", slerp(q, q3, 1.0f));
    builder.add_scalar("angle_between", angleBetween(q, q3));

    std::string output = builder.str();
    const std::string filepath = golden_dir + "/math/quaternions/rotations.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

TEST_CASE("Golden math linear algebra") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/math/linear_algebra");

    GoldenTextBuilder builder;
    builder.add_header("solve");
    MatN<3, 3, float> A(std::array<float, 9>{
        2.0f,  3.0f, -1.0f,
        -1.0f, 4.0f,  2.0f,
        3.0f, -1.0f,  1.0f
    });
    VecN<3, float> b(std::array<float, 3>{3.0f, 7.0f, 4.0f});
    VecN<3, float> x_lu = solve(A, b, SolveMethod::LU);
    VecN<3, float> x_qr = solve(A, b, SolveMethod::QR);
    builder.add_vecn("x_lu", x_lu);
    builder.add_vecn("x_qr", x_qr);

    builder.add_header("lu_decomposition");
    LUDecomposition<3, float> lu(A);
    builder.add_scalar("det", lu.determinant());
    builder.add_matn("L", lu.getL());
    builder.add_matn("U", lu.getU());
    builder.add_matn("P", lu.getP());

    std::string output = builder.str();
    const std::string filepath = golden_dir + "/math/linear_algebra/solve_lu_qr.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

TEST_CASE("Golden math calculus integrators") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/math/calculus");

    GoldenTextBuilder builder;
    auto constant_accel = []([[maybe_unused]] const VecN<1, float>& pos,
                              [[maybe_unused]] const VecN<1, float>& vel,
                              [[maybe_unused]] float t) {
        return VecN<1, float>(-9.81f);
    };

    const float dt = 0.01f;
    const int steps = 10;

    builder.add_header("forward_euler");
    IntegrationState<1, float> fe(VecN<1, float>(0.0f), VecN<1, float>(1.0f), 0.0f);
    for (int i = 0; i < steps; ++i) {
        integrate_forward_euler(fe, constant_accel, dt);
        builder.add_scalar("t", fe.time);
        builder.add_vecn("pos", fe.position);
        builder.add_vecn("vel", fe.velocity);
    }

    builder.add_header("semi_implicit");
    IntegrationState<1, float> si(VecN<1, float>(0.0f), VecN<1, float>(1.0f), 0.0f);
    for (int i = 0; i < steps; ++i) {
        integrate_semi_implicit_euler(si, constant_accel, dt);
        builder.add_scalar("t", si.time);
        builder.add_vecn("pos", si.position);
        builder.add_vecn("vel", si.velocity);
    }

    builder.add_header("velocity_verlet");
    IntegrationState<1, float> vv(VecN<1, float>(0.0f), VecN<1, float>(1.0f), 0.0f);
    for (int i = 0; i < steps; ++i) {
        integrate_velocity_verlet(vv, constant_accel, dt);
        builder.add_scalar("t", vv.time);
        builder.add_vecn("pos", vv.position);
        builder.add_vecn("vel", vv.velocity);
    }

    builder.add_header("rk4");
    IntegrationState<1, float> rk(VecN<1, float>(0.0f), VecN<1, float>(1.0f), 0.0f);
    for (int i = 0; i < steps; ++i) {
        integrate_rk4(rk, constant_accel, dt);
        builder.add_scalar("t", rk.time);
        builder.add_vecn("pos", rk.position);
        builder.add_vecn("vel", rk.velocity);
    }

    std::string output = builder.str();
    const std::string filepath = golden_dir + "/math/calculus/integrators_constant_accel.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

TEST_CASE("Golden math finite differences") {
    std::string golden_dir = get_golden_dir();
    ensure_golden_dir(golden_dir + "/math/calculus");

    GoldenTextBuilder builder;
    auto f = [](float x) { return std::sin(x); };
    const float x = 0.5f;

    builder.add_header("scalar_derivatives");
    builder.add_scalar("forward_first", forward_difference_first(f, x));
    builder.add_scalar("central_first", central_difference_first(f, x));
    builder.add_scalar("forward_second", forward_difference_second(f, x));
    builder.add_scalar("central_second", central_difference_second(f, x));

    builder.add_header("jacobian");
    auto vector_func = [](const VecN<2, float>& v) {
        return VecN<2, float>(std::array<float, 2>{v[0] * v[0] + v[1], std::sin(v[0]) + v[1] * v[1]});
    };
    VecN<2, float> x0(std::array<float, 2>{0.25f, -0.5f});
    auto J = numerical_jacobian<2, 2>(vector_func, x0);
    builder.add_matn("J", J);

    std::string output = builder.str();
    const std::string filepath = golden_dir + "/math/calculus/finite_differences.golden";

#ifdef GOLDEN_CAPTURE_MODE
    save_text_file(filepath, output);
    SUCCEED("Golden file captured");
#else
    std::string golden = load_text_file(filepath);
    REQUIRE(golden == output);
#endif
}

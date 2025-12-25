#include <core/physics/particle.hpp>
#include <iostream>
#include <cmath>

using phynity::physics::Particle;
using phynity::math::Vec3;

static bool near_equal(float a, float b, float eps = 1e-4f) {
    return std::fabs(a - b) <= eps;
}

int main() {
    int failures = 0;

    Particle p;
    p.position = Vec3(0.0f, 0.0f, 0.0f);
    p.velocity = Vec3(0.0f, 0.0f, 0.0f);
    p.mass = 1.0f;

    // Apply gravity force F = m * g; with m=1, just g
    const float g = -9.81f;
    p.applyForce(Vec3(0.0f, g, 0.0f));

    const float dt = 1.0f / 60.0f; // 60 FPS
    p.integrate(dt);

    // After one step: v = g*dt, y = v*dt
    float expected_vy = g * dt;
    float expected_y = expected_vy * dt;

    if (!near_equal(p.velocity.y, expected_vy)) {
        std::cerr << "Particle velocity update failed: vy=" << p.velocity.y
                  << " expected=" << expected_vy << "\n";
        ++failures;
    }
    if (!near_equal(p.position.y, expected_y)) {
        std::cerr << "Particle position update failed: y=" << p.position.y
                  << " expected=" << expected_y << "\n";
        ++failures;
    }

    if (failures) {
        std::cerr << failures << " particle test(s) failed\n";
        return 1;
    }
    std::cout << "Particle tests passed\n";
    return 0;
}

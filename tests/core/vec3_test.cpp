#include <core/math/vec3.hpp>
#include <iostream>
#include <cmath>

using phynity::math::Vec3;

static bool near_equal(float a, float b, float eps = 1e-4f) {
    return std::fabs(a - b) <= eps;
}

int main() {
    int failures = 0;

    // Test addition
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b;
    if (!near_equal(c.x, 5.0f) || !near_equal(c.y, 7.0f) || !near_equal(c.z, 9.0f)) {
        std::cerr << "Vec3 add failed: got (" << c.x << ", " << c.y << ", " << c.z << ")\n";
        ++failures;
    }

    // Test dot & length
    float d = a.dot(a);
    float len = a.length();
    if (!near_equal(d, 14.0f) || !near_equal(len, std::sqrt(14.0f))) {
        std::cerr << "Vec3 dot/length failed: dot=" << d << ", len=" << len << "\n";
        ++failures;
    }

    // Test normalize
    Vec3 n = a.normalized();
    float nlen = n.length();
    if (!near_equal(nlen, 1.0f)) {
        std::cerr << "Vec3 normalize failed: len=" << nlen << "\n";
        ++failures;
    }

    if (failures) {
        std::cerr << failures << " vec3 test(s) failed\n";
        return 1;
    }
    std::cout << "Vec3 tests passed\n";
    return 0;
}

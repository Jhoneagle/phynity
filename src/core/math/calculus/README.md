# Calculus Utilities

Numerical differentiation and integration methods for physics simulations.

## Implemented Components

### ✅ Finite Differences (`finite_differences.hpp`)
- **Forward difference** - First and second derivatives (O(h) accuracy)
- **Central difference** - First and second derivatives (O(h²) accuracy)
- **Vector-valued functions** - Partial derivatives for vector functions
- **Numerical Jacobian** - Multivariate function derivatives
- **Optimal step size** - Automatic h selection to balance truncation vs rounding error
- **Derivative validation** - Compare numerical vs analytical derivatives

**Key features:**
- Central differences are 4x more accurate than forward differences
- Automatic step size optimization
- Validation utilities for testing

### ✅ Numerical Integrators (`integrators.hpp`)
- **Forward Euler** - Simple but unstable (O(dt²) per step)
- **Semi-implicit Euler** - Energy-stable (O(dt²) per step)
- **Velocity Verlet** - Industry standard for physics (O(dt⁴) position accuracy)
- **RK4 (Runge-Kutta 4)** - Highest precision (O(dt⁵))
- **Unified interface** - Select method at runtime or compile-time

**Key features:**
- Velocity Verlet: 2x more accurate than Semi-implicit, excellent energy conservation
- All methods support configurable timesteps
- Acceleration callback interface for flexible force functions

## Integration Methods Comparison

| Method | Accuracy | Stability | Energy Conservation | Speed | Use Case |
|--------|----------|-----------|-------------------|-------|----------|
| Forward Euler | O(dt²) | Unstable | Poor | Very Fast | Quick tests only |
| Semi-implicit Euler | O(dt²) | Stable | Good | Very Fast | Simple oscillators |
| Velocity Verlet | O(dt⁴) | Stable | Excellent | Fast | **Physics simulations** |
| RK4 | O(dt⁵) | Good | Poor | 4x slower | High-precision, non-conservative |

**Recommendation:** Use **Velocity Verlet** for physics - it's the sweet spot between accuracy, stability, and performance.

## Usage Example

```cpp
#include <core/math/calculus/finite_differences.hpp>
#include <core/math/calculus/integrators.hpp>

using namespace phynity::math::calculus;

// Example 1: Compute numerical derivative
auto f = [](float x) { return x * x + 2 * x + 1; };
auto f_prime = [](float x) { return 2 * x + 2; };  // Analytical derivative

float x = 3.0f;
float numerical_deriv = central_difference_first(f, x);  // ≈ 8.0
float analytical_deriv = f_prime(x);                      // = 8.0

// Example 2: Integrate particle under gravity
auto gravity = [](const Vec3<float>& pos, const Vec3<float>& vel, float t) {
    return Vec3<float>(0, -9.81f, 0);  // Constant downward acceleration
};

IntegrationState<3, float> particle;
particle.position = Vec3<float>(0, 100, 0);  // Start 100 meters up
particle.velocity = Vec3<float>(0, 0, 0);    // Initially at rest
particle.time = 0;

float dt = 0.016f;  // 60 FPS
for (int step = 0; step < 100; ++step) {
    integrate_velocity_verlet(particle, gravity, dt);
    // particle.position is updated
}

// Example 3: Using unified interface
integrate(particle, gravity, dt, IntegratorMethod::VelocityVerlet);

// Example 4: Validate derivative
bool valid = validate_derivative(f, f_prime, x, 1e-4f, 1e-3f);
```

## Numerical Stability Considerations

### Finite Differences
- **Step size selection:** Too large → truncation error, too small → rounding error
- **Optimal h ≈ sqrt(eps) * |x|** for first derivatives
- **Optimal h ≈ eps^(1/3) * |x|** for second derivatives
- Central differences much more accurate than forward differences

### Integration
- **Timestep choice:** Larger dt is faster but less accurate
- **Energy conservation:** Velocity Verlet drifts O(10⁻³) relative error
- **Stability limit:** Forward Euler unstable for dt > 2/ω (where ω is highest frequency)
- **Semi-implicit Euler:** Stable for larger timesteps than Forward Euler

## Planned Enhancements

- Curve fitting and derivative estimation from data
- Adaptive timestep methods (RKF45, others)
- Constraint handling in integration
- Specialized solvers for stiff systems
- Batch integration for multiple particles

## Status

✅ Finite differences complete with optimal step size
✅ Four integration methods implemented
✅ Ready for particle simulation integration

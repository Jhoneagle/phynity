# Phynity User Guide

## Philosophy: Dual-Scale Physics

Phynity is designed as a **professional-grade physics engine** capable of simulating phenomena across vastly different scales with equal fidelity. The engine recognizes that micro-scale and macro-scale physics require different representations, optimizations, and numerical approaches.

**Status**: Active development, production-leaning. Core modules are stable; advanced features are still evolving.

## Quick Commands

Prefer `tools/` scripts for day-to-day workflow.

### Windows

```bat
tools\build.bat debug
tools\test.bat debug
tools\run.bat debug
```

### Linux/macOS/MSYS

```bash
./tools/build.sh debug
./tools/test.sh debug
./tools/run.sh debug
```

### The Two Scales

```
MICRO-SCALE                      MACRO-SCALE
(Particles)                      (Rigid Bodies)
─────────────────────────────────────────────────────────
Point masses                     Extended bodies
No rotation                      6-DOF (position + rotation)
Sphere collisions (broadphase)   Complex shapes (convex hulls)
O(n^2) fallback                  O(n log n) with broadphase
Mass-spring systems              Inertia tensors
Molecular dynamics               Mechanical systems
Fluids (SPH)                     Vehicles, buildings
Thousands of entities            Hundreds of entities
```

## When to Use Each Scale

### Use **Micro-Scale (Particle Physics)** for:

✅ **Granular materials**: Sand, gravel, powders  
✅ **Fluids**: Water, gas simulations (SPH, PBF)  
✅ **Molecular dynamics**: Chemical simulations, atom interactions  
✅ **Particle effects**: Sparks, smoke, debris clouds  
✅ **Cloth/fabrics**: Mass-spring networks  
✅ **Soft bodies**: Deformable objects with many DOFs  
✅ **Abstract systems**: Swarms, crowd behavior  

**Characteristics**:
- Entities represented as point masses
- Position and velocity only (no rotation)
- Sphere-sphere collisions with spatial broadphase
- Force-based interactions (springs, fields)
- Scales to thousands of entities
- Lower per-entity computational cost

### Use **Macro-Scale (Rigid Body Physics)** for:

✅ **Mechanical objects**: Boxes, spheres, complex convex shapes  
✅ **Vehicles**: Cars, planes with proper rotation  
✅ **Buildings/structures**: With joints and constraints  
✅ **Machinery**: Articulated systems, doors, elevators  
✅ **Projectiles**: Rotating objects with angular momentum  
✅ **Characters**: Ragdolls, animated physics  
✅ **Destruction**: Breaking objects with realistic rotation  

**Characteristics**:
- Full 6-DOF state (position + orientation)
- Inertia tensors for realistic rotation
- Complex collision shapes (boxes, capsules, convex hulls)
- Constraint-based joints (fixed; hinge/ball/prismatic planned)
- Typically 10s to 100s of entities
- Higher per-entity computational cost

## Cross-Scale Interactions

The physics engine supports **hybrid simulations** where both scales coexist:

```cpp
// Example: Debris explosion
// - Rigid bodies for large chunks (with rotation)
// - Particles for dust and small fragments

RigidBodySystem rigid_bodies;
ParticleSystem particles;

// Large debris: full rotation and complex collision
rigid_bodies.spawn(position, orientation, box_shape, 10.0f);

// Small debris: simple point masses
for (int i = 0; i < 1000; ++i) {
    particles.spawn(position + random_offset(), velocity, 0.01f);
}
```

### Collision Between Scales

```cpp
// Particles can interact with rigid body surfaces
// - Rigid body acts as static/dynamic obstacle
// - Particles bounce off with correct restitution
// - Two-way coupling is planned (optional)

// Example: Water (particles) splashing on rocks (rigid bodies)
```

## Architecture Overview

### Common Infrastructure (Shared)

Both scales share fundamental components:

```cpp
#include <core/physics/dynamics/material.hpp>
#include <core/physics/dynamics/force_field.hpp>
#include <core/physics/config/timestep_controller.hpp>
```

**Material**: Physical properties (mass, restitution, friction)  
**ForceField**: Pluggable forces (gravity, drag, custom)  
**TimestepController**: Fixed timestep with determinism  
**Diagnostics**: Energy and momentum tracking  

### Micro-Scale Components

```cpp
#include <core/physics/particles/particle.hpp>
#include <core/physics/particles/particle_system.hpp>
```

**Particle**: Point mass with position, velocity, forces  
**ParticleSystem**: Manages lifecycle, forces, collisions  
**Future**: SPH fluids, mass-spring networks  

### Macro-Scale Components

```cpp
#include <core/physics/rigid_bodies/rigid_body.hpp>
#include <core/physics/rigid_bodies/rigid_body_system.hpp>
#include <core/physics/shapes/shape.hpp>
#include <core/physics/constraints/weld_joint.hpp>
```

**RigidBody**: 6-DOF with inertia tensor  
**RigidBodySystem**: Manages lifecycle, forces, constraints  
**Shape**: Collision geometry (sphere, box, capsule, convex)  
**Constraints**: Joints for articulated systems (fixed today; more planned)  

### Shared Collision and Constraints

```cpp
#include <core/physics/collision/broadphase/spatial_grid.hpp>
#include <core/physics/collision/narrowphase/gjk_solver.hpp>
#include <core/physics/constraints/constraint_solver.hpp>
```

Both scales use the same collision and constraint infrastructure:
- **Broadphase**: Spatial acceleration (grid, BVH)
- **Narrowphase**: Collision detection (GJK, EPA, SAT)
- **Contact**: Manifold generation and caching
- **Constraints**: Solver + contact constraints, with joint constraints evolving over time

## Usage Patterns

### Pattern 1: Pure Micro-Scale Simulation

```cpp
#include <core/physics/particles/particle_system.hpp>
#include <core/physics/dynamics/force_field.hpp>

// Water simulation with 10,000 particles
ParticleSystem water;

// Add gravity
auto gravity = std::make_shared<GravityField>(Vec3f(0, -9.81f, 0));
water.add_force_field(gravity);

// Spawn particles in volume
for (int i = 0; i < 10000; ++i) {
    Vec3f pos = random_in_sphere(Vec3f(0, 10, 0), 2.0f);
    water.spawn(pos, Vec3f::Zero(), 0.001f);  // 1 gram each
}

// Simulate
float dt = 1.0f / 60.0f;
for (int frame = 0; frame < 600; ++frame) {
    water.update(dt);
}
```

### Pattern 2: Pure Macro-Scale Simulation

```cpp
#include <core/physics/rigid_bodies/rigid_body_system.hpp>
#include <core/physics/shapes/shape.hpp>
#include <core/physics/constraints/weld_joint.hpp>

// Vehicle with wheels welded to chassis (hinge joints planned)
RigidBodySystem vehicle_sim;

// Chassis (100 kg box)
auto chassis_shape = std::make_shared<BoxShape>(Vec3f(2, 0.5f, 1));
auto chassis_id = vehicle_sim.spawn_body(
    Vec3f(0, 1, 0), Quatf(), chassis_shape, 100.0f
);

// Wheels (10 kg cylinders)
auto wheel_shape = std::make_shared<CylinderShape>(0.3f, 0.2f);
for (int i = 0; i < 4; ++i) {
    Vec3f wheel_pos = get_wheel_position(i);
    auto wheel_id = vehicle_sim.spawn_body(
        wheel_pos, Quatf(), wheel_shape, 10.0f
    );
    
    // Weld wheel to chassis (placeholder until hinge joints are implemented)
    auto weld = std::make_shared<WeldJoint>(
        vehicle_sim.get_body(chassis_id),
        vehicle_sim.get_body(wheel_id),
        wheel_pos,
        wheel_pos
    );
    vehicle_sim.add_constraint(weld);
}
```

### Pattern 3: Hybrid Simulation

```cpp
// Demolition scene: building (rigid bodies) + dust (particles)
RigidBodySystem building;
ParticleSystem dust;

// Building as stacked boxes
for (int floor = 0; floor < 10; ++floor) {
    auto box = std::make_shared<BoxShape>(Vec3f(5, 0.5f, 5));
    building.spawn_body(
        Vec3f(0, floor * 1.0f, 0),
        Quatf(),
        box,
        100.0f
    );
}

// Apply explosive force to building
// Rigid bodies break apart with realistic rotation

// Generate dust cloud as particles
for (int i = 0; i < 5000; ++i) {
    Vec3f pos = explosion_center + random_offset();
    Vec3f vel = (pos - explosion_center).normalized() * speed;
    dust.spawn(pos, vel, 0.0001f);
}

// Update both systems
void simulate(float dt) {
    building.update(dt);
    dust.update(dt);
    
    // Optional: Check particle-rigid body collisions
    check_cross_scale_collisions(dust, building);
}
```

## Force Fields

Force fields are pluggable, polymorphic sources of force. Each derives from
`ForceField` and implements a single method that receives a `ForceContext`
(the body's position, velocity, and mass) and returns a force vector:

```cpp
struct ForceContext {
    Vec3f position{0.0f};
    Vec3f velocity{0.0f};
    float mass{0.0f};
    // extended additively over time (charge, volume, temperature, ...)
};

Vec3f ForceField::apply(const ForceContext &ctx) const;
```

Add any number of fields to a `ParticleSystem` or `RigidBodySystem` via
`add_force_field(...)`; every field is applied to every body each step. Fields
are not part of the serialization snapshot — reconstruct them on setup.

### Built-in fields

| Field | Formula | Use for |
|---|---|---|
| `GravityField` | `F = m·g` | uniform gravity |
| `DragField` | `F = -c·v` | linear (low-speed) drag |
| `QuadraticDragField` | `F = -c·\|v\|·v` | high-speed drag |
| `SpringField` | `F = -k·(x - center)` | restoring spring |
| `PointGravityField` | `F = m·G·M/r² toward center` | radial "gravity well" (softened) |
| `WindField` | `F = c·(wind - v)`, optional AABB region | wind / bounded drag volumes |
| `SpringDamperField` | `F = -k·(x - center) - c·v` | damped oscillator |
| `BuoyancyField` | `F = -g·ρ_fluid·(m/ρ_object)` below surface | buoyancy for simple fluids |

### Examples

```cpp
#include <core/physics/dynamics/force_field.hpp>
#include <core/physics/config/physics_constants.hpp>

using namespace phynity::physics;
using namespace phynity::physics::constants;

// A radial gravity well at the origin (G·M = 20), softened near the center.
system.add_force_field(std::make_unique<PointGravityField>(Vec3f(0.0f), 20.0f, 1e-2f));

// Wind blowing +x, but only inside a bounded corridor.
shapes::AABB corridor(Vec3f(-6.0f, -2.0f, -2.0f), Vec3f(6.0f, 2.0f, 2.0f));
system.add_force_field(std::make_unique<WindField>(Vec3f(8.0f, 0.0f, 0.0f), 0.5f, corridor));

// A damped spring that settles a body back to the center.
system.add_force_field(std::make_unique<SpringDamperField>(Vec3f(0.0f), 10.0f, 2.0f));

// Buoyancy in water: light objects (ρ_object < ρ_fluid) float to the surface.
// BuoyancyField reads "down" from the system's ambient gravity (shared via the
// ForceContext), so publish it once with set_ambient_gravity — no per-field copy.
Vec3f gravity(0.0f, -EARTH_GRAVITY, 0.0f);
system.set_ambient_gravity(gravity);
system.add_force_field(std::make_unique<GravityField>(gravity));
system.add_force_field(std::make_unique<BuoyancyField>(WATER_DENSITY, 500.0f, /*surface=*/0.0f));
system.add_force_field(std::make_unique<DragField>(1.5f)); // dissipates bobbing
```

See the `Wind Tunnel` and `Floating Objects` sandbox scenarios for runnable demos.

## Fluids (SPH prototype)

Beyond the per-particle force fields above, Phynity includes a **particle-based
fluid solver** as a third peer subsystem alongside `ParticleSystem` and
`RigidBodySystem`. Fluids cannot be modeled as a `ForceField`: SPH's
density → pressure → force dependency couples each particle to its neighbors
across multiple passes, which the pure per-particle `apply(ctx)` contract cannot
express. `SphFluidSystem` therefore owns its own particles, neighbor search,
parameters, and step pipeline.

The prototype implements **weakly-compressible SPH (WCSPH)** with the classic
Müller (2003) kernels — poly6 (density), spiky gradient (pressure), and the
viscosity laplacian — a linear equation of state, Monaghan's momentum-conserving
symmetric pressure force, and an optional color-field surface-tension term. It
reuses the engine's `SpatialGrid` (cell size = smoothing radius) for a
deterministic fixed-radius neighbor search.

```cpp
#include <core/physics/fluids/sph_fluid_system.hpp>
using namespace phynity::physics::fluids;

SphParameters params;
params.smoothing_radius = 0.1f;          // kernel support h
params.rest_density = WATER_DENSITY;     // ρ₀
params.stiffness = 100.0f;               // EOS: p = k·(ρ − ρ₀)
params.viscosity = 0.05f;                // Müller viscosity μ
params.clamp_negative_pressure = true;   // suppress free-surface tensile instability
params.bounds = AABB(Vec3f(-0.5f), Vec3f(0.5f));

// particle_mass and rest_density are NOT independent: for a lattice of spacing s
// to start at rest, mass ≈ ρ₀·s³. Always size mass through mass_for_spacing().
const float spacing = 0.05f;             // h = 2·spacing is a good ratio
params.particle_mass = mass_for_spacing(params.rest_density, spacing);

SphFluidSystem fluid(params);
fluid.set_ambient_gravity(Vec3f(0.0f, -EARTH_GRAVITY, 0.0f));

// Seed a block on the lattice, then step:
for (/* ix, iy, iz over the block */)
    fluid.spawn(origin + Vec3f(ix, iy, iz) * spacing);

fluid.update(dt); // rebuild neighbors → density → pressure → forces → integrate → boundaries
```

**Stability caveat.** WCSPH is explicit and subject to a CFL-like timestep limit:
higher `stiffness` (a stiffer fluid) needs a smaller `dt`, and because the
viscosity term is integrated explicitly, a large `viscosity` is unconditionally
unstable at a given `dt`/`h`. The defaults above are tuned for a stable prototype
pool; if a fluid "explodes," lower `dt`, lower `stiffness`, or lower `viscosity`.
The box boundary is a simple clamp-and-reflect container with no boundary
pressure, so near-wall particles are neighbor-deficient (a standard SPH
limitation) — treat wall-adjacent behavior as approximate.

See the `Dam Break (SPH)` sandbox scenario for a runnable demo. The sandbox
sub-steps the fluid internally so it stays stable at the render frame rate.

Fluid state is intentionally **excluded from the snapshot/replay timeline** for
the prototype (the solver itself is deterministic; persistence is deferred).

## Determinism and Reproducibility

Both scales support **deterministic simulation**:

```cpp
// Enable deterministic mode
TimestepController::Config config{
    .target_fps = 60.0f,
    .use_determinism = true
};

// Guarantees:
// - Bit-identical results on same platform
// - Fixed timestep (no variable dt)
// - Deterministic collision order
// - No floating-point non-determinism

// Use for:
// - Regression testing
// - Replay systems
// - Networked physics
// - Scientific validation
```

## Performance Characteristics

These numbers are illustrative only. Use profiling in your target environment for accurate measurements.

### Micro-Scale Performance

| Entity Count | Update Time | Collision Time | Total    |
|--------------|-------------|----------------|----------|
| 100          | ~0.1 ms     | ~0.5 ms        | ~0.6 ms  |
| 1,000        | ~1 ms       | ~5-20 ms       | ~6-21 ms |
| 10,000       | ~10 ms      | ~100-500 ms    | ~110-510 ms |

*Note: O(n^2) collision without broadphase. Use spatial grid for 1,000+ particles.*

### Macro-Scale Performance

| Body Count | Update Time | Collision Time | Constraint Solving | Total  |
|------------|-------------|----------------|--------------------|--------|
| 10         | ~0.2 ms     | ~0.1 ms        | ~0.1 ms            | ~0.4 ms |
| 100        | ~2 ms       | ~1-3 ms        | ~2-5 ms            | ~5-10 ms |
| 1,000      | ~20 ms      | ~10-30 ms      | ~30-80 ms          | ~60-130 ms |

*Note: With spatial broadphase. Constraint solving scales with contact count.*

## Best Practices

### 1. Choose the Right Scale

```cpp
// Micro-scale: particles for large counts and simple interactions
// ✅ Use particles for fluids and granular materials
ParticleSystem water;
for (int i = 0; i < 10000; ++i) {
    water.spawn(...);  // Fast, appropriate
}

// ❌ Avoid rigid bodies for large particle-like effects
RigidBodySystem debris;
for (int i = 0; i < 10000; ++i) {
    debris.spawn_body(...);  // Too slow, overkill
}

// Macro-scale: rigid bodies for rotation and complex shapes
// ✅ Use rigid bodies for rotating objects and joints
RigidBodySystem box;
box.spawn_body(..., orientation, ...);  // Proper rotation

// ❌ Avoid particles when rotation and shape matter
ParticleSystem rotating_box;
rotating_box.spawn(...);  // Can't represent rotation
```

### 2. Use Appropriate Timesteps

```cpp
// Micro-scale: smaller timestep for stability
ParticleSystem particles;
float dt_micro = 1.0f / 120.0f;  // 120 Hz

// Macro-scale: larger timestep acceptable
RigidBodySystem bodies;
float dt_macro = 1.0f / 60.0f;  // 60 Hz

// If running both, keep a fixed step and decouple render rate
```

### 3. Profile and Optimize

```cpp
// Identify bottlenecks
PROFILE_SCOPE("Physics Update");

{
    PROFILE_SCOPE("Particles");
    particles.update(dt);  // Is this too slow?
}

{
    PROFILE_SCOPE("Rigid Bodies");
    bodies.update(dt);  // Or this?
}

// Micro-scale: add spatial broadphase if collision is slow
particles.set_broadphase(std::make_shared<SpatialGrid>(cell_size));

// Macro-scale: profile constraint solve and contact density
// before increasing solver iterations globally
```

### 4. Test Determinism

```cpp
// Validate deterministic stepping for both scales
TEST_CASE("Deterministic simulation replay") {
    ParticleSystem particles_a, particles_b;
    RigidBodySystem rigid_a, rigid_b;

    setup_particle_scenario(particles_a);
    setup_particle_scenario(particles_b);
    setup_rigid_scenario(rigid_a);
    setup_rigid_scenario(rigid_b);

    for (int i = 0; i < 1000; ++i) {
        particles_a.update(dt);
        particles_b.update(dt);
        rigid_a.update(dt);
        rigid_b.update(dt);
    }

    REQUIRE(hash_state(particles_a) == hash_state(particles_b));
    REQUIRE(hash_state(rigid_a) == hash_state(rigid_b));
}
```

### 5. Monitor Conservation Laws

```cpp
// Physics should conserve energy and momentum (without external forces)
Diagnostics initial = system.diagnostics();

// Run simulation
for (int i = 0; i < 1000; ++i) {
    system.update(dt);
}

Diagnostics final = system.diagnostics();

// Energy should be conserved (within numerical error)
REQUIRE(abs(final.total_energy - initial.total_energy) < tolerance);

// Momentum should be conserved
REQUIRE((final.total_momentum - initial.total_momentum).length() < tolerance);
```

## Troubleshooting

### Problem: Particles Exploding (Unstable)

**Cause**: Timestep too large, forces too strong  
**Solution**:
```cpp
// Reduce timestep
float dt = 1.0f / 120.0f;  // Instead of 1.0f / 60.0f

// Or add damping
Material mat;
mat.linear_damping = 0.1f;  // 10% damping per second
```

### Problem: Rigid Bodies Jittering or Exploding

**Cause**: Large timestep, high contact density, or solver under-iteration  
**Solution**:
```cpp
// Reduce timestep
float dt = 1.0f / 120.0f;

// Increase solver iterations for contact-heavy scenes
ConstraintSolverConfig config;
config.iterations = 20;  // Default: 4
```

### Problem: Rigid Bodies Penetrating

**Cause**: Constraint solver not converging  
**Solution**:
```cpp
// Increase solver iterations
ConstraintSolverConfig config;
config.iterations = 20;  // Default: 4

// Or reduce timestep
float dt = 1.0f / 120.0f;
```

### Problem: Slow Performance with Many Entities

**Cause**: O(n²) collision detection  
**Solution**:
```cpp
// Enable spatial broadphase
auto grid = std::make_shared<SpatialGrid>(2.0f);  // 2m cells
system.set_broadphase(grid);

// If rigid-body scenes are slow:
// - Profile narrowphase vs. constraint solver time
// - Reduce contact density where possible
// - Tune solver iteration count per scenario
```

### Problem: Non-Deterministic Behavior

**Cause**: Floating-point non-determinism, variable timestep  
**Solution**:
```cpp
// Enable deterministic mode
TimestepController::Config config{
    .use_determinism = true  // Fixed timestep, deterministic order
};

// Avoid multi-threading in deterministic mode
config.enable_parallel = false;
```

## Future Directions

### Planned Extensions

**Micro-Scale**:
- SPH (Smoothed Particle Hydrodynamics) for fluids
- Mass-spring cloth simulation
- Molecular dynamics templates

**Macro-Scale**:
- Soft bodies with deformation
- Destructible objects with fracture
- Articulated characters (ragdolls)

**Cross-Scale**:
- Two-way coupling (particles affect rigid bodies)
- Unified broadphase for both scales
- Adaptive LOD (switch representations)

**Advanced Features**:
- Continuous collision detection (CCD)
- Friction models (Coulomb, viscous)
- Restitution curves for materials
- Temperature and thermodynamics

## Further Reading

- **architecture.md**: Overall engine design
- **roadmap.md**: Planned features timeline
- **testing_best_practices.md**: How to test physics code

---

**Version**: 1.0  
**Last Updated**: 2026-02-28  
**Maintained By**: Phynity Team

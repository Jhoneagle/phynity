# Physics Engine User Guide

## Philosophy: Dual-Scale Physics

Phynity is designed as a **professional-grade physics engine** capable of simulating phenomena across vastly different scales with equal fidelity. The engine recognizes that micro-scale and macro-scale physics require different representations, optimizations, and numerical approaches.

### The Two Scales

```
MICRO-SCALE                      MACRO-SCALE
(Particles)                      (Rigid Bodies)
─────────────────────────────────────────────────────────
Point masses                     Extended bodies
No rotation                      6-DOF (position + rotation)
Simple collisions (spheres)      Complex shapes (convex hulls)
O(n²) interactions               O(n log n) with broadphase
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
- Simple sphere-sphere collisions
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
- Constraint-based joints (hinge, ball, prismatic)
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
// - No torque applied to rigid body (optional)

// Example: Water (particles) splashing on rocks (rigid bodies)
```

## Architecture Overview

### Common Infrastructure (Shared)

Both scales share fundamental components:

```cpp
#include <core/physics/common/material.hpp>
#include <core/physics/common/force_field.hpp>
#include <core/physics/common/timestep_controller.hpp>
```

**Material**: Physical properties (mass, restitution, friction)  
**ForceField**: Pluggable forces (gravity, drag, custom)  
**TimestepController**: Fixed timestep with determinism  
**Diagnostics**: Energy and momentum tracking  

### Micro-Scale Components

```cpp
#include <core/physics/micro/particle.hpp>
#include <core/physics/micro/particle_system.hpp>
```

**Particle**: Point mass with position, velocity, forces  
**ParticleSystem**: Manages lifecycle, forces, collisions  
**Future**: SPH fluids, mass-spring networks  

### Macro-Scale Components

```cpp
#include <core/physics/macro/rigid_body.hpp>
#include <core/physics/macro/rigid_body_system.hpp>
#include <core/physics/macro/shape.hpp>
#include <core/physics/constraints/joints/hinge_joint.hpp>
```

**RigidBody**: 6-DOF with inertia tensor  
**RigidBodySystem**: Manages lifecycle, forces, constraints  
**Shape**: Collision geometry (sphere, box, capsule, convex)  
**Constraints**: Joints for articulated systems  

### Universal Collision

```cpp
#include <core/physics/collision/broadphase/spatial_grid.hpp>
#include <core/physics/collision/narrowphase/gjk_solver.hpp>
```

Both scales use the same collision infrastructure:
- **Broadphase**: Spatial acceleration (grid, BVH)
- **Narrowphase**: Collision detection (GJK, EPA, SAT)
- **Contact**: Manifold generation and caching

## Usage Patterns

### Pattern 1: Pure Micro-Scale Simulation

```cpp
#include <core/physics/micro/particle_system.hpp>
#include <core/physics/common/force_field.hpp>

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
#include <core/physics/macro/rigid_body_system.hpp>
#include <core/physics/macro/shape.hpp>
#include <core/physics/constraints/joints/hinge_joint.hpp>

// Vehicle with wheels connected by hinge joints
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
    
    // Connect wheel to chassis with hinge
    auto hinge = std::make_shared<HingeJoint>(
        chassis_id, wheel_id,
        wheel_pos, wheel_pos,
        Vec3f(1, 0, 0)  // Rotation axis
    );
    vehicle_sim.add_constraint(hinge);
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

### Micro-Scale Performance

| Entity Count | Update Time | Collision Time | Total    |
|--------------|-------------|----------------|----------|
| 100          | 0.1 ms      | 0.5 ms         | 0.6 ms   |
| 1,000        | 1.0 ms      | 50 ms          | 51 ms    |
| 10,000       | 10 ms       | 5000 ms        | 5010 ms  |

*Note: O(n²) collision without broadphase. Use spatial grid for 1000+ particles.*

### Macro-Scale Performance

| Body Count | Update Time | Collision Time | Constraint Solving | Total  |
|------------|-------------|----------------|--------------------|--------|
| 10         | 0.2 ms      | 0.1 ms         | 0.1 ms             | 0.4 ms |
| 100        | 2.0 ms      | 1.0 ms         | 2.0 ms             | 5.0 ms |
| 1,000      | 20 ms       | 10 ms          | 30 ms              | 60 ms  |

*Note: With spatial broadphase. Constraint solving scales with contact count.*

## Best Practices

### 1. Choose the Right Scale

```cpp
// ❌ Don't use rigid bodies for fluids
RigidBodySystem water;
for (int i = 0; i < 10000; ++i) {
    water.spawn_body(...);  // Too slow, overkill
}

// ✅ Use particles for fluids
ParticleSystem water;
for (int i = 0; i < 10000; ++i) {
    water.spawn(...);  // Fast, appropriate
}

// ❌ Don't use particles for rotating objects
ParticleSystem box;
box.spawn(...);  // Can't represent rotation

// ✅ Use rigid bodies
RigidBodySystem box;
box.spawn_body(..., orientation, ...);  // Proper rotation
```

### 2. Use Appropriate Timesteps

```cpp
// Micro-scale: smaller timestep for stability
ParticleSystem particles;
float dt_micro = 1.0f / 120.0f;  // 120 Hz

// Macro-scale: larger timestep acceptable
RigidBodySystem bodies;
float dt_macro = 1.0f / 60.0f;  // 60 Hz

// Can run at different rates if needed
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

// Add spatial broadphase if collision is slow
particles.set_broadphase(std::make_shared<SpatialGrid>(cell_size));
```

### 4. Test Determinism

```cpp
// Validate your simulation is deterministic
TEST_CASE("Deterministic particle physics") {
    ParticleSystem sys1, sys2;
    
    // Identical setup
    setup_scenario(sys1);
    setup_scenario(sys2);
    
    // Run in parallel
    for (int i = 0; i < 1000; ++i) {
        sys1.update(dt);
        sys2.update(dt);
    }
    
    // Should be bit-identical
    REQUIRE(sys1.diagnostics() == sys2.diagnostics());
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

### Problem: Rigid Bodies Penetrating

**Cause**: Constraint solver not converging  
**Solution**:
```cpp
// Increase solver iterations
RigidBodySystem::Config config;
config.solver_iterations = 20;  // Default: 10

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

// Or reduce entity count
// Or switch to coarser representation
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
- **physics_restructuring_plan.md**: Module organization
- **testing_best_practices.md**: How to test physics code

---

**Version**: 1.0  
**Last Updated**: 2026-02-28  
**Maintained By**: Phynity Team

# Observability System

## Overview

The Phynity observability system provides comprehensive profiling and diagnostics for performance analysis and correctness validation. It consists of three main subsystems:

1. **Performance Profiling** - Measure execution time of code sections
2. **Physics Monitoring** - Validate energy and momentum conservation
3. **Collision Analysis** - Track collision detection efficiency

All systems are designed with **zero-cost abstractions** - when disabled, they add no runtime overhead.

---

## Performance Profiling

### Quick Start

```cpp
#include <core/diagnostics/profiling_macros.hpp>

void update_simulation(float dt) {
    PROFILE_FUNCTION();  // Automatically uses function name
    
    {
        PROFILE_SCOPE("physics_step");
        // ... physics update code
    }
    
    {
        PROFILE_SCOPE("collision_detection");
        // ... collision detection code
    }
}
```

### Profiling Macros

| Macro | Description | Example |
|-------|-------------|---------|
| `PROFILE_FUNCTION()` | Profile entire function (uses `__FUNCTION__`) | Place at function start |
| `PROFILE_SCOPE(name)` | Profile a specific code block | Use for hot loops, subsections |
| `PROFILE_ZONE(name)` | Alias for `PROFILE_SCOPE` | Same as above |

### Enabling/Disabling Profiling

```cpp
#include <core/diagnostics/profiler.hpp>

// Enable profiling at runtime
Profiler::enable(true);

// Disable profiling (zero overhead)
Profiler::enable(false);

// Check current state
if (Profiler::is_enabled()) {
    // ...
}
```

### Retrieving Profiling Data

```cpp
// Get all zones collected in current frame
const auto& zones = Profiler::get_zones();

for (const auto& zone : zones) {
    // zone.name - Zone name (string_view)
    // zone.duration_us - Duration in microseconds
    // zone.depth - Nesting level (0 = root)
    // zone.parent_index - Index of parent zone
}

// Clear frame data (typically at frame start)
Profiler::clear_frame();
```

### Frame-Level Profiling

```cpp
#include <core/diagnostics/frame_profiler.hpp>

FrameProfiler profiler;
profiler.set_history_size(60);  // Keep last 60 frames

// At frame boundaries
profiler.begin_frame();
// ... simulation code with PROFILE_SCOPE macros
profiler.end_frame();

// Get statistics
auto stats = profiler.get_zone_stats("physics_step");
std::cout << "Physics avg: " << stats.avg_duration_us / 1000.0 << " ms\n";
std::cout << "Physics max: " << stats.max_duration_us / 1000.0 << " ms\n";

// Get last frame data
auto last_frame = profiler.get_last_frame();
std::cout << "Frame time: " << last_frame.total_frame_time_us / 1000.0 << " ms\n";
```

### Frame Budget Monitoring

```cpp
#include <core/diagnostics/frame_budget.hpp>

FrameBudget budget;
budget.set_target_fps(60.0);  // 16.67ms frame budget

// Set per-zone budgets (optional)
budget.set_zone_budget("physics_step", 5000);     // 5ms
budget.set_zone_budget("collision", 3000);         // 3ms
budget.set_zone_budget("rendering", 8000);         // 8ms

// Get violation warnings
budget.set_violation_callback([](const FrameBudgetViolation& v) {
    std::cerr << "Frame " << v.frame_number 
              << " exceeded budget: " << v.frame_time_us / 1000.0 
              << " ms (budget: " << v.frame_budget_us / 1000.0 << " ms)\n";
});

// Per zone violation callback
budget.set_zone_violation_callback([](const ZoneBudgetViolation& v) {
    std::cerr << "Zone '" << v.zone_name 
              << "' exceeded budget by " 
              << (v.actual_duration_us - v.budget_us) / 1000.0 << " ms\n";
});

// At frame boundaries
budget.begin_frame();
// ... profiled code
budget.end_frame();
```

---

## Physics Monitoring

### Energy Conservation

```cpp
#include <core/diagnostics/energy_monitor.hpp>

auto energy_monitor = std::make_shared<EnergyMonitor>();

// Configure tolerances
energy_monitor->set_max_loss_percent(2.0);   // Allow 2% energy loss per frame
energy_monitor->set_max_gain_percent(0.5);   // Allow 0.5% energy gain per frame
energy_monitor->set_max_spike_percent(10.0); // Allow 10% energy spikes (collisions)

// Set violation callback
energy_monitor->set_violation_callback([](const EnergyViolation& v) {
    std::cerr << "Frame " << v.frame_number 
              << ": Energy loss " << v.loss_percentage << "% (limit: " 
              << v.max_loss_percent << "%)\n";
    std::cerr << "  Previous: " << v.previous_energy 
              << ", Current: " << v.current_energy << "\n";
});

// Update each frame
float total_energy = kinetic_energy + potential_energy;
energy_monitor->update(static_cast<double>(total_energy));

// Enable/disable
energy_monitor->set_enabled(true);
energy_monitor->reset();  // Reset baseline
```

### Momentum Conservation

```cpp
#include <core/diagnostics/momentum_monitor.hpp>

auto momentum_monitor = std::make_shared<MomentumMonitor>();

// Configure tolerance (magnitude of acceptable change)
momentum_monitor->set_max_change_magnitude(100.0);

// Set violation callback
momentum_monitor->set_violation_callback([](const MomentumViolation& v) {
    std::cerr << "Frame " << v.frame_number 
              << ": Momentum changed by " << v.change_magnitude 
              << " (limit: " << v.max_change_magnitude << ")\n";
    std::cerr << "  Cause: " << v.cause << "\n";
});

// Update each frame with total system momentum
Vec3 total_momentum(px, py, pz);
momentum_monitor->update(total_momentum);
```

### Collision Detection Efficiency

```cpp
#include <core/diagnostics/collision_monitor.hpp>

auto collision_monitor = std::make_shared<CollisionMonitor>();

// Configure efficiency threshold
collision_monitor->set_min_efficiency(0.1);  // Expect at least 10% hit rate

// Set violation callback
collision_monitor->set_violation_callback([](const CollisionEfficiencyViolation& v) {
    std::cerr << "Frame " << v.frame_number 
              << ": Low collision efficiency " << (v.efficiency * 100.0) << "%\n";
    std::cerr << "  Broadphase candidates: " << v.broadphase_candidates 
              << ", Actual collisions: " << v.actual_collisions << "\n";
    std::cerr << "  Hint: Consider increasing spatial grid cell size\n";
});

// During collision detection
collision_monitor->set_broadphase_candidates(candidate_pairs.size());
collision_monitor->set_narrowphase_tests(tests_performed);
collision_monitor->set_actual_collisions(confirmed_collisions);
collision_monitor->end_frame();  // Triggers efficiency check

// Get statistics
CollisionStats stats = collision_monitor->get_last_frame_stats();
std::cout << "Efficiency: " << (stats.efficiency * 100.0) << "%\n";
std::cout << "False positive rate: " << (stats.false_positive_rate * 100.0) << "%\n";
```

---

## Integration with ParticleSystem

The `ParticleSystem` class has built-in support for all monitoring systems:

```cpp
#include <core/physics/micro/particle_system.hpp>

ParticleSystem system;

// Enable physics monitoring
auto energy_monitor = std::make_shared<EnergyMonitor>();
energy_monitor->set_max_loss_percent(2.0);
system.enable_energy_monitor(energy_monitor);

auto momentum_monitor = std::make_shared<MomentumMonitor>();
momentum_monitor->set_max_change_magnitude(100.0);
system.enable_momentum_monitor(momentum_monitor);

// Enable collision monitoring (if collisions are enabled)
auto collision_monitor = std::make_shared<CollisionMonitor>();
collision_monitor->set_min_efficiency(0.1);
system.enable_collision_monitor(collision_monitor);

// Normal update - monitoring happens automatically
system.update(dt);

// Disable monitoring
system.disable_energy_monitor();
system.disable_momentum_monitor();
system.disable_collision_monitor();
```

---

## Best Practices

### When to Use Profiling

✅ **DO:**
- Profile hot loops and performance-critical sections
- Use `PROFILE_FUNCTION()` at function entry for broad overview
- Use `PROFILE_SCOPE()` for fine-grained bottleneck identification
- Clear frame data at frame boundaries to avoid memory buildup
- Disable profiling in production builds unless actively debugging

❌ **DON'T:**
- Profile trivial functions (< 10 µs execution time)
- Create deeply nested scopes (> 20 levels) - overhead adds up
- Keep profiling enabled for hours - data grows over time
- Profile inside tight loops - use outer loop scope instead

### Profiling Overhead

Measured overhead (debug builds on Windows/MinGW):
- **Empty scope**: ~50-200 ns per PROFILE_SCOPE
- **Disabled profiling**: ~1-5 ns (branch prediction)
- **Nested scopes**: Linear scaling (not exponential)
- **Frame data retrieval**: < 1 µs

Optimized builds (Release with -O2/-O3) typically see 10x lower overhead.

### Physics Monitoring Best Practices

**Energy Monitoring:**
- Enable during development and testing
- Set tolerances based on integrator accuracy (semi-implicit: 1-2% loss acceptable)
- Account for collision energy dissipation (restitution < 1.0)
- Include potential energy from gravity/spring fields
- Reset baseline after adding/removing particles

**Momentum Monitoring:**
- Essential for validating collision resolution
- In isolated systems (no forces), momentum should be constant
- Set tolerance to account for numerical drift (~0.01% per frame acceptable)
- With external forces (gravity), momentum *should* change - disable or account for it

**Collision Efficiency:**
- Low efficiency (<5%) indicates poor spatial partitioning
- Increase grid cell size if false positive rate is high
- Ideal efficiency: 10-30% (depends on particle density)
- 100% efficiency means no spatial culling - re-enable broadphase!

---

## Performance Tuning Guide

### Identifying Bottlenecks

1. **Enable profiling**:
   ```cpp
   Profiler::enable(true);
   ```

2. **Profile your update loop**:
   ```cpp
   void game_loop() {
       PROFILE_FUNCTION();
       
       {
           PROFILE_SCOPE("input");
           process_input();
       }
       
       {
           PROFILE_SCOPE("physics");
           physics_system.update(dt);
       }
       
       {
           PROFILE_SCOPE("rendering");
           render_scene();
       }
   }
   ```

3. **Analyze frame data**:
   ```cpp
   for (const auto& zone : Profiler::get_zones()) {
       if (zone.duration_us > 1000) {  // More than 1ms
           std::cout << zone.name << ": " 
                     << zone.duration_us / 1000.0 << " ms\n";
       }
   }
   ```

### Common Issues & Solutions

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Frame time spikes | GC/allocations | Pre-allocate, use object pools |
| Consistent slow frame | Hot loop bottleneck | Profile inner scopes, optimize algorithm |
| Energy loss | Integration errors | Lower timestep, use better integrator |
| Low collision efficiency | Grid cell size | Increase cell size to 2-4x particle diameter |
| Momentum drift | Floating point accumulation | Use double precision, smaller timesteps |

---

## Example: Complete Instrumentation

```cpp
#include <core/physics/micro/particle_system.hpp>
#include <core/diagnostics/frame_profiler.hpp>
#include <core/diagnostics/frame_budget.hpp>
#include <core/diagnostics/energy_monitor.hpp>

class PhysicsEngine {
public:
    PhysicsEngine() {
        // Setup frame profiling
        frame_profiler_.set_history_size(60);
        
        // Setup frame budget
        frame_budget_.set_target_fps(60.0);
        frame_budget_.set_zone_budget("physics", 5000);  // 5ms
        frame_budget_.set_violation_callback([](const FrameBudgetViolation& v) {
            std::cerr << "Frame budget exceeded!\n";
        });
        
        // Setup energy monitoring
        energy_monitor_ = std::make_shared<EnergyMonitor>();
        energy_monitor_->set_max_loss_percent(2.0);
        energy_monitor_->set_violation_callback([](const EnergyViolation& v) {
            std::cerr << "Energy loss detected: " << v.loss_percentage << "%\n";
        });
        particle_system_.enable_energy_monitor(energy_monitor_);
        
        // Enable profiling
        Profiler::enable(true);
    }
    
    void update(float dt) {
        PROFILE_FUNCTION();
        
        frame_profiler_.begin_frame();
        frame_budget_.begin_frame();
        
        {
            PROFILE_SCOPE("physics");
            particle_system_.update(dt);
        }
        
        frame_budget_.end_frame();
        frame_profiler_.end_frame();
        
        // Report statistics every second
        if (frame_profiler_.get_frame_count() % 60 == 0) {
            print_statistics();
        }
        
        // Clear profiling data for next frame
        Profiler::clear_frame();
    }
    
    void print_statistics() {
        auto stats = frame_profiler_.get_zone_stats("physics");
        std::cout << "Physics: avg=" << stats.avg_duration_us / 1000.0 << " ms"
                  << ", max=" << stats.max_duration_us / 1000.0 << " ms\n";
        
        auto last_frame = frame_profiler_.get_last_frame();
        std::cout << "Frame time: " << last_frame.total_frame_time_us / 1000.0 << " ms\n";
    }
    
private:
    ParticleSystem particle_system_;
    FrameProfiler frame_profiler_;
    FrameBudget frame_budget_;
    std::shared_ptr<EnergyMonitor> energy_monitor_;
};
```

---

## Troubleshooting

### Profiling Data Not Appearing

- Verify `Profiler::enable(true)` was called
- Check that `PROFILE_SCOPE` macros are present
- Ensure code is actually executing (breakpoint test)
- Call `Profiler::get_zones()` before `clear_frame()`

### High Profiling Overhead

- Reduce number of profiled scopes
- Move profiling to outer loops only
- Disable profiling in release builds
- Check for deep nesting (>20 levels)

### Energy/Momentum False Positives

- Adjust tolerance thresholds
- Verify integrator is stable (try smaller timestep)
- Check for particle additions/removals affecting baseline
- Use double precision for accumulation
- Reset monitors after major state changes

### Collision Efficiency Always Low

- Spatial grid cell size too small - increase it
- Particles too clustered - check spawn distribution
- Broadphase disabled - re-enable spatial partitioning
- Particle radius not matching collision radius

---

## API Reference Summary

### Profiling
- `Profiler::enable(bool)` - Enable/disable profiling
- `Profiler::is_enabled()` - Check if enabled
- `Profiler::get_zones()` - Get frame zones
- `Profiler::clear_frame()` - Clear frame data
- `PROFILE_FUNCTION()` - Profile function
- `PROFILE_SCOPE(name)` - Profile scope

### Frame Profiling
- `FrameProfiler::begin_frame()` - Start frame
- `FrameProfiler::end_frame()` - End frame
- `FrameProfiler::get_zone_stats(name)` - Get statistics
- `FrameProfiler::get_last_frame()` - Get last frame data

### Frame Budget
- `FrameBudget::set_target_fps(fps)` - Set target frame rate
- `FrameBudget::set_zone_budget(name, us)` - Set zone budget
- `FrameBudget::set_violation_callback(fn)` - Set callback

### Energy Monitor
- `EnergyMonitor::update(energy)` - Update energy
- `EnergyMonitor::set_max_loss_percent(pct)` - Set loss tolerance
- `EnergyMonitor::set_max_gain_percent(pct)` - Set gain tolerance
- `EnergyMonitor::set_violation_callback(fn)` - Set callback

### Momentum Monitor
- `MomentumMonitor::update(Vec3)` - Update momentum
- `MomentumMonitor::set_max_change_magnitude(mag)` - Set tolerance
- `MomentumMonitor::set_violation_callback(fn)` - Set callback

### Collision Monitor
- `CollisionMonitor::set_broadphase_candidates(count)` - Set candidates
- `CollisionMonitor::set_narrowphase_tests(count)` - Set tests
- `CollisionMonitor::set_actual_collisions(count)` - Set collisions
- `CollisionMonitor::end_frame()` - Calculate efficiency
- `CollisionMonitor::get_last_frame_stats()` - Get statistics

---

## Further Reading

- [Architecture](architecture.md) - System design overview
- [Roadmap](roadmap.md) - Development timeline
- [References](references.md) - Algorithm references and papers

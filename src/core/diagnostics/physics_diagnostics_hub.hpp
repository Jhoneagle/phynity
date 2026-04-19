#pragma once

#include <core/diagnostics/collision_monitor.hpp>
#include <core/diagnostics/energy_monitor.hpp>
#include <core/diagnostics/momentum_monitor.hpp>
#include <core/math/vectors/vec3.hpp>

#include <memory>

namespace phynity::diagnostics
{

/// Manages optional physics diagnostics monitors (energy, momentum, collision).
/// Extracted from ParticleSystem to reduce class complexity.
class PhysicsDiagnosticsHub
{
public:
    // ========================================================================
    // Energy Monitoring
    // ========================================================================

    void enable_energy_monitor(std::shared_ptr<EnergyMonitor> monitor)
    {
        energy_monitor_ = monitor;
        energy_monitor_enabled_ = true;
    }

    void disable_energy_monitor()
    {
        energy_monitor_enabled_ = false;
    }

    // ========================================================================
    // Momentum Monitoring
    // ========================================================================

    void enable_momentum_monitor(std::shared_ptr<MomentumMonitor> monitor)
    {
        momentum_monitor_ = monitor;
        momentum_monitor_enabled_ = true;
    }

    void disable_momentum_monitor()
    {
        momentum_monitor_enabled_ = false;
    }

    // ========================================================================
    // Collision Monitoring
    // ========================================================================

    void enable_collision_monitor(std::shared_ptr<CollisionMonitor> monitor)
    {
        collision_monitor_ = monitor;
        collision_monitor_enabled_ = true;
    }

    void disable_collision_monitor()
    {
        collision_monitor_enabled_ = false;
    }

    // ========================================================================
    // Reporting
    // ========================================================================

    /// Report physics diagnostics (energy and momentum) for the current frame.
    void report_physics(float kinetic_energy, const math::vectors::Vec3f &momentum)
    {
        if (energy_monitor_enabled_ && energy_monitor_)
        {
            energy_monitor_->update(static_cast<double>(kinetic_energy));
        }

        if (momentum_monitor_enabled_ && momentum_monitor_)
        {
            Vec3 mom(static_cast<double>(momentum.x), static_cast<double>(momentum.y), static_cast<double>(momentum.z));
            momentum_monitor_->update(mom);
        }
    }

    /// Report collision statistics for the current frame.
    void report_collisions(uint32_t broadphase, uint32_t narrowphase, uint32_t actual)
    {
        if (collision_monitor_enabled_ && collision_monitor_)
        {
            collision_monitor_->set_broadphase_candidates(broadphase);
            collision_monitor_->set_narrowphase_tests(narrowphase);
            collision_monitor_->set_actual_collisions(actual);
            collision_monitor_->end_frame();
        }
    }

    /// Check if any physics monitor (energy or momentum) is active.
    bool has_active_physics_monitors() const
    {
        return (energy_monitor_enabled_ && energy_monitor_) || (momentum_monitor_enabled_ && momentum_monitor_);
    }

    /// Check if collision monitor is active.
    bool has_active_collision_monitor() const
    {
        return collision_monitor_enabled_ && collision_monitor_;
    }

private:
    std::shared_ptr<EnergyMonitor> energy_monitor_;
    std::shared_ptr<MomentumMonitor> momentum_monitor_;
    std::shared_ptr<CollisionMonitor> collision_monitor_;
    bool energy_monitor_enabled_ = false;
    bool momentum_monitor_enabled_ = false;
    bool collision_monitor_enabled_ = false;
};

} // namespace phynity::diagnostics

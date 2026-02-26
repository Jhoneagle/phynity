#pragma once

#include "profiler.hpp"
#include "frame_profiler.hpp"
#include "frame_budget.hpp"
#include "energy_monitor.hpp"
#include "momentum_monitor.hpp"
#include "collision_monitor.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <unordered_set>

namespace phynity::diagnostics {

/**
 * @brief Console output utilities for profiling data.
 * 
 * Provides formatted text output for quick debugging and development.
 */
class ConsoleOutput {
public:
    /**
     * @brief Print all zones from current profiling frame.
     * 
     * Displays zones in hierarchical format with indentation for nested scopes.
     * 
     * @param indent_size Number of spaces per nesting level (default: 2)
     */
    static void print_frame_zones(int indent_size = 2) {
        const auto& zones = Profiler::get_zones();
        
        if (zones.empty()) {
            std::cout << "No profiling data collected (profiling disabled or no scopes active)\n";
            return;
        }
        
        std::cout << "\n=== Frame Profile ===\n";
        for (const auto& zone : zones) {
            // Indent based on depth
            const std::string indent(zone.depth * indent_size, ' ');
            
            // Format duration
            const double ms = zone.duration_us / 1000.0;
            
            std::cout << indent 
                      << std::left << std::setw(30 - zone.depth * indent_size) << zone.name
                      << std::right << std::setw(8) << std::fixed << std::setprecision(3) << ms 
                      << " ms\n";
        }
        std::cout << "====================\n\n";
    }
    
    /**
     * @brief Print frame profiler statistics.
     * 
     * Shows aggregate statistics for all zones over the profiler's history window.
     * 
     * @param profiler Frame profiler instance
     */
    static void print_frame_stats(const FrameProfiler& profiler) {
        // Get unique zone names from last frame
        const auto last_frame = profiler.get_last_frame();
        std::unordered_set<std::string_view> zone_names;
        for (const auto& zone : last_frame.zones) {
            zone_names.insert(zone.name);
        }
        
        if (zone_names.empty()) {
            std::cout << "No frame statistics available\n";
            return;
        }
        
        std::cout << "\n=== Frame Statistics ===\n";
        std::cout << std::left << std::setw(25) << "Zone"
                  << std::right << std::setw(10) << "Avg (ms)"
                  << std::setw(10) << "Min (ms)"
                  << std::setw(10) << "Max (ms)"
                  << std::setw(10) << "Count" << "\n";
        std::cout << std::string(65, '-') << "\n";
        
        for (const auto& name : zone_names) {
            const auto stats = profiler.get_zone_stats(name);
            
            std::cout << std::left << std::setw(25) << name
                      << std::right << std::setw(10) << std::fixed << std::setprecision(3) 
                      << (stats.average_duration_us() / 1000.0)
                      << std::setw(10) << (stats.min_duration_us / 1000.0)
                      << std::setw(10) << (stats.max_duration_us / 1000.0)
                      << std::setw(10) << stats.call_count << "\n";
        }
        
        // Show total frame time from last frame
        const auto last_frame = profiler.get_last_frame();
        const double frame_ms = last_frame.total_frame_time_us / 1000.0;
        
        std::cout << std::string(65, '-') << "\n";
        std::cout << std::left << std::setw(25) << "Total Frame Time"
                  << std::right << std::setw(10) << std::fixed << std::setprecision(3) 
                  << frame_ms << " ms\n";
        std::cout << "========================\n\n";
    }
    
    /**
     * @brief Print a simple one-line frame summary.
     * 
     * Useful for real-time monitoring during simulation.
     * 
     * @param profiler Frame profiler instance
     */
    static void print_frame_summary(const FrameProfiler& profiler) {
        const auto last_frame = profiler.get_last_frame();
        const double frame_ms = last_frame.total_frame_time_us / 1000.0;
        const double fps = (frame_ms > 0.0) ? (1000.0 / frame_ms) : 0.0;
        
        std::cout << "Frame " << profiler.get_frame_count() 
                  << ": " << std::fixed << std::setprecision(2) << frame_ms << " ms"
                  << " (" << std::fixed << std::setprecision(1) << fps << " fps)"
                  << " | " << last_frame.zones.size() << " zones\n";
    }
    
    /**
     * @brief Print frame budget report.
     * 
     * @param budget Frame budget instance
     */
    static void print_budget_report(const FrameBudget& budget) {
        std::cout << "\n=== Frame Budget Report ===\n";
        std::cout << "Target FPS: " << budget.get_target_fps() << "\n";
        std::cout << "Frame budget: " << std::fixed << std::setprecision(2) 
                  << (budget.get_target_frame_time() / 1000.0) << " ms\n";
        std::cout << "Enabled: " << (budget.is_enabled() ? "Yes" : "No") << "\n";
        std::cout << "===========================\n\n";
    }
    
    /**
     * @brief Print energy monitor status.
     * 
     * @param monitor Energy monitor instance
     */
    static void print_energy_status(const EnergyMonitor& monitor) {
        std::cout << "\n=== Energy Monitor ===\n";
        std::cout << "Enabled: " << (monitor.is_enabled() ? "Yes" : "No") << "\n";
        std::cout << "Frame: " << monitor.get_frame_count() << "\n";
        std::cout << "Current energy: " << std::fixed << std::setprecision(2) 
                  << monitor.get_current_energy() << "\n";
        std::cout << "======================\n\n";
    }
    
    /**
     * @brief Print momentum monitor status.
     * 
     * @param monitor Momentum monitor instance
     */
    static void print_momentum_status(const MomentumMonitor& monitor) {
        const auto momentum = monitor.get_current_momentum();
        const double magnitude = momentum.magnitude();
        
        std::cout << "\n=== Momentum Monitor ===\n";
        std::cout << "Enabled: " << (monitor.is_enabled() ? "Yes" : "No") << "\n";
        std::cout << "Frame: " << monitor.get_frame_count() << "\n";
        std::cout << "Current momentum: (" << std::fixed << std::setprecision(2)
                  << momentum.x << ", " << momentum.y << ", " << momentum.z << ")\n";
        std::cout << "Magnitude: " << magnitude << "\n";
        std::cout << "========================\n\n";
    }
    
    /**
     * @brief Print collision monitor status.
     * 
     * @param monitor Collision monitor instance
     */
    static void print_collision_status(const CollisionMonitor& monitor) {
        const auto stats = monitor.get_last_frame_stats();
        
        std::cout << "\n=== Collision Monitor ===\n";
        std::cout << "Enabled: " << (monitor.is_enabled() ? "Yes" : "No") << "\n";
        std::cout << "Frame: " << monitor.get_frame_count() << "\n";
        std::cout << "Broadphase candidates: " << stats.broadphase_candidates << "\n";
        std::cout << "Narrowphase tests: " << stats.narrowphase_tests << "\n";
        std::cout << "Actual collisions: " << stats.actual_collisions << "\n";
        std::cout << "Efficiency: " << std::fixed << std::setprecision(1) 
                  << (stats.efficiency * 100.0) << "%\n";
        std::cout << "False positive rate: " << (stats.false_positive_rate * 100.0) << "%\n";
        std::cout << "Min efficiency threshold: " << (monitor.get_min_efficiency() * 100.0) << "%\n";
        std::cout << "=========================\n\n";
    }
    
    /**
     * @brief Print a comprehensive diagnostics report.
     * 
     * Combines all monitoring systems into a single report.
     */
    static void print_full_diagnostics_report(
        const FrameProfiler* frame_profiler = nullptr,
        const FrameBudget* frame_budget = nullptr,
        const EnergyMonitor* energy_monitor = nullptr,
        const MomentumMonitor* momentum_monitor = nullptr,
        const CollisionMonitor* collision_monitor = nullptr
    ) {
        std::cout << "\n";
        std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                    DIAGNOSTICS REPORT                          ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════════╝\n";
        
        if (frame_profiler) {
            print_frame_stats(*frame_profiler);
        }
        
        if (frame_budget) {
            print_budget_report(*frame_budget);
        }
        
        if (energy_monitor) {
            print_energy_status(*energy_monitor);
        }
        
        if (momentum_monitor) {
            print_momentum_status(*momentum_monitor);
        }
        
        if (collision_monitor) {
            print_collision_status(*collision_monitor);
        }
        
        std::cout << "╔════════════════════════════════════════════════════════════════╗\n\n";
    }
};

}  // namespace phynity::diagnostics

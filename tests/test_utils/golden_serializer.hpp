#pragma once

#include <core/physics/particle.hpp>
#include <core/physics/particle_system.hpp>
#include <core/math/vectors/vec3.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace phynity::test {

using phynity::physics::Particle;
using phynity::physics::ParticleSystem;
using phynity::math::vectors::Vec3f;

// ============================================================================
// Serialized State Snapshot
// ============================================================================

/// Captures a snapshot of particle system state at a single frame
struct SerializedState {
    std::vector<Vec3f> positions;
    std::vector<Vec3f> velocities;
    std::vector<float> masses;
    uint64_t frame_number = 0;
    float elapsed_time = 0.0f;
    
    /// Check if two states are equal within tolerance
    bool equals(const SerializedState& other, float tolerance = 1e-6f) const {
        if (frame_number != other.frame_number) return false;
        if (positions.size() != other.positions.size()) return false;
        if (velocities.size() != other.velocities.size()) return false;
        
        for (size_t i = 0; i < positions.size(); ++i) {
            float pos_dist = (positions[i] - other.positions[i]).length();
            if (pos_dist > tolerance) return false;
        }
        
        for (size_t i = 0; i < velocities.size(); ++i) {
            float vel_dist = (velocities[i] - other.velocities[i]).length();
            if (vel_dist > tolerance) return false;
        }
        
        return true;
    }
};

// ============================================================================
// Golden File Serializer
// ============================================================================

class GoldenSerializer {
public:
    /// Serialize a particle system state to JSON string
    /// @param state The state snapshot to serialize
    /// @return JSON string representation
    static std::string to_json(const SerializedState& state) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(9);
        
        oss << "{\n";
        oss << "  \"frame_number\": " << state.frame_number << ",\n";
        oss << "  \"elapsed_time\": " << state.elapsed_time << ",\n";
        
        // Positions array
        oss << "  \"positions\": [\n";
        for (size_t i = 0; i < state.positions.size(); ++i) {
            const Vec3f& p = state.positions[i];
            oss << "    {";
            oss << "\"x\": " << p.x << ", ";
            oss << "\"y\": " << p.y << ", ";
            oss << "\"z\": " << p.z;
            oss << "}";
            if (i < state.positions.size() - 1) oss << ",";
            oss << "\n";
        }
        oss << "  ],\n";
        
        // Velocities array
        oss << "  \"velocities\": [\n";
        for (size_t i = 0; i < state.velocities.size(); ++i) {
            const Vec3f& v = state.velocities[i];
            oss << "    {";
            oss << "\"x\": " << v.x << ", ";
            oss << "\"y\": " << v.y << ", ";
            oss << "\"z\": " << v.z;
            oss << "}";
            if (i < state.velocities.size() - 1) oss << ",";
            oss << "\n";
        }
        oss << "  ],\n";
        
        // Masses array
        oss << "  \"masses\": [\n";
        for (size_t i = 0; i < state.masses.size(); ++i) {
            oss << "    " << state.masses[i];
            if (i < state.masses.size() - 1) oss << ",";
            oss << "\n";
        }
        oss << "  ]\n";
        
        oss << "}\n";
        return oss.str();
    }
    
    /// Serialize a sequence of states to JSON string
    /// @param states Vector of state snapshots
    /// @return JSON string representation
    static std::string trajectory_to_json(const std::vector<SerializedState>& states) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(9);
        
        oss << "{\n";
        oss << "  \"trajectory\": [\n";
        
        for (size_t frame_idx = 0; frame_idx < states.size(); ++frame_idx) {
            const SerializedState& state = states[frame_idx];
            
            oss << "    {\n";
            oss << "      \"frame_number\": " << state.frame_number << ",\n";
            oss << "      \"elapsed_time\": " << state.elapsed_time << ",\n";
            
            oss << "      \"positions\": [\n";
            for (size_t i = 0; i < state.positions.size(); ++i) {
                const Vec3f& p = state.positions[i];
                oss << "        {\"x\": " << p.x << ", \"y\": " << p.y << ", \"z\": " << p.z << "}";
                if (i < state.positions.size() - 1) oss << ",";
                oss << "\n";
            }
            oss << "      ],\n";
            
            oss << "      \"velocities\": [\n";
            for (size_t i = 0; i < state.velocities.size(); ++i) {
                const Vec3f& v = state.velocities[i];
                oss << "        {\"x\": " << v.x << ", \"y\": " << v.y << ", \"z\": " << v.z << "}";
                if (i < state.velocities.size() - 1) oss << ",";
                oss << "\n";
            }
            oss << "      ],\n";
            
            oss << "      \"masses\": [";
            for (size_t i = 0; i < state.masses.size(); ++i) {
                oss << state.masses[i];
                if (i < state.masses.size() - 1) oss << ", ";
            }
            oss << "]\n";
            
            oss << "    }";
            if (frame_idx < states.size() - 1) oss << ",";
            oss << "\n";
        }
        
        oss << "  ]\n";
        oss << "}\n";
        return oss.str();
    }
    
    /// Load a JSON state string from file
    /// @param filepath Path to golden file
    /// @return JSON string contents
    static std::string load_golden_file(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open golden file: " + filepath);
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }
    
    /// Save a state to golden file
    /// @param state The state to save
    /// @param filepath Path where golden file should be written
    static void save_golden_file(const SerializedState& state, const std::string& filepath) {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file for writing: " + filepath);
        }
        file << to_json(state);
    }
    
    /// Save a trajectory to golden file
    /// @param states Vector of state snapshots
    /// @param filepath Path where golden file should be written
    static void save_trajectory_golden(const std::vector<SerializedState>& states, const std::string& filepath) {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file for writing: " + filepath);
        }
        file << trajectory_to_json(states);
    }
};

// ============================================================================
// Helpers for Snapshotting Particle System
// ============================================================================

/// Create a serialized snapshot of the current particle system state
/// @param system The particle system to snapshot
/// @param frame_number Frame counter for tracking
/// @param elapsed_time Current simulation time
/// @return Serialized state snapshot
inline SerializedState snapshot_system(
    const ParticleSystem& system,
    uint64_t frame_number = 0,
    float elapsed_time = 0.0f
) {
    SerializedState state;
    state.frame_number = frame_number;
    state.elapsed_time = elapsed_time;
    
    // Access particles through public interface
    for (const auto& particle : system.particles()) {
        if (particle.active) {
            state.positions.push_back(particle.position);
            state.velocities.push_back(particle.velocity);
            state.masses.push_back(particle.material.mass);
        }
    }
    
    return state;
}

} // namespace phynity::test

#pragma once

#include <core/physics/rigid_body_system.hpp>
#include <core/physics/shape.hpp>
#include <core/physics/force_field.hpp>
#include <core/physics/constraints/fixed_constraint_rb.hpp>
#include <core/math/vectors/vec3.hpp>

namespace phynity::app::rigid_body_demos {

using namespace phynity::physics;
using namespace phynity::math::vectors;
using namespace phynity::math::quaternions;

/// Simple stacking demo: tower of boxes under gravity
class StackingDemoRB {
public:
    StackingDemoRB()
        : system_()
    {
        setup();
    }

    /// Run simulation for specified duration
    void simulate(float duration) {
        const float dt = 0.01f;  // 10ms per frame
        const int num_frames = static_cast<int>(duration / dt);
        
        for (int frame = 0; frame < num_frames; ++frame) {
            system_.update(dt);
            
            // Print diagnostics every 100 frames (1 second)
            if (frame % 100 == 0) {
                print_diagnostics(static_cast<float>(frame) * dt);
            }
        }
    }

    RigidBodySystem& get_system() { return system_; }

private:
    RigidBodySystem system_;

    void setup() {
        // Add gravity
        auto gravity = std::make_shared<GravityField>(
            Vec3f(0.0f, -9.81f, 0.0f)
        );
        system_.add_force_field(gravity);

        // Create ground (static box)
        auto ground_shape = std::make_shared<BoxShape>(
            Vec3f(10.0f, 0.5f, 10.0f)
        );
        system_.spawn_body(
            Vec3f(0.0f, -1.0f, 0.0f),
            Quatf(),
            ground_shape,
            0.0f  // Static (infinite mass)
        );

        // Create tower of boxes
        auto box_shape = std::make_shared<BoxShape>(
            Vec3f(0.5f, 0.5f, 0.5f)
        );
        
        const int num_boxes = 5;
        const float box_height = 1.1f;
        
        for (int i = 0; i < num_boxes; ++i) {
            float y = 0.5f + static_cast<float>(i) * box_height;
            
            system_.spawn_body(
                Vec3f(0.0f, y, 0.0f),
                Quatf(),
                box_shape,
                1.0f  // 1 kg
            );
        }
    }

    void print_diagnostics(float time) {
        const auto& diag = system_.get_diagnostics();
        
        printf("[%.2fs] Bodies: %zu, "
               "KE_linear: %.3f J, KE_angular: %.3f J, "
               "Total KE: %.3f J\n",
               time,
               diag.body_count,
               diag.total_linear_ke,
               diag.total_angular_ke,
               diag.total_kinetic_energy);
    }
};

/// Interactive stacking demo with toppling
class ToppleDemoRB {
public:
    ToppleDemoRB() {
        stable_demo_.simulate(2.0f);  // Let tower settle
    }

    /// Rock the tower with an impulse
    void apply_topple_impulse() {
        auto& system = stable_demo_.get_system();
        
        // Find topmost body (last spawned)
        RigidBody* top_body = system.get_body(4);  // Assuming body IDs 0-4
        
        if (top_body) {
            // Apply horizontal impulse at top
            Vec3f impulse = Vec3f(10.0f, 0.0f, 0.0f);  // 10 N-s to the right
            top_body->velocity += impulse * top_body->inv_mass;
        }
    }

    /// Simulate tower toppling
    void simulate() {
        apply_topple_impulse();
        
        printf("\n=== Toppling Simulation ===\n");
        stable_demo_.simulate(5.0f);  // Watch it fall
    }

private:
    StackingDemoRB stable_demo_;
};

/// Example demonstrating hinge constraint (door)
class HingeDemoRB {
public:
    HingeDemoRB() {
        setup();
    }

    void simulate(float duration) {
        const float dt = 0.01f;
        const int num_frames = static_cast<int>(duration / dt);
        
        for (int frame = 0; frame < num_frames; ++frame) {
            system_.update(dt);
            
            if (frame % 50 == 0 && door_body_) {
                printf("[%.2fs] Door angle: %.2f°\n",
                       static_cast<float>(frame) * dt,
                       std::acos(std::clamp(door_body_->orientation.w, -1.0f, 1.0f)) * 2.0f);
            }
        }
    }

private:
    RigidBodySystem system_;
    RigidBody* door_body_;

    void setup() {
        // Frame (static)
        auto frame_shape = std::make_shared<BoxShape>(Vec3f(2.0f, 3.0f, 0.1f));
        RigidBody* frame = system_.get_body(
            system_.spawn_body(
                Vec3f(0.0f, 0.0f, 0.0f),
                Quatf(),
                frame_shape,
                0.0f  // Static
            )
        );

        // Door (dynamic)
        auto door_shape = std::make_shared<BoxShape>(Vec3f(0.05f, 3.0f, 1.0f));
        door_body_ = system_.get_body(
            system_.spawn_body(
                Vec3f(1.0f, 0.0f, 0.0f),
                Quatf(),
                door_shape,
                5.0f  // 5 kg
            )
        );

        // Hinge constraint at door pivot
        auto hinge = std::make_shared<constraints::HingeConstraintRB>(
            frame,
            door_body_,
            Vec3f(0.0f, 1.5f, 0.0f),   // Pivot on frame
            Vec3f(-0.05f, 1.5f, 0.0f), // Pivot on door
            Vec3f(0.0f, 1.0f, 0.0f)    // Hinge axis (vertical)
        );
        system_.add_constraint(hinge);

        // Apply initial spin
        if (door_body_) {
            door_body_->angular_velocity = Vec3f(0.0f, 2.0f, 0.0f);
        }
    }
};

}  // namespace phynity::app::rigid_body_demos

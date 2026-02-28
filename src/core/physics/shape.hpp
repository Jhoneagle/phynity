#pragma once

#include <core/math/vectors/vec3.hpp>
#include <core/math/matrices/mat3.hpp>
#include <vector>
#include <memory>
#include <cmath>

namespace phynity::physics {

using phynity::math::vectors::Vec3f;
using phynity::math::matrices::Mat3f;

/// Enumeration for supported shape types
enum class ShapeType {
    Sphere,
    Box,
    Capsule
};

/// Abstract base class for collision shapes.
/// Defines interface for inertia tensor computation and geometric queries.
class Shape {
public:
    virtual ~Shape() = default;

    ShapeType shape_type;
    Vec3f local_center;  ///< Center offset in body-local coordinates

    explicit Shape(ShapeType type, const Vec3f& center = Vec3f(0.0f))
        : shape_type(type), local_center(center) {}

    // ========================================================================
    // Inertia Tensor Computation
    // ========================================================================

    /// Compute body-space inertia tensor and its inverse
    /// @param mass The mass of the body
    /// @param[out] I The computed inertia tensor (body-space)
    /// @param[out] I_inv The computed inverse inertia tensor (body-space)
    virtual void compute_inertia_tensors(
        float mass,
        Mat3f& I,
        Mat3f& I_inv
    ) const = 0;

    // ========================================================================
    // Geometric Queries (for collision detection)
    // ========================================================================

    /// Get vertices of the shape (for SAT/GJK, if applicable)
    /// For sphere: returns single point (center)
    /// For box: returns 8 corner points
    /// For capsule: returns approximation (endpoints + ring)
    virtual std::vector<Vec3f> get_vertices() const = 0;

    /// Support function for GJK: find furthest point in given direction
    /// @param direction The query direction (should be normalized)
    /// @return The furthest point on the shape in that direction
    virtual Vec3f support_function(const Vec3f& direction) const = 0;

    /// Get bounding sphere radius (for broadphase AABB)
    virtual float get_bounding_radius() const = 0;

    /// Clone this shape into a unique_ptr
    virtual std::unique_ptr<Shape> clone() const = 0;
};

// ============================================================================
// SPHERE SHAPE
// ============================================================================

/// Represents a spherical shape
class SphereShape : public Shape {
public:
    float radius = 0.5f;

    explicit SphereShape(float r = 0.5f, const Vec3f& center = Vec3f(0.0f))
        : Shape(ShapeType::Sphere, center), radius(r) {}

    void compute_inertia_tensors(
        float mass,
        Mat3f& I,
        Mat3f& I_inv
    ) const override {
        // Sphere: I = (2/5) * m * r²
        // Diagonal tensor (same value on diagonal)
        float diag_val = (2.0f / 5.0f) * mass * radius * radius;
        
        I = Mat3f(0.0f);
        I(0, 0) = diag_val;
        I(1, 1) = diag_val;
        I(2, 2) = diag_val;
        
        // Inverse
        if (diag_val > 1e-6f) {
            float inv_diag = 1.0f / diag_val;
            I_inv = Mat3f(0.0f);
            I_inv(0, 0) = inv_diag;
            I_inv(1, 1) = inv_diag;
            I_inv(2, 2) = inv_diag;
        } else {
            I_inv = Mat3f(0.0f);  // Zero inertia inverse (body is massless)
        }
    }

    std::vector<Vec3f> get_vertices() const override {
        return { local_center };  // Single point for collision tests
    }

    Vec3f support_function(const Vec3f& direction) const override {
        // Furthest point on sphere in given direction
        Vec3f normalized_dir = direction.normalized();
        return local_center + normalized_dir * radius;
    }

    float get_bounding_radius() const override {
        return radius;
    }

    std::unique_ptr<Shape> clone() const override {
        return std::make_unique<SphereShape>(*this);
    }
};

// ============================================================================
// BOX SHAPE
// ============================================================================

/// Represents an axis-aligned box shape (for MVP; OBB extends later)
class BoxShape : public Shape {
public:
    Vec3f half_extents;  ///< Half-size in each dimension (x, y, z)

    explicit BoxShape(const Vec3f& half_ext = Vec3f(0.5f), const Vec3f& center = Vec3f(0.0f))
        : Shape(ShapeType::Box, center), half_extents(half_ext) {}

    void compute_inertia_tensors(
        float mass,
        Mat3f& I,
        Mat3f& I_inv
    ) const override {
        // Box (cuboid): I_diagonal = (1/12) * m * (a² + b²) for each axis
        // I_xx = (1/12) * m * (height² + depth²)  // depends on y and z
        // I_yy = (1/12) * m * (width² + depth²)   // depends on x and z
        // I_zz = (1/12) * m * (width² + height²)  // depends on x and y
        
        float a2 = half_extents.x * half_extents.x * 4.0f;  // Full dimension squared
        float b2 = half_extents.y * half_extents.y * 4.0f;
        float c2 = half_extents.z * half_extents.z * 4.0f;
        
        float coeff = mass / 12.0f;
        
        I = Mat3f(0.0f);
        I(0, 0) = coeff * (b2 + c2);  // I_xx
        I(1, 1) = coeff * (a2 + c2);  // I_yy
        I(2, 2) = coeff * (a2 + b2);  // I_zz
        
        // Inverse
        float diag_xx = I(0, 0);
        float diag_yy = I(1, 1);
        float diag_zz = I(2, 2);
        
        I_inv = Mat3f(0.0f);
        if (diag_xx > 1e-6f) I_inv(0, 0) = 1.0f / diag_xx;
        if (diag_yy > 1e-6f) I_inv(1, 1) = 1.0f / diag_yy;
        if (diag_zz > 1e-6f) I_inv(2, 2) = 1.0f / diag_zz;
    }

    std::vector<Vec3f> get_vertices() const override {
        // Return 8 corners
        std::vector<Vec3f> verts;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    Vec3f v = local_center;
                    v.x += (i == 0) ? -half_extents.x : half_extents.x;
                    v.y += (j == 0) ? -half_extents.y : half_extents.y;
                    v.z += (k == 0) ? -half_extents.z : half_extents.z;
                    verts.push_back(v);
                }
            }
        }
        return verts;
    }

    Vec3f support_function(const Vec3f& direction) const override {
        // Box support function: extend in direction of largest components
        Vec3f result = local_center;
        result.x += (direction.x >= 0.0f) ? half_extents.x : -half_extents.x;
        result.y += (direction.y >= 0.0f) ? half_extents.y : -half_extents.y;
        result.z += (direction.z >= 0.0f) ? half_extents.z : -half_extents.z;
        return result;
    }

    float get_bounding_radius() const override {
        // Maximum distance from center to corner
        return half_extents.length();
    }

    std::unique_ptr<Shape> clone() const override {
        return std::make_unique<BoxShape>(*this);
    }
};

// ============================================================================
// CAPSULE SHAPE
// ============================================================================

/// Represents a capsule shape (cylinder with hemispherical ends)
class CapsuleShape : public Shape {
public:
    float radius = 0.25f;      ///< Radius of the capsule
    float half_height = 0.5f;  ///< Half-length of the cylinder

    explicit CapsuleShape(float r = 0.25f, float h = 0.5f, const Vec3f& center = Vec3f(0.0f))
        : Shape(ShapeType::Capsule, center), radius(r), half_height(h) {}

    void compute_inertia_tensors(
        float mass,
        Mat3f& I,
        Mat3f& I_inv
    ) const override {
        // Capsule = cylinder + 2 hemispheres
        // Simplified: treat as cylinder (ignore hemisphere adjustment for MVP)
        // I_xx = I_yy ≈ m * (3*r²/4 + h²/12)
        // I_zz ≈ m * r² / 2
        // (Simplified, doesn't match exact geometry, but reasonable for MVP)
        
        float r2 = radius * radius;
        float h2 = half_height * half_height;
        
        float coeff_xy = mass * (3.0f * r2 / 4.0f + h2 / 12.0f);
        float coeff_z = mass * r2 / 2.0f;
        
        I = Mat3f(0.0f);
        I(0, 0) = coeff_xy;
        I(1, 1) = coeff_xy;
        I(2, 2) = coeff_z;
        
        // Inverse
        I_inv = Mat3f(0.0f);
        if (coeff_xy > 1e-6f) {
            I_inv(0, 0) = 1.0f / coeff_xy;
            I_inv(1, 1) = 1.0f / coeff_xy;
        }
        if (coeff_z > 1e-6f) {
            I_inv(2, 2) = 1.0f / coeff_z;
        }
    }

    std::vector<Vec3f> get_vertices() const override {
        // Approximation: endpoints + some points on cylinder
        std::vector<Vec3f> verts;
        
        // Hemisphere endpoints
        verts.push_back(local_center + Vec3f(0, half_height + radius, 0));
        verts.push_back(local_center - Vec3f(0, half_height + radius, 0));
        
        // Cylinder endpoints
        verts.push_back(local_center + Vec3f(0, half_height, 0));
        verts.push_back(local_center - Vec3f(0, half_height, 0));
        
        // Ring of points
        int ring_points = 6;
        for (int i = 0; i < ring_points; ++i) {
            float angle = 2.0f * 3.14159f * static_cast<float>(i) / static_cast<float>(ring_points);
            Vec3f p;
            p.x = local_center.x + radius * std::cos(angle);
            p.z = local_center.z + radius * std::sin(angle);
            p.y = local_center.y;
            verts.push_back(p);
        }
        
        return verts;
    }

    Vec3f support_function(const Vec3f& direction) const override {
        Vec3f normalized = direction.normalized();
        Vec3f result = local_center;
        result += normalized * radius;  // Extend by radius
        
        // Extend along axis
        float axis_support = (normalized.y >= 0) ? half_height : -half_height;
        result.y += axis_support;
        
        return result;
    }

    float get_bounding_radius() const override {
        // Maximum distance from center to furthest point
        return std::sqrt(radius * radius + half_height * half_height + radius * radius);
    }

    std::unique_ptr<Shape> clone() const override {
        return std::make_unique<CapsuleShape>(*this);
    }
};

}  // namespace phynity::physics

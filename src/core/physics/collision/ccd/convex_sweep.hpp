#pragma once

#include <core/physics/collision/ccd/toi_solver.hpp>
#include <core/physics/collision/narrowphase/gjk_solver.hpp>
#include <core/physics/collision/narrowphase/support_function.hpp>
#include <core/physics/macro/shape.hpp>

#include <algorithm>
#include <cmath>

namespace phynity::physics::collision::ccd
{

using phynity::math::vectors::Vec3f;

/// Conservative advancement CCD for convex shapes using GJK distance queries.
/// Assumes linear motion and fixed orientation over the timestep.
class ConvexSweepSolver
{
public:
	struct MovingConvex
	{
		const phynity::physics::Shape *shape = nullptr;
		Vec3f position = Vec3f(0.0f);
		Vec3f velocity = Vec3f(0.0f);
		float expansion = 0.0f; // Minkowski expansion radius (for rotational CCD)
	};

	/// Solve for time of impact between two moving convex shapes.
	/// @param a Moving shape A
	/// @param b Moving shape B
	/// @param dt Timestep duration (seconds)
	/// @param max_iterations Maximum conservative advancement iterations
	/// @param separation_tolerance Distance threshold for contact
	static TimeOfImpactResult solve(const MovingConvex &a,
	                                const MovingConvex &b,
	                                float dt,
	                                int max_iterations = 16,
	                                float separation_tolerance = 1e-4f)
	{
		TimeOfImpactResult result;
		if (dt <= 0.0f || !a.shape || !b.shape)
		{
			return result;
		}

		const float safe_dt = dt > 1e-8f ? dt : 1.0f;
		float time_normalized = 0.0f;
		const float tolerance = std::max(separation_tolerance, 1e-6f);

		for (int iter = 0; iter < max_iterations; ++iter)
		{
			Vec3f pos_a = a.position + a.velocity * (safe_dt * time_normalized);
			Vec3f pos_b = b.position + b.velocity * (safe_dt * time_normalized);

			MacroShapeSupportFunction support_a(a.shape, pos_a, a.expansion);
			MacroShapeSupportFunction support_b(b.shape, pos_b, b.expansion);

			GJKResult gjk_result = GJKSolver::solve(support_a, support_b);

			float separation = gjk_result.distance;
			Vec3f separation_vec = gjk_result.closest_point_b - gjk_result.closest_point_a;
			if (separation_vec.squaredLength() < 1e-10f)
			{
				separation_vec = pos_b - pos_a;
			}
			Vec3f normal =
			    separation_vec.squaredLength() > 1e-10f ? separation_vec.normalized() : Vec3f(1.0f, 0.0f, 0.0f);

			if (gjk_result.collision || separation <= tolerance)
			{
				Vec3f support_point_a = a.shape->support_function(normal) + pos_a + normal * a.expansion;
				Vec3f support_point_b = b.shape->support_function(-normal) + pos_b - normal * b.expansion;
				Vec3f contact_point = (support_point_a + support_point_b) * 0.5f;
				Vec3f rel_vel = b.velocity - a.velocity;

				result.collision_occurs = true;
				result.toi = std::clamp(time_normalized, 0.0f, 1.0f);
				result.contact_point = contact_point;
				result.contact_normal = normal;
				result.relative_velocity = rel_vel;
				return result;
			}

			Vec3f rel_vel = b.velocity - a.velocity;
			float approach_speed = -rel_vel.dot(normal);
			if (approach_speed <= 1e-6f)
			{
				return result; // Not approaching
			}

			float advance_physical = separation / approach_speed;
			float advance_normalized = advance_physical / safe_dt;

			if (advance_normalized <= 0.0f)
			{
				return result;
			}

			time_normalized += advance_normalized;
			if (time_normalized > 1.0f)
			{
				return result;
			}
		}

		return result;
	}

private:
	/// Adapter for macro shapes to the support function interface.
	class MacroShapeSupportFunction : public SupportFunction
	{
	public:
		MacroShapeSupportFunction(const phynity::physics::Shape *shape, const Vec3f &origin, float expansion)
		    : shape_(shape), origin_(origin), expansion_(expansion)
		{
		}

		Vec3f get_support_point(const Vec3f &direction) const override
		{
			if (!shape_)
			{
				return origin_;
			}
			Vec3f dir = direction;
			if (dir.squaredLength() > 1e-10f)
			{
				dir = dir.normalized();
			}
			return shape_->support_function(dir) + origin_ + dir * expansion_;
		}

		Vec3f get_origin() const override
		{
			return origin_;
		}

	private:
		const phynity::physics::Shape *shape_;
		Vec3f origin_;
		float expansion_;
	};
};

} // namespace phynity::physics::collision::ccd

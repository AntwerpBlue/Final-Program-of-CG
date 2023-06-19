#pragma once
#include "Eigen/Dense"
#include "ray.hpp"
#include <limits>
class AABB
{
	using v3f = Eigen::Vector3f;
	using v6f = Eigen::Vector<float, 6>;

public:
	v6f limits;
	AABB() : limits(v6f::Zero()) {}
	AABB(float x, float X, float y, float Y, float z, float Z) :limits(v6f(x, X, y, Y, z, Z)) {}
	AABB(v6f v) :limits(v) {}

	// test intersection with a ray
	bool is_ray_inter(Ray& ray)
	{
		float t_min[3] = { 1e20, 1e20, 1e20 }, t_max[3] = {-1e20, -1e20, -1e20};
		
		for (int axis = 0; axis < 3; ++axis)
		{
			float delta1 = limits(axis * 2) - ray.pos(axis);
			float delta2 = limits(axis * 2+1) - ray.pos(axis);
			float t1, t2;
			if (std::abs(ray.dir(axis)) < 1e-20)
			{
				t1 = delta1 >= 0 ? 1e20 : -1e20;
				t2 = delta2 >=0 ? 1e20 : -1e20;
			}
			else
			{
				t1 = delta1 / ray.dir(axis);
				t2 = delta2 / ray.dir(axis);
			}

			t_min[axis] = std::min(t1, t2);
			t_max[axis] = std::max(t1, t2);
		}

		float t_m = std::max({ t_min[0], t_min[1], t_min[2] });
		float t_M = std::max( { t_max[0], t_max[1], t_max[2] });

		if (t_M >= 0 && t_m <= t_M) return true;
		else return false;
	}

	// get the union of two boxes
	AABB union_with_other(AABB& another)
	{
		v6f new_lim = v6f::Zero();
		for (int ax = 0; ax < 3; ++ax)
		{
			new_lim(ax * 2) = std::min(limits(ax * 2), another.limits(ax * 2));
			new_lim(ax * 2 + 1) = std::max(limits(ax * 2 + 1), another.limits(ax * 2 + 1));
		}
		return AABB(new_lim);
	}

	// the center
	v3f get_center()
	{
		return v3f((limits(0) + limits(1)) / 2, (limits(2) + limits(3)) / 2, (limits(4) + limits(5)) / 2);
	}
};

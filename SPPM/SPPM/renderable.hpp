#pragma once
#include <random>
#include "Eigen/Dense"
#include "ray.hpp"
#include <vector>
#include "photon.hpp"

class Renderable
{
public:
	int n_materials;
	bool has_spec;
	bool has_lamb;
	bool has_tran;
	bool is_light;

	enum InteractType
	{
		SPEC, LAMB, TRAN
	};

	float spec_ratio;
	float lamb_ratio;
	float tran_ratio;

	using v3f = Eigen::Vector3f;
	v3f color;
	float emit_power;
	float n_t;
	AABB aabb;

	Renderable(int n_mat, bool sp, bool lb, bool tr, bool lt, float spr, float lbr, float trr, v3f c, float emt, float n) :
		n_materials(n_mat),
		has_spec(sp),
		has_lamb(lb),
		has_tran(tr),
		is_light(lt),
		spec_ratio(spr),
		lamb_ratio(lbr),
		tran_ratio(trr),
		color(c),
		emit_power(emt),
		n_t(n)
	{
		std::random_device dev;
		rng = std::mt19937(dev());
		if (sp) available_mat.push_back(std::make_pair(SPEC, spec_ratio));
		if (lb) available_mat.push_back(std::make_pair(LAMB, lamb_ratio));
		if (tr) available_mat.push_back(std::make_pair(TRAN, tran_ratio));
	}


	virtual std::pair<Ray, InteractType> get_next_ray_type(Ray& in, v3f pos) = 0;
	virtual v3f get_bsdf(InteractType type, v3f in_dir, v3f out_dir, v3f pos) = 0;
	virtual std::pair<float, v3f> try_intersect_ray(Ray& in) = 0;
	virtual Photon emit_a_photon() = 0;
	virtual v3f get_norm() = 0;


	std::mt19937 rng;
	std::vector<std::pair<InteractType, float>> available_mat;
};


class Triangle: public Renderable
{
public:
	v3f v0, v1, v2; // three points
	v3f norm;

	Triangle(v3f pv0, v3f pv1, v3f pv2, int n_mat, bool is_sp, bool is_lb, bool is_tr, bool is_lt, float sp_ratio, float lb_ratio, float tr_ratio,
		v3f c, float emit, float p_n) :
		Renderable(n_mat,
			is_sp,
			is_lb,
			is_tr,
			is_lt,
			sp_ratio,
			lb_ratio,
			tr_ratio,
			c,
			emit,
			p_n),
		v0(pv0), v1(pv1), v2(pv2)
	{
		norm = (v1 - v0).cross(v2 - v0).normalized();

		float low[3], up[3];
		for (int axis = 0; axis < 3; ++axis)
		{
			low[axis] = std::min({ v0(axis), v1(axis), v2(axis) });
			up[axis] = std::max({v0(axis), v1(axis), v2(axis)});
		}

		aabb = AABB(low[0], up[0], low[1], up[1], low[2], up[2]);
	}

	virtual std::pair<float, v3f> try_intersect_ray(Ray& in) override
	{
		v3f E1 = v1 - v0;
		v3f E2 = v2 - v0;
		v3f S = in.pos - v0;
		v3f S1 = in.dir.cross(E2);
		v3f S2 = S.cross(E1);

		v3f tb1b2 = v3f(S2.dot(E2), S1.dot(S), S2.dot(in.dir)) / S1.dot(E1);

		if (tb1b2(0) > 1e-20
			&& tb1b2(1) >= 0 && tb1b2(1) <= 1
			&& tb1b2(2) >= 0 && tb1b2(2) <= 1
			&& (1 - tb1b2(1) - tb1b2(2)) >= 0 && (1 - tb1b2(1) - tb1b2(2)) <= 1)
		{
			return std::make_pair(tb1b2(0), in.get_t(tb1b2(0)));
		}
		else
			return std::make_pair(-114514.0f, v3f::Zero());
	}
	virtual std::pair<Ray, InteractType> get_next_ray_type(Ray& in, v3f pos) override
	{
		std::uniform_real_distribution<float> choose_material(0, 1);
		float thresh = choose_material(rng);
		InteractType current_type = LAMB;
		float acc = 0.0f;
		for (auto m : available_mat)
		{
			acc += m.second;
			if (acc >= thresh)
			{
				current_type = m.first;
				break;
			}
		}

		if (current_type == LAMB) // not realated with the ray in
		{
			std::uniform_real_distribution<float> choose_cos_phi(-1, 1);
			std::uniform_real_distribution<float> choose_theta(0, 2 * EIGEN_PI);
			float u = choose_cos_phi(rng);
			float theta = choose_theta(rng);

			// get the sample direction in the local coordinate
			float x = std::sqrt(1 - u * u) * std::cos(theta);
			float y = std::sqrt(1 - u * u) * std::sin(theta);
			float z = std::abs(u);

			// generate the local coordinate
			v3f x_axis = (v1 - v0).normalized();
			v3f y_axis = norm.cross(x_axis);

			v3f dir = x * x_axis + y * y_axis + z * norm;
			Ray ret(pos, dir.normalized());
			return std::make_pair(ret, LAMB);
		}
		else if (current_type == SPEC) // use the ray in to calculate the reflected ray
		{
			v3f dir_in = -in.dir;
			v3f dir_out = 2 * norm * norm.dot(dir_in) - dir_in;

			Ray ret(pos, dir_out);
			return std::make_pair(ret, SPEC);
		}
		else if (current_type == TRAN) // use the ray in and the n to calculate the ray
		{
			v3f dir_in = -in.dir;
			float cos_theta = -in.dir.dot(norm);
			if (cos_theta >= 0)
			{ // inward
				float in_sin = std::sqrt(1 - cos_theta * cos_theta) / n_t;
				float in_cos = std::sqrt(1 - in_sin * in_sin);

				// the "x-axis" is -norm
				v3f y_axis = (dir_in.dot(norm) * norm - dir_in).normalized();
				v3f dir_out = -norm * in_cos + y_axis * in_sin;
				Ray ret(pos, dir_out);
				return std::make_pair(ret, TRAN);

			}
			else
			{ // outward
				float in_sin = std::sqrt(1 - cos_theta * cos_theta) * n_t;
				float in_cos = std::sqrt(1 - in_sin * in_sin);
				v3f y_axis = (dir_in.dot(norm) * norm - dir_in).normalized();
				v3f dir_out = norm * in_cos + y_axis * in_sin;
				Ray ret(pos, dir_out);
				return std::make_pair(ret, TRAN);

			}
		}
		else // default
		{
			return std::make_pair(in, TRAN); // return itself
		}
	}

	virtual v3f get_bsdf(InteractType type, v3f in_dir, v3f out_dir, v3f pos)
	{
		if (type == SPEC)
		{
			// assume the direction is correct!
			return color;
		}
		else if (type == LAMB)
		{
			return color / EIGEN_PI;
		}
		else if (type == TRAN)
		{
			// assume the direction is correct!
			return color;
		}
		else
		{
			return v3f::Zero();
		}
	}

	virtual Photon emit_a_photon() override
	{
		// cosine weighted
		float A = (v1 - v0).cross(v2 - v0).norm() / 2; // the area
		// the power is Pi*A*Le
		v3f power = EIGEN_PI * A * color * emit_power;

		std::uniform_real_distribution<float> u01(0, 1);
		std::uniform_real_distribution<float> u11(-1, 1);

		float u = u01(rng), v = u01(rng);
		if (u + v > 1)
		{
			u = 1 - u;
			v = 1 - v;
		}
		v3f emit_position = (v1 - v0) * u + (v2 - v0) * v;
		
		float x = u11(rng), y = u11(rng);

		if (x * x + y * y > 1)
		{
			x = u11(rng); y = u11(rng);
		}
		float z = std::sqrt(1 - x * x - y * y);
		v3f x_axis = (v1 - v0).normalized();
		v3f y_axis = norm.cross(x_axis);
		v3f dir = x_axis * x + y_axis * y + z * norm;
		return Photon(power, emit_position, dir);
	}
	virtual v3f get_norm() override
	{
		return norm;
	}

};




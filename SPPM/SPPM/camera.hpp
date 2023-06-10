#pragma once
#include"Eigen/Dense"
#include "ray.hpp"
#include <vector>
#include "scene.hpp"
#include "renderable.hpp"
#include <queue>
#include "photon_map.hpp"
#include "photon_kdt.hpp"
class Cam
{
public:
	using v3f = Eigen::Vector3f;
	const int N = 20;
	v3f top, lookat, pos;
	int w, h;
	float focus, fov;
	float ccd_w, ccd_h;
	int sub_x, sub_y;
	v3f local_x_ax;

	std::vector<v3f> radiance_rec;



	Cam(v3f p, v3f t, v3f l, int w, int h, float deg_fovY ,int subx, int suby,float focus = 1.0f) : 
		pos(p), top(t), lookat(l), focus(focus), w(w), h(h), fov(deg_fovY), sub_x(subx), sub_y(suby)
	{
		float asp = float(w) / float(h);
		float rad_half_fov = deg_fovY / 2 / 180 * EIGEN_PI;
		
		ccd_h = focus * std::tan(rad_half_fov) * 2;
		ccd_w = ccd_h * asp;
		local_x_ax = lookat.cross(top).normalized();
		radiance_rec.resize(w * h);
	}

	Ray sample_ray_xy(int x, int y, int idx, int idy)
	{
		if (x >= w || y >= h || x < 0 || y < 0) return Ray();
		v3f center(ccd_w / 2, ccd_h / 2, -focus);
		float ccdx = (float(x) + float(idx) / sub_x) / w * ccd_w;
		float ccdy = (float(y) + float(idy) / sub_y) / h * ccd_h;

		v3f ccd(ccdx, ccdy, 0);
		v3f vec = ccd - center;
		
		v3f world_dir = (vec(0) * local_x_ax + vec(1) * (-top) + vec(2) * lookat).normalized();
		return Ray(pos, world_dir);
	}
	

	v3f single_sample(int x, int y, int sx, int sy, Scene & sc, KDT& kdt)
	{
		const int N = 1000;
		const int max_reflection = 128;
		v3f all_radiance(0, 0, 0);
		Ray sample = sample_ray_xy(x, y, sx, sy);

		IntersectInfo intersect = sc.bvh->get_first_intersection(sample, nullptr);

		if (intersect.do_intersect) // actually have intersection
		{
			auto next_ray_type = intersect.obj->get_next_ray_type(sample, intersect.pos);
			int cnt = 0; bool not_found = false;
			while (next_ray_type.second != Renderable::LAMB) // if it's not a lambertain reflection
			{
				sample = next_ray_type.first;
				intersect = sc.bvh->get_first_intersection(sample, intersect.obj); // get new intersection
				if (!intersect.do_intersect) return all_radiance;
				next_ray_type = intersect.obj->get_next_ray_type(sample, intersect.pos);
				cnt++;
				if (cnt > max_reflection)
				{
					not_found = true;
					break;
				}
			}

			if (not_found)
			{
				return all_radiance;
			}
			


			// the first lambertain obj
			auto near_ph = kdt.N_near(N,intersect.pos);
			v3f ref_power(0, 0, 0);
			for (auto p : near_ph)
			{
				v3f bsdf = intersect.obj->get_bsdf(Renderable::LAMB, -p->ray.dir, -sample.dir, intersect.pos);
				ref_power += bsdf.cwiseProduct(p->power_rgb);
			}

			float r = (near_ph[0]->ray.pos - intersect.pos).norm();
			ref_power /= EIGEN_PI * r * r * kdt.ptrs.size();
			if (intersect.obj->is_light) all_radiance += intersect.obj->emit_power * intersect.obj->color;
			all_radiance += ref_power;
		}
		return all_radiance;
	}
	
};
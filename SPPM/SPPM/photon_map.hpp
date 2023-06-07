#pragma once
#include <random>
#include<vector>
#include "renderable.hpp"
#include "photon.hpp"
#include "scene.hpp"
class PhotonMap
{
public:
	using v3f = Eigen::Vector3f;
	std::vector<Photon> photon_record;
	int n;
	PhotonMap():n(0) {}
	PhotonMap(Scene& sc, int n):n(n) 
	{
		get_map(n, sc);
	}

	void get_map(int n, Scene& sc ,int single_max_iter = 32, float rr_prob = 0.8)
	{
		this->n = n;
		std::random_device dev;
		std::mt19937 rng(dev());
		std::uniform_int_distribution<int> choose_light(0, sc.light_src.size() - 1);
		std::uniform_real_distribution<float> RR(0, 1);
		photon_record.clear();
		for (int iter = 0; iter < n; ++iter)
		{
			Renderable* light = sc.light_src[choose_light(rng)];
			Photon ph = light ->emit_a_photon();
			// iteration
			for (int ph_iter = 0; ph_iter < single_max_iter; ++ph_iter)
			{
				
				// get the first intersection

				IntersectInfo info = sc.bvh->get_first_intersection(ph.ray);
				if (!info.do_intersect) break;
				auto ray_tp = info.obj->get_next_ray_type(ph.ray, info.pos);
				if (ray_tp.second == Renderable::InteractType::LAMB)
				{
					// RR
					if (RR(rng) > rr_prob) break;
					photon_record.push_back(ph);

					// lambertain reflection
					v3f bsdf = info.obj->get_bsdf(Renderable::LAMB, -ph.ray.dir, ray_tp.first.dir, info.pos);
					v3f R = 2 * EIGEN_PI * bsdf * ray_tp.first.dir.dot(info.obj->get_norm());
					v3f new_power = R.cwiseProduct(ph.power_rgb) / info.obj->n_materials;
					// update the photons
					ph.ray = ray_tp.first;
					ph.power_rgb = new_power;
				}
				else if (ray_tp.second == Renderable::InteractType::SPEC)
				{
					ph.ray = ray_tp.first;
					//photon_record.push_back(ph);
				}
				else if (ray_tp.second == Renderable::InteractType::TRAN)
				{
					ph.ray = ray_tp.first;
					//photon_record.push_back(ph);
				} 
				// else: do nothing
			}
		}
	}
};
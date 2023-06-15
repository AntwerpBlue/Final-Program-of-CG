#pragma once
#include <random>
#include<vector>
#include "renderable.hpp"
#include "photon.hpp"
#include "scene.hpp"
#include <thread>
class PhotonMap
{
public:
	using v3f = Eigen::Vector3f;
	std::vector<Photon> photon_record;
	int n;
	PhotonMap():n(0) {}
	PhotonMap(Scene& sc, int n):n(n) 
	{
		get_map_mt(1, n, sc);
	}


	void get_map_mt(int n_th, int n, Scene& sc, int single_max_iter = 32, float rr_prob = 0.8)
	{
		photon_record.clear();
		std::vector<std::vector<Photon>> photon_record_vec(8);
		std::vector<std::thread> threads;
		this->n = n;

		for (int i = 0; i < n_th; ++i)
		{
			threads.push_back(std::thread(
				[](std::vector<Photon>& phr,const int n, Scene& sc,const int single_max_,const float rr_prob) 
				{
					std::random_device dev;
					std::mt19937 rng(dev());
					std::uniform_int_distribution<int> choose_light(0, sc.light_src.size() - 1);
					std::uniform_real_distribution<float> RR(0, 1);
					phr.clear();
					for (int iter = 0; iter < n; ++iter)
					{
						Renderable* light = sc.light_src[choose_light(rng)];
						Photon ph = light->emit_a_photon(rng);
						Renderable* last = nullptr;
						// iteration
						for (int ph_iter = 0; ph_iter < single_max_; ++ph_iter)
						{

							// get the first intersection

							IntersectInfo info = sc.bvh->get_first_intersection(ph.ray, last);
							if (!info.do_intersect) break;
							auto ray_tp = info.obj->get_next_ray_type(ph.ray, info.pos, rng);
							last = info.obj;
							if (ray_tp.second == Renderable::InteractType::LAMB)
							{
								// RR
								if (RR(rng) > rr_prob)
								{
									ph.ray.pos = info.pos;
									phr.push_back(ph);
									break;
								}


								// lambertain reflection
								v3f bsdf = info.obj->get_bsdf(Renderable::LAMB, -ph.ray.dir, ray_tp.first.dir, info.pos);
								v3f R = 2 * EIGEN_PI * bsdf * std::abs(ph.ray.dir.dot(info.obj->get_norm()));
								v3f new_power = R.cwiseProduct(ph.power_rgb) / info.obj->n_materials;
								//new_power = v3f(200, 200 ,200);
								// update the photons
								ph.ray.pos = info.pos;
								ph.power_rgb = new_power;

								phr.push_back(Photon(ph)); 
								ph.ray = ray_tp.first;
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
				},
				std::ref(photon_record_vec[i]), n/n_th, std::ref(sc), single_max_iter, rr_prob));
		}

		for (auto& i : threads) i.join();

		for (auto& a : photon_record_vec)
		{
			photon_record.insert(photon_record.end(), a.begin(), a.end());
		}
	}

};
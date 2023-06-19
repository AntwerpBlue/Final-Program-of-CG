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
	std::vector<Photon> caustic_photon_record;
	int n_global, n_caustic;
	PhotonMap():n_global(0), n_caustic(0) {}

	void get_map_mt(int n_th, int n, Scene& sc, int single_max_iter = 32, float rr_prob = 0.8)
	{
		photon_record.clear();
		std::vector<std::vector<Photon>> photon_record_vec(n_th);
		std::vector<std::thread> threads;
		this->n_global = n;

		for (int i = 0; i < n_th; ++i)
		{
			threads.push_back(
				std::thread([](PhotonMap* pm_ptr, Scene& sc, int single_max_iter, float rr_prob, int n_th, std::vector<Photon>& phr)
					{
						// calculate global map here
						int n_photons = pm_ptr->n_global / n_th; // how many photons do we need
						std::random_device dev;
						std::mt19937 rng(dev());
						std::uniform_int_distribution<> choose_light(0, sc.light_src.size() - 1);
						std::uniform_real_distribution<> RR(0, 1);
						for (int i = 0; i < n_photons; ++i)
						{
							// choose a light source
					
							Renderable* light = sc.light_src[choose_light(rng)];

							// emit a photon
							Photon ph = light->emit_a_photon();
							ph.power_rgb *= sc.light_src.size(); // scaling
							// get the first intersection
							IntersectInfo info = sc.bvh->get_first_intersection(ph.ray, light);
							Renderable* last_obj = nullptr;
							for (int iter = 0; iter < single_max_iter; iter++) // transport
							{
								if (!info.do_intersect) break; // no intersection
								last_obj = info.obj;
								// else: have intersection
								auto type = info.obj->sample_reflect_type(); // get the type
								using TP = Renderable::InteractType;
								if (type == TP::LAMB)
								{
									Ray next_ray = info.obj->get_next_ray(type, ph.ray, info.pos);
									v3f bsdf = info.obj->get_bsdf(type, -ph.ray.dir, next_ray.dir, info.pos);
									v3f R = 2 * EIGEN_PI * bsdf * std::abs(ph.ray.dir.dot(info.obj->get_norm(info.pos)));
									v3f new_power = ph.power_rgb.cwiseProduct(R) * info.obj->lamb_ratio / info.obj->n_materials;
									// new update the photon


									Photon newph = ph;
									ph.bounce = true;
									ph.end_at = info.pos;

									phr.push_back(ph);
									if (RR(rng) > rr_prob)
										break;
									newph.power_rgb = new_power;
									newph.ray = next_ray;
									ph = newph;
								}
								else if (type == TP::SPEC)
								{
									Ray next_ray = info.obj->get_next_ray(type, ph.ray, info.pos);
									v3f bsdf = info.obj->get_bsdf(type, -ph.ray.dir, next_ray.dir, info.pos);
									ph.ray = next_ray;
									ph.power_rgb = ph.power_rgb.cwiseProduct(bsdf) / info.obj->n_materials;
								}
								else if (type == TP::TRAN)
								{
									Ray next_ray = info.obj->get_next_ray(type, ph.ray, info.pos);
									v3f bsdf = info.obj->get_bsdf(type, -ph.ray.dir, next_ray.dir, info.pos);
									ph.ray = next_ray;
									ph.power_rgb = ph.power_rgb.cwiseProduct(bsdf) / info.obj->n_materials;
								}
								// else, do nothing

								info = sc.bvh->get_first_intersection(ph.ray, last_obj);

							}

						}
					},
					this, std::ref(sc), single_max_iter, rr_prob, n_th, std::ref(photon_record_vec[i]))
			);
		}
		for (auto& t : threads) t.join();
		for (auto& a : photon_record_vec)
		{
			photon_record.insert(photon_record.end(), a.begin(), a.end());
		}
	}


	
};
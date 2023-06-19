#pragma once
#include "camera.hpp"
#include<vector>
#include <random>
#include "Eigen/Dense"
#include <thread>


constexpr float dec_fac = 0.7f;
struct Q
{
	using v3f = Eigen::Vector3f;
	v3f pos, dir;
	Renderable* obj;
	v3f beta;
	bool islt;
	Q() :pos(v3f::Zero()), dir(v3f::Zero()), obj(nullptr), beta(v3f::Ones()) , islt(false) {}
};

struct SppmPix
{

	using v3f = Eigen::Vector3f;

	v3f direct_radiance;
	int direct_radiance_cnt;

	// pp part
	bool is_first_estimate; // if the iteration is the first time
	float global_shared_radius; // the estimation radius
	v3f global_cumulate_pow; // power
	int global_N; // how many photons

	int needed_N;
	// samples
	std::vector<Q> sample_position_dir_obj_beta; // the direction points outside

	// init
	SppmPix() :is_first_estimate(true), global_shared_radius(-1),
		global_cumulate_pow(v3f::Zero()),global_N(0), direct_radiance(v3f::Zero()), direct_radiance_cnt(0), needed_N(0) {}
};

class SppmManager
{
public:
	using v3f = Eigen::Vector3f;
	using v3b = Eigen::Vector3<unsigned char>;
	Cam camera;
	std::vector<SppmPix> sppm_pix_list;
	std::mt19937 rng;
	std::uniform_real_distribution<float> u01;
	int single_sp;
	SppmManager(Cam& cam) :camera(cam), single_sp(0)
	{
		std::random_device dev;
		rng = std::mt19937(dev());
		u01 = std::uniform_real_distribution<float>(0, 1);
		sppm_pix_list = std::vector<SppmPix>(cam.w * cam.h); // allocate mem
	}

	inline int get_pix_id(int x, int y) { return y * camera.w + x; }

	void initialize_samples_mt(Scene & sc, int th = 8, int single_samples = 32, int max_ray_reflect = 16)
	{
		single_sp = single_samples;
		using TP = Renderable::InteractType;
		std::vector<std::pair<int, int>> st_ed_vec;
		int interval = camera.w / th;

		int st = 0, ed = interval;
		while (st < camera.w)
		{
			if (ed <= camera.w)
			{
				st_ed_vec.push_back(std::make_pair(st, ed));
				st = ed;
				ed += interval;
			}
			else
			{
				st_ed_vec.push_back(std::make_pair(st, camera.w));
				break;
			}
		}
		
		std::vector<std::thread> threads;
		for (int i = 0; i < st_ed_vec.size(); ++i)
		{
			threads.push_back(std::thread([](SppmManager* man, int single_samples, Scene& sc, int st, int ed, int h, Cam& cam, int max_ray_reflect)
				{
					std::random_device dev;
					std::mt19937 rng(dev());
					std::uniform_real_distribution<> u01(0,1);
					for (int x = st; x < ed; ++x)
					{
						for (int y = 0; y < h; ++y)
						{
							for (int iter = 0; iter < single_samples; ++iter)
							{
								float dx = u01(rng);
								float dy = u01(rng);
								Ray sample_r = cam.sample_ray_xy(x, y, dx, dy);
								Renderable* lastobj = nullptr;
								IntersectInfo info = sc.bvh->get_first_intersection(sample_r, nullptr);
								Q sample_point;

								for (int ray_ref = 0; ray_ref < max_ray_reflect; ++ray_ref)
								{
									if (!info.do_intersect) break;
									lastobj = info.obj;
									// do have intersection
									TP type = info.obj->sample_reflect_type(); // choose a reflection type
									if (type == TP::LAMB)
									{
										// calculate the beta and store the position
										sample_point.obj = info.obj;
										sample_point.pos = info.pos;
										sample_point.dir = -sample_r.dir;
										//sample_point.beta /= info.obj->lamb_ratio;
										// here we do not calculate the bsdf 
										man->sppm_pix_list[y * cam.w + x].sample_position_dir_obj_beta.push_back(sample_point);
										break; // stop the transporting of this ray
									}
									else if (type == TP::SPEC)
									{
										// update the ray and go to the next iteration

										v3f in_dir = -sample_r.dir;
										sample_r = info.obj->get_next_ray(type, sample_r, info.pos);
										v3f bsdf = info.obj->get_bsdf(type, in_dir, sample_r.dir, info.pos);
										sample_point.beta = sample_point.beta.cwiseProduct(bsdf);
									}
									else if (type == TP::TRAN)
									{
										// same as above
										v3f in_dir = -sample_r.dir;
										sample_r = info.obj->get_next_ray(type, sample_r, info.pos);
										v3f bsdf = info.obj->get_bsdf(type, in_dir, sample_r.dir, info.pos);
										sample_point.beta = sample_point.beta.cwiseProduct(bsdf) ;
									}

									info = sc.bvh->get_first_intersection(sample_r, lastobj);
								}
							}
						}
					}
				},
				this, single_samples, std::ref(sc), st_ed_vec[i].first, st_ed_vec[i].second, camera.h, std::ref(camera), max_ray_reflect));
		}
		for (auto& p : threads) p.join();
	}


	void single_run(Scene& sc, int th = 8,
		const int global_scale = 50000,
		const int global_map_iter = 32,
		const int global_search_N = 100,
		const float global_search_R = 0.05,
		const int all_sp = 32)
	{
		std::vector<std::pair<int, int>> st_ed_vec;
		int interval = camera.w / th;

		int st = 0, ed = interval;
		while (st < camera.w)
		{
			if (ed <= camera.w)
			{
				st_ed_vec.push_back(std::make_pair(st, ed));
				st = ed;
				ed += interval;
			}
			else
			{
				st_ed_vec.push_back(std::make_pair(st, camera.w));
				break;
			}
		}

		//get the maps
		PhotonMap mp;
		mp.get_map_mt(th, global_scale, sc, global_map_iter);

		KDT kdt_global(mp.photon_record);

		std::vector<std::thread> threads;//multi thread

		for (int i = 0; i < st_ed_vec.size(); ++i)
		{
			threads.push_back(
				std::thread(
					[](SppmManager* mgr, Scene& sc, int st, int ed, KDT& globalkdt, int global_N, float glb_R,
						int global_scale, int allsp)
					{
						std::random_device dev;
						std::mt19937 rng(dev());
						// traverse each pixel
				

						for (int x = st; x < ed; ++x)
						{
							for (int y = 0; y < mgr->camera.h; ++y)
							{
								SppmPix& pix = mgr->sppm_pix_list[y * mgr->camera.w + x];

								if (pix.sample_position_dir_obj_beta.size() > 0)
								{

										std::uniform_int_distribution<> choose_sp(0, pix.sample_position_dir_obj_beta.size() - 1);
										Q& sp = pix.sample_position_dir_obj_beta[choose_sp(rng)];
										float ratio = float(pix.sample_position_dir_obj_beta.size()) / allsp;
									if (std::find(sc.light_src.begin(), sc.light_src.end(), sp.obj) != sc.light_src.end())
									{
										// hit the light
										pix.direct_radiance = pix.direct_radiance_cnt * pix.direct_radiance + sp.obj->emit_power * sp.obj->color*ratio;
										pix.direct_radiance_cnt++;
										pix.direct_radiance /= pix.direct_radiance_cnt;

									}
									else
									{
										pix.direct_radiance *= float(pix.direct_radiance_cnt) / float(pix.direct_radiance_cnt + 1);
										pix.direct_radiance_cnt++;
									}

									if (pix.is_first_estimate)
									{
										auto near = globalkdt.N_near_with_R(global_N, sp.pos, glb_R);
										if (near.size() == 0) continue;
										pix.is_first_estimate = false;
										
										v3f flux(0, 0, 0);
										for (auto& p : near)
										{
											v3f bsdf = sp.obj->get_bsdf(Renderable::LAMB, p->ray.dir, sp.dir, sp.pos);
											flux += 2 * EIGEN_PI * bsdf.cwiseProduct(p->power_rgb) * std::abs(sp.obj->get_norm(sp.pos).dot(p->ray.dir));
										}
										flux /= global_scale;
										pix.global_N = near.size();
										pix.needed_N = near.size();
										pix.global_shared_radius = glb_R;
										pix.global_cumulate_pow = flux.cwiseProduct(sp.beta)*ratio;
									}
									else if (pix.needed_N>0)
									{
										auto near = globalkdt.N_near_with_R(std::min(int(1.02 * pix.needed_N), global_N), sp.pos, pix.global_shared_radius);
										v3f flux(0, 0, 0);
										for (auto& p : near)
										{
											v3f bsdf = sp.obj->get_bsdf(Renderable::LAMB, p->ray.dir, sp.dir, sp.pos);
											flux += 2 * EIGEN_PI * bsdf.cwiseProduct(p->power_rgb) * std::abs(sp.obj->get_norm(sp.pos).dot(p->ray.dir));
										}
										flux /= global_scale;
										flux = flux.cwiseProduct(sp.beta);
										int M = near.size();
										float fac = (float(pix.global_N) + dec_fac * M) / (pix.global_N + M);
										pix.global_shared_radius *= std::sqrt(fac);
										pix.global_N += int(dec_fac * M);
										pix.needed_N = near.size();
										pix.global_cumulate_pow = (pix.global_cumulate_pow + flux*ratio) * fac;
									}
								}
							}
						}
					},
					// parameters
					this, std::ref(sc), st_ed_vec[i].first, st_ed_vec[i].second, std::ref(kdt_global),global_search_N,
						global_search_R ,global_scale,all_sp
			)
			);
		}
		for (auto& p : threads) p.join();
	}


	v3b get_value(float scale , int x, int y)
	{

		SppmPix& p = sppm_pix_list[get_pix_id(x, y)];
		v3f power(0, 0, 0);
		if(p.global_N > 0)
			power = (p.global_cumulate_pow / p.global_N / EIGEN_PI / (p.global_shared_radius * p.global_shared_radius));
		power += p.direct_radiance;
		v3b res;
		//std::cout << power << std::endl;
		for (int i = 0; i < 3; ++i)
		{
			res(i) = unsigned char(std::floor(std::min(power(i) / scale, 1.f) * 0xff));
		}
		return res;
	}
};
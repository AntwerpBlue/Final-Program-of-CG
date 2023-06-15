#pragma once
#include "camera.hpp"
#include<vector>
#include <random>
#include "Eigen/Dense"
#include <thread>

constexpr float dec_fac = 0.7f;
constexpr int first_time_N = 200; // how many photons is needed in the first round
constexpr int n_samples_singlepix = 128; // the maximum of samples in one pixel
constexpr int map_scale = 200000; // how many photons in a single run
constexpr int max_sample_reflection = 5; // how many times a sample ray can reflection in the scene
constexpr float first_radius = 0.07f;
struct Trip
{
	using v3f = Eigen::Vector3f;
	v3f pos, dir;
	Renderable* obj;
};

struct SppmPix
{

	using v3f = Eigen::Vector3f;
	bool is_first_estimate; // if the iteration is the first time
	float shared_radius;
	v3f cumulate_pow;
	std::vector<Trip> sample_position_dirin; // the direction points outside
	int N; // how many photons
	int current_needed_neighbors;
	SppmPix() :is_first_estimate(true), shared_radius(-1), cumulate_pow(v3f::Zero()), N(0) , current_needed_neighbors(0) {}
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
	SppmManager(Cam& cam) :camera(cam)
	{
		std::random_device dev;
		rng = std::mt19937(dev());
		u01 = std::uniform_real_distribution<float>(0, 1);
		sppm_pix_list = std::vector<SppmPix>(cam.w * cam.h); // allocate mem
	}

	inline int get_pix_id(int x, int y) { return y * camera.w + x; }

	void initialize_samples(Scene & sc)
	{
		// traverse each pixel
		for (int x = 0; x < camera.w; ++x)
		{
			for (int y = 0; y < camera.h; ++y)
			{
				std::cout << "Generating Sample Points x:" << x << "\ty:" << y << "\n";
				for (int idx_spl = 0; idx_spl < n_samples_singlepix; ++idx_spl)
				{
					float localx = u01(rng);
					float localy = u01(rng);
					Ray sample_r = camera.sample_ray_xy(x, y, localx, localy);

					IntersectInfo info = sc.bvh->get_first_intersection(sample_r, nullptr);
					if (!info.do_intersect) continue; // no intersection, go to the next loop

					auto next_r_t = info.obj->get_next_ray_type(sample_r, info.pos);
					int times = 0; bool not_found = false;
					v3f last_ray_dir = sample_r.dir;
					while (next_r_t.second != Renderable::LAMB) // if not lambertain
					{
						last_ray_dir = next_r_t.first.dir; // record the direction
						info = sc.bvh->get_first_intersection(next_r_t.first, info.obj);
						if (!info.do_intersect)
						{
							not_found = true;
							break;
						}
						// have a intersection
						next_r_t = info.obj->get_next_ray_type(next_r_t.first, info.pos);
						++times;
						if (times > max_sample_reflection)
						{
							not_found = true;
							break;
						}
					}

					if (not_found) continue; // no result, go to the next iteration

					//sppm_pix_list[get_pix_id(x, y)].sample_position_dirin.push_back(std::make_pair(info.pos, -last_ray_dir));
					sppm_pix_list[get_pix_id(x, y)].sample_position_dirin.push_back(Trip{info.pos, -last_ray_dir, info.obj});
				}
			}
		}
	}
	

	void single_run(Scene &sc, int th = 8)
	{
		PhotonMap mp;
		mp.get_map_mt(th, map_scale, sc, 5);
		KDT kdt(mp.photon_record);
	
		/*for (auto& p : mp.photon_record)
		{
			std::cout << p.power_rgb << std::endl;
		}*/

		// traverse each pixel
		for (int x = 0; x < camera.w; ++x)
		{
			for (int y = 0; y < camera.h; ++y)
			{
				SppmPix& pix = sppm_pix_list[get_pix_id(x, y)];
				if (pix.sample_position_dirin.size() == 0) continue;
				std::uniform_int_distribution<int> choose_spl(0, pix.sample_position_dirin.size()-1);
				Trip& pos_dir = pix.sample_position_dirin[choose_spl(rng)];
				if (pix.is_first_estimate)
				{
					auto near = kdt.N_near_with_R(first_time_N, pos_dir.pos, first_radius);
					v3f flux(0, 0, 0);
					for (auto p : near) // get the flux
					{
						v3f bsdf = pos_dir.obj->get_bsdf(Renderable::LAMB, -p->ray.dir, pos_dir.dir, pos_dir.pos);
						flux += bsdf.cwiseProduct(p->power_rgb) / map_scale;
					} 
					float R = first_radius;
					if (pos_dir.obj->is_light) pix.cumulate_pow += pos_dir.obj->emit_a_photon().power_rgb ;
	
					pix.cumulate_pow += flux;
					pix.is_first_estimate = false;
					pix.N =  near.size();
					pix.shared_radius = R;
					pix.current_needed_neighbors = pix.N;
				}
				else // not the first round
				{

					auto near_R = kdt.N_near_with_R(first_time_N, pos_dir.pos, pix.shared_radius);
					int M1 = near_R.size();
					pix.current_needed_neighbors = M1;
					float fac = (pix.N + dec_fac * M1) / (pix.N + M1);
					float R2 = pix.shared_radius * std::sqrt(fac);
					if (x == 10 && y == 10) std::cout << M1 <<'\t' << R2 << '\n';
					v3f flux(0, 0, 0);
					for (auto p : near_R) // get the flux
					{
						v3f bsdf = pos_dir.obj->get_bsdf(Renderable::LAMB, -p->ray.dir, pos_dir.dir, pos_dir.pos);
						flux += bsdf.cwiseProduct(p->power_rgb) / map_scale;

					}
					if (pos_dir.obj->is_light) flux += pos_dir.obj->emit_a_photon().power_rgb;
					
					
					pix.cumulate_pow = (pix.cumulate_pow + flux) * fac;
					pix.N += int(dec_fac *  M1);
					pix.shared_radius = R2;
				}
			}
		}
	}


	v3b get_value(float scale , int x, int y)
	{
		SppmPix& p = sppm_pix_list[get_pix_id(x, y)];
		v3f pow = p.cumulate_pow / p.N / EIGEN_PI / (p.shared_radius * p.shared_radius);
		//v3f pow = p.cumulate_pow;
		//v3f pow = v3f(1 / p.shared_radius, 1 / p.shared_radius, 1 / p.shared_radius);
		//std::cout << "_________________________________" << std::endl;
		//std::cout << p.shared_radius << std::endl;
		//std::cout << pow << std::endl;
		//std::cout << "_________________________________" << std::endl;


		v3b res;
		for (int i = 0; i < 3; ++i)
		{
			res(i) = unsigned char(std::floor(std::min(pow(i) / scale, 1.f) * 0xff));
		}
		return res;
	}
};
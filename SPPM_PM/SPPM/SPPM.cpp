#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include"bvh.hpp"
#include "scene.hpp"
#include "photon_map.hpp"
#include <fstream>
#include "camera.hpp"
#include "photon_kdt.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize.h"
#include "SppmManager.hpp"
#include <algorithm>
#include<thread>
#include<chrono>
#include"config_parse.hpp"
using v3f = Eigen::Vector3f;

int main()
{

	SppmConf conf;
	std::string conf_filename;
	std::cout << "Input the configuration:";
	std::cin >> conf_filename;
	conf.parse(conf_filename);
	conf.print_conf();

	std::string fn = conf.model_filename;
	Scene scene;

	auto v = Scene::parse_triangle_file(fn);
	for (auto p : v)
	{
		scene.append_obj(p);
	}

	std::cout << "start constructing bvh" << '\n';

	scene.construct_bvh();

	
	Cam cam(conf.cam_pos, conf.cam_top, conf.cam_lookat, conf.rast_w, conf.rast_h, conf.fovY, 1, 1);

	int ths = std::thread::hardware_concurrency();
	std::cout << "Available Threads:" << ths << std::endl;
	SppmManager man(cam);
	man.initialize_samples_mt(scene, ths, conf.single_samples, conf.sample_bounces);
	auto now = std::chrono::high_resolution_clock::now();

	for (int i = 0; i < conf.iter_times; ++i)
	{
		std::cout << i << std::endl;
		man.single_run(scene,ths, conf.photon_emit,conf.photon_bounce,conf.first_search_N, conf.first_radius);
		auto end = std::chrono::high_resolution_clock::now();
		std::cout << "Time:" << std::chrono::duration_cast<std::chrono::seconds>(end - now).count() << std::endl;
	}

	float hd_scale;
	std::cout << "input the scale" << std::endl;
	while (true)
	{
		std::cin >> hd_scale;
		std::cout << "start write" << '\n';
		std::vector<unsigned char> pixel(cam.w * cam.h * 3);
		for (int x = 0; x < cam.w; ++x)
		{
			for (int y = 0; y < cam.h; ++y)
			{
				auto v = man.get_value(hd_scale, x, y);

				pixel[3 * (y * cam.w + x)] = v(0);
				pixel[3 * (y * cam.w + x) + 1] = v(1);
				pixel[3 * (y * cam.w + x) + 2] = v(2);
			}
		}

		stbi_write_bmp("res.bmp", cam.w, cam.h, 3, pixel.data());

	}
	return EXIT_SUCCESS;
}

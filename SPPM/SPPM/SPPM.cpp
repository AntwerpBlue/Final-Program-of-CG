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
using v3f = Eigen::Vector3f;
const float scaling_factor = 0.2;
const int iter_times = 1500;
const int w = 200;
const int h = 200;
int main()
{
	std::string fn("all_models_2.models");
	Scene scene;

	auto v = Scene::parse_triangle_file(fn);
	for (auto p : v)
	{
		scene.append_obj(p);
	}
	std::cout << "start constructing bvh" << '\n';

	scene.construct_bvh();

	
	Cam cam(v3f(2, 0, 0), v3f(0, 1, 0), v3f(-1, 0, 0), w, h, 90, 1, 1);

	

	SppmManager man(cam);
	std::cout << std::thread::hardware_concurrency() << std::endl;
	//man.initialize_samples_mt(scene, std::thread::hardware_concurrency());
	man.initialize_samples(scene);


	for (int i = 0; i<iter_times; ++i)
	{
		std::cout << "Round:" << i << '\n';
		man.single_run(scene);
	}

	float hd_scale;
	std::cout << "input the scale"<<std::endl;
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

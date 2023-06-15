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
using v3f = Eigen::Vector3f;
constexpr int w = 100;
constexpr int h = 100;

int main()
{
	std::string fn("bocchi.models");
	Scene scene;

	auto v = Scene::parse_triangle_file(fn);
	for (auto p : v)
	{
		scene.append_obj(p);
	}
	std::cout << "start constructing bvh" << '\n';

	scene.construct_bvh();

	
	Cam cam(v3f(2, 0, 0), v3f(0, 1, 0), v3f(-1, 0, 0), w, h, 90, 1, 1);

	int ths = std::thread::hardware_concurrency();
	std::cout << "Available Threads:" << ths << std::endl;
	SppmManager man(cam);
	man.initialize_samples_mt(scene, ths, 16, 5);
	auto now = std::chrono::high_resolution_clock::now();

	for (int i = 0; i < 10; ++i)
	{
		std::cout << i << std::endl;
		man.single_run(scene,ths, 100000,5,500, 0.1);
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

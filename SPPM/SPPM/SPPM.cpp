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
using v3f = Eigen::Vector3f;

int main()
{
	std::string fn("all_models_3.models");
	Scene scene;

	auto v = Scene::parse_triangle_file(fn);
	for (auto p : v)
	{
		scene.append_obj(p);
	}
	std::cout << "start constructing bvh" << '\n';

	scene.construct_bvh();
	std::cout << "start constructing map" << '\n';

	PhotonMap phm;
	//phm.get_map(100000, scene, 5);
	phm.get_map_mt(8, 100000, scene, 5);

 	std::cout << "start constructing kdt" << '\n';
	KDT kdt(phm.photon_record);


	
	Cam cam(v3f(2, 0, 0), v3f(0, 1, 0), v3f(-1, 0, 0), 400, 400, 90, 1, 1);

	const float thresh = 7.0f;
	std::cout << "start" << '\n';
	std::vector<unsigned char> pixel(cam.w * cam.h * 3);
	for (int x = 0; x < cam.w; ++x)
	{
		for (int y = 0; y < cam.h; ++y)
		{
			v3f radi(0, 0, 0);
			for (int i = 0; i < cam.sub_x*cam.sub_y; ++i)
			{
				radi += cam.single_sample(x, y, i/cam.sub_x, i%cam.sub_y, scene, kdt);
			}
			radi /= cam.sub_x * cam.sub_y;
			
			pixel[3 * (y * cam.w + x)] = char(std::min(radi(0)/thresh, 1.0f)*0xff);
			pixel[3 * (y * cam.w + x)+1] = char(std::min(radi(1)/thresh, 1.0f)*0xff);
			pixel[3 * (y * cam.w + x)+2] = char(std::min(radi(2)/thresh, 1.0f)*0xff);
		}
		if (x % 10 == 0) std::cout << x << '\n';
	}

	stbi_write_bmp("res.bmp", cam.w, cam.h, 3, pixel.data());

	std::fstream photons_file("photons.csv", std::ios::out);
	for (auto& p : phm.photon_record)
	{
		auto pos = p.ray.pos;
		photons_file << pos(0) << ',' << pos(1) << ',' << pos(2) << "\n";
	}

	photons_file.close();
	return EXIT_SUCCESS;
}

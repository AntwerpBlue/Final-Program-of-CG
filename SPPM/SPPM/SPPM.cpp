#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include"bvh.hpp"
#include "scene.hpp"
#include "photon_map.hpp"
#include <fstream>
int main()
{
	std::string fn("all_models.models");
	Scene scene;

	auto v = Scene::parse_triangle_file(fn);
	for (auto p : v)
	{
		scene.append_obj(p);
	}

	scene.construct_bvh();

	PhotonMap phm(scene, 100000);



	return EXIT_SUCCESS;
}

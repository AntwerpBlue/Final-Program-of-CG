#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include"bvh.hpp"
#include "scene.hpp"
#include "photon_map.hpp"
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


	std::cout << scene.light_src.size() << '\n';

	PhotonMap phm(scene, 10000);

	std::cout << phm.photon_record.size() << '\n';


	return EXIT_SUCCESS;
}

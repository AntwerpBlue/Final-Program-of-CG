#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include"bvh.hpp"
#include "scene.hpp"

int main()
{
	std::string fn("res.txt");
	Scene scene;

	auto v = Scene::parse_triangle_file(fn);
	for (auto p : v)
	{
		scene.append_obj(p);
	}

	scene.construct_bvh();

	std::cout << scene.objs.size();
	return EXIT_SUCCESS;
}

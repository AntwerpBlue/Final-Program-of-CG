#pragma once
#include "renderable.hpp"
#include <vector>
#include "bvh.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
class Scene
{
public:
	using v3f = Eigen::Vector3f;
	Scene(Scene&) = delete;
	Scene(Scene&&) = delete;

	std::vector<Renderable* > objs;
	std::vector<Renderable* > light_src; // this must be the subset of objs

	BVH *bvh;
	Scene() :bvh(nullptr) {}
	~Scene()
	{
		if (bvh) delete bvh;
		for (auto p : objs)
		{
			delete p;
		}
	}

	void append_obj(Renderable* obj)
	{
		objs.push_back(obj);
		if (obj->is_light)
		{
			light_src.push_back(obj);
		}
	}

	void construct_bvh()
	{
		if (bvh) delete bvh;
		bvh = new BVH(objs);
	}

	static std::vector<Renderable*> parse_triangle_file(std::string filename)
	{
		std::vector<Renderable*> res;
		std::ifstream infile(filename, std::ios::in);
		
		std::string buffer;
		while (!infile.eof())
		{
			buffer.clear();
			std::getline(infile, buffer);
			if (buffer.size() < 1) continue; // a null line
			std::stringstream ss(buffer);
			v3f vert[3];
			for (int i = 0; i < 9; ++i)
			{
				ss>> vert[i / 3](i % 3);
			}

			int n_matreial, has_spec, has_lamb, has_tran, is_light;
			ss >> n_matreial >> has_spec >> has_lamb >> has_tran >> is_light;

			float sp_r, lb_r, tr_r;
			ss >> sp_r >> lb_r>>tr_r;

			v3f color;
			ss >> color(0) >> color(1) >> color(2);
			float emit, n;
			ss >> emit >> n;

			auto p = new Triangle(
				vert[0], vert[1], vert[2],
				n_matreial,
				has_spec == 1,
				has_lamb == 1,
				has_tran == 1,
				is_light == 1,
				sp_r,
				lb_r,
				tr_r,
				color,
				emit,
				n
			);

			res.push_back(p);
		}
		return res;
	}
};
#pragma once
#include "Eigen/Dense"
#include "aabb.hpp"
#include "renderable.hpp"
#include <vector>
#include <queue>
struct BVHNd
{
	BVHNd* lc, * rc;
	AABB aabb;
	Renderable *rd_obj; // not null if it's a leaf

	BVHNd() : lc(nullptr), rc(nullptr), rd_obj(nullptr) {}
};

struct IntersectInfo
{
	using v3f = Eigen::Vector3f;
	Renderable* obj;
	v3f pos;
	float distance;
	bool do_intersect;

	IntersectInfo() : obj(nullptr), pos(v3f::Zero()), do_intersect(true), distance(-114514.0f) {};
};


class BVH
{
public:
	BVHNd* root;
	std::vector<Renderable*> obj_ptr_list;
	std::vector<BVHNd*> tree_nodes;
	BVH(BVH&) = delete;
	BVH(BVH&&) = delete;

	BVH(std::vector<Renderable*>& obj_list) :obj_ptr_list(obj_list), root(nullptr)
	{
		assert(obj_list.size() > 0); // obj cannot be empty
		// sort the obj
		using begin_end_pair_t = std::pair<int, int>;
		std::queue<begin_end_pair_t> sort_queue;

		int axis = 0;
		auto cmp = [axis](Renderable* a, Renderable* b)
		{
			return a->aabb.get_center()(axis) < b->aabb.get_center()(axis);
		};
 		
		sort_queue.push(std::make_pair(0, obj_ptr_list.size()));
		while (!sort_queue.empty())
		{
			int n_segment = sort_queue.size();
			for (int seg = 0; seg < n_segment; ++seg)
			{
				// sort each segment
				begin_end_pair_t begend = sort_queue.front();
				sort_queue.pop();
				std::sort(obj_ptr_list.begin() + begend.first, obj_ptr_list.begin() + begend.second, cmp);

				int mid = (begend.first + begend.second) >> 1;
				if (mid > begend.first + 1)
					sort_queue.push(std::make_pair(begend.first, mid));
				if (mid + 1 < begend.second)
					sort_queue.push(std::make_pair(mid, begend.second));
			}

			axis += 1;
			axis %= 3;
		}

		// construct the tree
	
		for (int i = 0; i < obj_ptr_list.size(); ++i)
		{
			BVHNd* nd = new BVHNd;
			nd->aabb = obj_ptr_list[i]->aabb;
			nd->rd_obj = obj_ptr_list[i];
			tree_nodes.push_back(nd);
		}

		int st = 0, ed = obj_ptr_list.size() - 1;
		while (st < ed)
		{
			int new_nodes = 0;
			for (int i = st; i <= ed;i += 2)
			{
				++new_nodes;
				BVHNd* nd = new BVHNd;
				if (i + 1 <= ed)
				{
					nd->aabb = tree_nodes[i]->aabb.union_with_other(tree_nodes[i]->aabb);
					nd->lc = tree_nodes[i];
					nd->rc = tree_nodes[i+1];
					
				}
				else
				{
					nd->aabb = tree_nodes[i]->aabb;
					nd->lc = tree_nodes[i];
					
				}

				tree_nodes.push_back(nd);
			}
			st = ed + 1;
			ed = st + new_nodes - 1;
		}
		root = tree_nodes[tree_nodes.size() - 1];
		
	}
	
	~BVH()
	{
		for (auto i : tree_nodes)
		{
			delete i;
		}
	}

	
	IntersectInfo get_first_intersection(Ray& ray)
	{
		IntersectInfo res;
		res.do_intersect = false;
		res.distance = 1e20;
		std::queue<BVHNd*> traverse_queue;
		traverse_queue.push(root);

		while (!traverse_queue.empty())
		{
			auto top = traverse_queue.front();
			traverse_queue.pop();
			if (top->aabb.is_ray_inter(ray))
			{
				if (top->rd_obj) // is a leaf
				{
					auto info = top->rd_obj->try_intersect_ray(ray);
					if (info.first > 0 && info.first < res.distance)
					{
						IntersectInfo inter;
						inter.obj = top->rd_obj;
						inter.pos = info.second;
						inter.distance = info.first;
						res = inter;
					}
				}
				else // not a leaf
				{
					// push if not null
					if (top->lc) traverse_queue.push(top->lc);
					if (top->rc) traverse_queue.push(top->rc);
				}
			}
		}

		return res;
	}
};

#pragma once
#include "photon.hpp"
#include "Eigen/Dense"
#include <functional>
#include <queue>
#include <vector>
#include <stack>
struct KDTNd
{
	Photon *ph; // the photon
	int axis;
	KDTNd *p, * lc, * rc, *sibling;
	bool visited;

	KDTNd() :ph(nullptr), axis(0), p(nullptr), lc(nullptr), rc(nullptr), visited(false), sibling(nullptr) {};
};


using dist_kdtnd = std::pair<float, KDTNd*>;
using pq_kdt = std::priority_queue<dist_kdtnd, std::vector<dist_kdtnd>, std::function<bool(const dist_kdtnd &a, const dist_kdtnd &b)>>;

class KDT
{
	using v3f = Eigen::Vector3f;
public:
	KDTNd *root;
	std::vector<KDTNd*> ptrs;
	std::vector<KDTNd*> visited;

	KDT(std::vector<Photon> & ph_rec) : root(nullptr)
	{
		root = recursive_construction(0, ph_rec.size(), ph_rec, 0);
		recursive_set_p_sib(root);
	}

	static bool cmpx(Photon& a, Photon& b)
	{
		return a.ray.pos(0) < b.ray.pos(0);
	}
	static bool cmpy(Photon& a, Photon& b)
	{
		return a.ray.pos(1) < b.ray.pos(1);
	}

	static bool cmpz(Photon& a, Photon& b)
	{
		return a.ray.pos(2) < b.ray.pos(2);
	}


	KDTNd* recursive_construction(int st, int ed, std::vector<Photon>& photons, int axis)
	{
		if (st >= ed - 1)
		{
			KDTNd* nd = new KDTNd;
			nd->axis = axis;
			nd->ph = &photons[st];
			ptrs.push_back(nd);
			return nd;
		}

		// in this function, we won't set the parent at first
		/*std::sort(photons.begin() + st, photons.begin() + ed, [axis](Photon& a, Photon& b)
			{
				return a.ray.pos(axis) < b.ray.pos(axis);
			});*/
		switch (axis)
		{
		case 0:
			std::sort(photons.begin() + st, photons.begin() + ed, cmpx); break;
		case 1:
			std::sort(photons.begin() + st, photons.begin() + ed, cmpy); break;
		case 2:
			std::sort(photons.begin() + st, photons.begin() + ed, cmpz); break;
		}

		int mid = (st + ed) >> 1;
		KDTNd* nd = new KDTNd;
		nd->axis = axis;
		nd->ph = &photons[mid];
		if (mid - 1 >= st)
		{
			nd->lc = recursive_construction(st, mid, photons, (axis + 1) % 3);
		}
		if (mid + 1 < ed)
		{
			nd->rc = recursive_construction(mid + 1, ed, photons, (axis + 1) % 3);
		}
		ptrs.push_back(nd);
		return nd;
	}

	void recursive_set_p_sib(KDTNd * mother) // call this after the function above
	{
		if (mother->lc)
		{
			mother->lc->p = mother;
			mother->lc->sibling = mother->rc;
			recursive_set_p_sib(mother->lc);
		}
		if (mother->rc)// if null, nmsl
		{
			mother->rc->p = mother;
			mother->rc->sibling = mother->lc;
			recursive_set_p_sib(mother->rc);
		}
	}

	~KDT()
	{
		for (auto p : ptrs)
			delete p;
	}

	void recursive_kdt_search(pq_kdt& pq, int N, KDTNd* nd, v3f& pos)
	{
		if (!nd) return;
	
		float d = (nd->ph->ray.pos - pos).norm();
		nd->visited = true;
		if (pq.size() < N)
		{
			pq.push(std::make_pair(d, nd));
		}
		else if (d < pq.top().first)
		{
			pq.pop();
			pq.push(std::make_pair(d, nd));
		}

		if (pos(nd->axis) < nd->ph->ray.pos(nd->axis))
		{
			recursive_kdt_search(pq, N, nd->lc, pos);
			if (pq.size() < N
				|| std::abs(pos(nd->axis) - nd->ph->ray.pos(nd->axis)) < pq.top().first)
			{
				recursive_kdt_search(pq, N, nd->rc, pos);
			}
		}
		else
		{
			recursive_kdt_search(pq, N, nd->rc, pos);
			if (pq.size() < N
				|| std::abs(pos(nd->axis) - nd->ph->ray.pos(nd->axis)) < pq.top().first)
			{
				recursive_kdt_search(pq, N, nd->lc, pos);
			}
		}

	/*	if (pq.size() < N
			|| std::abs(pos(nd->axis) - nd->ph->ray.pos(nd->axis)) < pq.top().first)
		{
			if(nd->lc && nd->lc->visited)
				recursive_kdt_search(pq, N, nd->rc, pos);
			else if (nd->rc && nd->rc->visited)
				recursive_kdt_search(pq, N, nd->lc, pos);
		}*/
	
	}

	std::vector<Photon*> N_near(int N, v3f pos)
	{
		pq_kdt pri_que([](const dist_kdtnd& a, const dist_kdtnd& b) {return a.first < b.first; });
		recursive_kdt_search(pri_que, N, root, pos);
		int cnt = 0;
		std::vector<Photon*> pht(N);
		while (!pri_que.empty())
		{
			pht[cnt++] = pri_que.top().second->ph;
			pri_que.pop();
		}
		for (auto p : visited) p->visited = false;
		return pht;
	}
};

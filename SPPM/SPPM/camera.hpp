#pragma once
#include"Eigen/Dense"
#include "ray.hpp"
#include <vector>
#include "scene.hpp"
#include "renderable.hpp"
#include <queue>
#include "photon_map.hpp"
class Cam
{
public:
	using v3f = Eigen::Vector3f;
	const int N = 20;
	v3f top, lookat, pos;
	int w, h;
	float focus, fov;
	float ccd_w, ccd_h;
	int sub_x, sub_y;
	v3f local_x_ax;

	std::vector<v3f> radiance_rec;



	Cam(v3f p, v3f t, v3f l, int w, int h, float deg_fovY ,int subx, int suby,float focus = 1.0f) : 
		pos(p), top(t), lookat(l), focus(focus), w(w), h(h), fov(deg_fovY), sub_x(subx), sub_y(suby)
	{
		float asp = float(w) / float(h);
		float rad_half_fov = deg_fovY / 2 * EIGEN_PI;
		
		ccd_h = focus * std::tan(rad_half_fov) * 2;
		ccd_w = ccd_h * asp;
		local_x_ax = lookat.cross(top).normalized();
		radiance_rec.resize(w * h);
	}

	Ray sample_ray_xy(int x, int y, int idx, int idy)
	{
		if (x >= w || y >= h || x < 0 || y < 0) return Ray();

		v3f center(ccd_w / 2, ccd_h / 2, -focus);

		float ccdx = (float(x) + float(idx) / sub_x) / w * ccd_w;
		float ccdy = (float(y) + float(idy) / sub_y) / h * ccd_h;

		v3f ccd(ccdx, ccdy, 0);
		v3f vec = ccd - center;
		
		v3f world_dir = (vec(0) * local_x_ax + vec(1) * (-top) + vec(2) * lookat).normalized();
		return Ray(pos, world_dir);
	}
	
	
};
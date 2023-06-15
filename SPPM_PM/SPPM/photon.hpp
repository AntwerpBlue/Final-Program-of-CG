#pragma once
#include "Eigen/Dense"
#include "ray.hpp"


class Photon
{
public:
	using v3f = Eigen::Vector3f;
	v3f power_rgb;
	Ray ray;
	v3f end_at;
	bool bounce;
	bool caustic_available;
	Photon() :power_rgb(v3f::Zero()), ray(Ray()), end_at(v3f::Zero()), bounce(false), caustic_available(false) {}
	Photon(v3f power, v3f pos, v3f dir) : power_rgb(power), ray(Ray(pos, dir)), end_at(v3f::Zero()), bounce(false), caustic_available(false) {}
};
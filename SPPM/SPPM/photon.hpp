#pragma once
#include "Eigen/Dense"
#include "ray.hpp"


class Photon
{
public:
	using v3f = Eigen::Vector3f;
	v3f power_rgb;
	Ray ray;

	Photon() :power_rgb(v3f::Zero()), ray(Ray()) {}
	Photon(v3f power, v3f pos, v3f dir) : power_rgb(power), ray(Ray(pos, dir)) {}
};
#pragma once
#include "Eigen/Dense"

class Ray
{
	using v3f = Eigen::Vector3f;
public:
	v3f pos;
	v3f dir;
	Ray() :pos(v3f::Zero()), dir(v3f(1, 0, 0)) {}
	Ray(v3f p, v3f d) : pos(p), dir(d) {} // pass by value, not ref

	v3f get_t(float t) { return pos + dir * t; }
};

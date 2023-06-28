#pragma once
#include <string>
#include <unordered_map>
#include <Eigen/Dense>
#include <sstream>
#include <fstream>
#include <iostream>
class SppmConf
{
	using v3f = Eigen::Vector3f;
public:
	int rast_w, rast_h;
	std::string model_filename;
	v3f cam_pos, cam_lookat, cam_top;
	float fovY;
	int single_samples;
	int sample_bounces;
	int iter_times;
	int photon_emit;
	int photon_bounce;
	int first_search_N;
	float first_radius;

	SppmConf() :rast_w(400), rast_h(400),
		model_filename("bocchi.models"),
		cam_pos(v3f(0, 0, 0)),
		cam_top(v3f(0, 0, 0)),
		cam_lookat(v3f(0, 0, 0)),
		fovY(90),
		single_samples(1),
		sample_bounces(1),
		iter_times(1),
		photon_emit(10000),
		photon_bounce(1),
		first_search_N(100),
		first_radius(1)
	{}

	void parse(std::string& filename)
	{
		std::ifstream conffile(filename);
		std::string line;
		while (std::getline(conffile, line))
		{
			line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
			if (line[0] == '#' || line.empty()) continue;
			auto delimiterPos = line.find("=");
			auto name = line.substr(0, delimiterPos);
			auto value = line.substr(delimiterPos + 1);
			if (name == "w")
				rast_w = std::stoi(value);
			else if (name == "h")
				rast_h = std::stoi(value);
			else if (name == "model_filename")
				model_filename = value;
			else if (name == "cam_pos_x")
				cam_pos(0) = std::stof(value);
			else if (name == "cam_pos_y")
				cam_pos(1) = std::stof(value);
			else if (name == "cam_pos_z")
				cam_pos(2) = std::stof(value);
			else if (name == "cam_top_x")
				cam_top(0) = std::stof(value);
			else if (name == "cam_top_y")
				cam_top(1) = std::stof(value);
			else if (name == "cam_top_z")
				cam_top(2) = std::stof(value);
			else if (name == "cam_lookat_x")
				cam_lookat(0) = std::stof(value);
			else if (name == "cam_lookat_y")
				cam_lookat(1) = std::stof(value);
			else if (name == "cam_lookat_z")
				cam_lookat(2) = std::stof(value);
			else if (name == "fovY")
				fovY = std::stof(value);
			else if (name == "single_samples")
				single_samples = std::stoi(value);
			else if (name == "sample_bounces")
				sample_bounces = std::stoi(value);
			else if (name == "iter_times")
				iter_times = std::stoi(value);
			else if (name == "photon_emit")
				photon_emit = std::stoi(value);
			else if (name == "photon_bounce")
				photon_bounce = std::stoi(value);
			else if (name == "first_search_N")
				first_search_N = std::stoi(value);
			else if(name == "first_radius")
				first_radius = std::stof(value);
		}
	}

	void print_conf()
	{
		using namespace std;
		cout << "Configuration:\n";
		cout << "Filename:" << model_filename << endl;
		cout << "Width:" << rast_w << "\tHeight" << rast_h << endl;
		cout << "Camera:\n Pos:\n" << cam_pos << endl << "Top:\n" << cam_top << endl << "LookAt:\n" << cam_lookat << endl;
		cout << "FovY" << fovY << endl;
		cout << "single samples" << single_samples << endl;
		cout << "sample bounces" << sample_bounces<<endl;
		cout << "iter times" << iter_times<<endl;
		cout << "photon emit" << photon_emit << endl;
		cout << "photon bounces" << photon_bounce<<endl;
		cout << "first search N" << first_search_N << endl;
		cout << "first radius" << first_radius << endl;
	}
};
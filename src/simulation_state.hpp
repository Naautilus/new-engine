#pragma once
#include "../constants/constants.hpp"
#include "../renderer/color.hpp"


struct simulation_state {
	//std::vector<physics_object::object> physics_objects;
	std::function<double(double, double)> ground_function;
	color sky_color;
	double gravity;
	double air_density;
	double timescale;
	simulation_state();
	simulation_state(
		std::function<double(double, double)> ground_function_,
		color sky_color_,
		double gravity_,
		double air_density_,
		double timescale_
	);
};
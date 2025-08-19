#pragma once
#include "../constants/constants.hpp"
//#include "../ground/ground_logic.hpp"

struct simulation_state {
	//std::vector<physics_object::object> physics_objects;
	std::function<double(double, double)> ground_function;
	uint8_t sky_r, sky_g, sky_b;
	double gravity;
	double air_density;
	double timescale;
	simulation_state();
	simulation_state(
		std::function<double(double, double)> ground_function_,
		uint8_t sky_r_,
		uint8_t sky_g_,
		uint8_t sky_b_,
		double gravity_,
		double air_density_,
		double timescale_
	);
};
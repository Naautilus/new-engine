#pragma once
#include "../constants/constants.cpp"
#include "../ground/ground_logic.cpp"

struct simulation_state {
	std::vector<physics_object::object> physics_objects;
	std::function<double(double, double)> ground_function;
	uint8_t sky_r, sky_g, sky_b;
	double gravity;
	double air_density;
	double timescale;
	simulation_state() {
		ground_function = ground::get_ground_altitude;
		sky_r = 72;
		sky_g = 116;
		sky_b = 163;
		gravity = 1;
		air_density = 1.293;
		timescale = 1;
	}
	simulation_state(
		std::function<double(double, double)> ground_function_,
		uint8_t sky_r_,
		uint8_t sky_g_,
		uint8_t sky_b_,
		double gravity_,
		double air_density_,
		double timescale_
	) {
		ground_function = ground_function_;
		sky_r = sky_r_;
		sky_g = sky_g_;
		sky_b = sky_b_;
		gravity = gravity_;
		air_density = air_density_;
		timescale = timescale_;
	}
};
#pragma once
#include "../constants/constants.hpp"

struct pid {
	double gain_p;
	double gain_i;
	double gain_d;
	double authority_limit;
	double last_input = 0;
	double output = 0;
	std::vector<double> previous_input_buffer;
	int previous_input_buffer_position = 0;
	pid(double p, double i, double d, double authority_limit_);
	void update(double input);
};
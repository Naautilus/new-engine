#pragma once
#include "../constants/constants.cpp"

struct pid {
	double gain_p;
	double gain_i;
	double gain_d;
	double authority_limit;
	double last_input = 0;
	double output = 0;
	std::vector<double> previous_input_buffer;
	int previous_input_buffer_position = 0;
	pid(double p, double i, double d, double authority_limit_) {
		gain_p = p;
		gain_i = i;
		gain_d = d;
		authority_limit = authority_limit_;
		previous_input_buffer = std::vector<double>(i / constants::DELTA_T, 0);
	}
	void update(double input) {

		double input_integral = 0;
		if (gain_i != 0) {
			input_integral = constants::DELTA_T * std::accumulate(previous_input_buffer.begin(), previous_input_buffer.end(), 0.0);
			previous_input_buffer[previous_input_buffer_position] = input;
			previous_input_buffer_position++;
			previous_input_buffer_position %= previous_input_buffer.size();
		}
		double input_derivative = (1/constants::DELTA_T) * (input - last_input);
		last_input = input;

		output = 0;
		output += gain_p * input;
		output += gain_i * input_integral;
		output += gain_d * input_derivative;

		output = fmax(output, -authority_limit);
		output = fmin(output,  authority_limit);

	}
};
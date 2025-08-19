#include "pid.hpp"

pid::pid(double p, double i, double d, double authority_limit_) {
    gain_p = p;
    gain_i = i;
    gain_d = d;
    authority_limit = authority_limit_;
    previous_input_buffer = std::vector<double>(i / constants::DELTA_T, 0);
}
void pid::update(double input) {

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
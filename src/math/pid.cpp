#include "pid.hpp"

pid::pid() {
    gain_p = 0;
    gain_i = 0;
    gain_d = 0;
    authority_limit = 0;
}
pid::pid(double p, double time_i, double time_d, double authority_limit_) {
    gain_p = p;
    gain_i = p * time_i;
    gain_d = p * time_d;
    authority_limit = authority_limit_;
    previous_input_buffer = std::vector<double>(time_i / constants::DELTA_T, 0);
}
void pid::update(double input) {

    double input_integral = 0;
    if (gain_i != 0) {
        input_integral = constants::DELTA_T * std::accumulate(previous_input_buffer.begin(), previous_input_buffer.end(), 0.0);
        input_integral /= previous_input_buffer.size();
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
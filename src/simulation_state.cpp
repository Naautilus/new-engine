// top of cpp marker
#include "simulation_state.hpp"
#include "../ground/ground_logic.hpp"

simulation_state::simulation_state() {
    ground_function = ground::get_ground_altitude;
    sky_color = color{0.28, 0.7, 1.2};
    gravity = 9.81;
    air_density = 1.293;
    timescale = 1;
}
simulation_state::simulation_state(
    std::function<double(double, double)> ground_function_,
    color sky_color_,
    double gravity_,
    double air_density_,
    double timescale_
) {
    ground_function = ground_function_;
    sky_color = sky_color_;
    gravity = gravity_;
    air_density = air_density_;
    timescale = timescale_;
}
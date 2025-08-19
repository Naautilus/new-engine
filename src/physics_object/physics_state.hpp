#pragma once
#include "control_bindings.hpp"
#include "../vector/vector_spaces.hpp"
#include "../constants/constants.hpp"

struct physics_state {
    const double mass;
    const double health;
    const double base_signal_strength;
	const vector::worldspace position;
	const vector::worldspace velocity;
	const Eigen::Quaterniond rotation;
	const vector::worldspace angular_velocity;
	const vector::localspace rotational_inertia;
	const vector::localspace recorded_acceleration;
	vector::worldspace previous_velocity;
    vector::worldspace velocity_at_point(vector::worldspace position_absolute);
};
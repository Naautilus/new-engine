#pragma once
#include "control_bindings.hpp"
#include "../vector/vector_spaces.hpp"
#include "../constants/constants.hpp"

struct physics_state {
    double mass = 1.0;
    double health = 1.0;
    double base_signal_strength = 1.0;
	vector::worldspace position = vector::worldspace(0,0,0);
	vector::worldspace velocity = vector::worldspace(0,0,0);
	Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
	vector::worldspace angular_velocity = vector::worldspace(0,0,0);
	vector::localspace rotational_inertia = vector::localspace(1,1,1);
	vector::localspace recorded_acceleration = vector::worldspace(0,0,0);
	vector::worldspace previous_velocity;
    vector::worldspace velocity_at_point(vector::worldspace position_absolute);
};
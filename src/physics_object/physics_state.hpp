#pragma once
#include "control_bindings.cpp"

struct physics_state {
	double mass;
	double health;
	double base_signal_strength;
	vector::worldspace position;
	vector::worldspace velocity;
	Eigen::Quaterniond rotation;
	vector::worldspace angular_velocity;
	vector::localspace rotational_inertia;
	vector::localspace recorded_acceleration;
	vector::worldspace previous_velocity;
    vector::worldspace velocity_at_point(vector::worldspace position_absolute);
};
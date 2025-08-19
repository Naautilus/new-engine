#pragma once
#include "linmath.h"

struct camera_properties {
    double fov;
	vec3 camera_position;
	vec3 camera_y_direction;
	vec3 camera_z_direction;
	std::string camera_target_name;
	bool camera_target_search_direction;
	vector::localspace camera_target_offset;
	vector::worldspace last_camera_position;
	vector::worldspace last_camera_target_velocity;
	std::vector<Eigen::Quaterniond> previous_camera_rotations;
	camera_properties(std::string camera_target_name_, bool camera_target_search_direction_, vector::localspace camera_target_offset_);
    enum track_mode;
    void update(Eigen::Quaterniond rotation, vector::worldspace position, vector::worldspace velocity, track_mode track_mode_);
};
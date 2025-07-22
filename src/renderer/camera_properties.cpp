#pragma once
#include "linmath.h"

struct camera_properties {
	vec3 camera_position = {0, 1000, 0};
	vec3 camera_y_direction = {0, 1, 0};
	vec3 camera_z_direction = {0, 0, 1};
	std::string camera_target_name;
	bool camera_target_search_direction;
	vector::localspace camera_target_offset;
	vector::worldspace last_camera_position;
	vector::worldspace last_camera_target_velocity;
	std::vector<Eigen::Quaterniond> previous_camera_rotations;
	camera_properties(std::string camera_target_name_, bool camera_target_search_direction_, vector::localspace camera_target_offset_) {
		camera_target_name = camera_target_name_;
		camera_target_search_direction = camera_target_search_direction_;
		camera_target_offset = camera_target_offset_;
	}
};
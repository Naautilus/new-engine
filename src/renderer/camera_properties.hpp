#pragma once
#include "linmath.h"
#include "../constants/constants.hpp"
#include "../vector/vector_spaces.hpp"

struct camera_properties {
    double fov = 90;
    vec3 camera_position = {0, 1000, 0};
    vec3 camera_y_direction = {0, 1, 0};
    vec3 camera_z_direction = {0, 0, 1};
	std::string camera_target_name;
	bool camera_target_search_direction;
	vector::localspace camera_target_offset;
	vector::worldspace last_camera_position;
	vector::worldspace last_camera_target_velocity;
	std::vector<Eigen::Quaterniond> previous_camera_rotations;
	camera_properties(std::string camera_target_name_, bool camera_target_search_direction_, vector::localspace camera_target_offset_);
    enum track_mode {
        TRACKING,
        NOT_TRACKING
    };
    void update(Eigen::Quaterniond rotation, vector::worldspace position, vector::worldspace velocity, track_mode track_mode_ = TRACKING);
};
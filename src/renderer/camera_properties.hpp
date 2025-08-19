#pragma once
#include "linmath.h"
#include "../constants/constants.hpp"
#include "../vector/vector_spaces.hpp"

struct camera_properties {
    extern double fov;
	extern vec3 camera_position;
	extern vec3 camera_y_direction;
	extern vec3 camera_z_direction;
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
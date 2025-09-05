#pragma once
#include "module.hpp"
#include "../ground/ground_logic.hpp"
#include "../math/pid.hpp"

namespace module {

struct missile_avionics : module {
    missile_avionics(double p, double i, double d, Eigen::Quaterniond rotation_, vector::localspace position_);
    double record_target_distance = std::numeric_limits<double>::max();
    pid pid_pitch;
    pid pid_yaw;
    pid pid_roll; // fed angular velocity, so P is D really
    double time_since_launch = 0;
    vector::worldspace get_worldspace_position(physics_object::object* parent);
    void update(physics_object::object* parent) override;
    double get_g_limit_fraction(double current_acceleration, double g_limit_min, double g_limit_max);
    vector::worldspace limit_g_forces(vector::worldspace unlimited_inputs, vector::localspace limited_inputs, double current_acceleration, double g_limit_min, double g_limit_max);
    double get_time_to_impact_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, vector::worldspace detection_velocity, physics_object::object* parent);
    double find_smallest_positive_root(double a, double b, double c);
    vector::localspace get_guidance_direct(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain);
    vector::localspace get_guidance_target_velocity(vector::worldspace current_detection_relative_worldspace, vector::worldspace detection_velocity, physics_object::object* parent, double gain);
    vector::localspace get_guidance_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, vector::worldspace detection_velocity, physics_object::object* parent, double gain, double speed_adjustment);
    vector::localspace get_guidance_proportional_navigation(vector::worldspace current_detection_relative_worldspace, vector::worldspace detection_velocity, physics_object::object* parent, double gain);
};

}

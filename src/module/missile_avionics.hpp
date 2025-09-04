#pragma once
#include "module.hpp"
#include "../ground/ground_logic.hpp"
#include "../math/pid.hpp"

// x = forward distance, y and z = perspectivified
//struct vector::scopespace : vector::localspace {};

namespace module {

struct flight_computer : module {
    double record_target_distance = 1e10;
    pid pid_pitch = pid(    1,    0,  0.1,  0.5);
    pid pid_yaw   = pid(    1,    0,  0.1,  0.5);
    pid pid_roll  = pid(    1,    0,  0.1,  1.0); // fed angular velocity, so P is D really
    double time_since_launch = 0;
    vector::worldspace get_worldspace_position(physics_object::object* parent);
    void update(physics_object::object* parent) override;
    double get_g_limit_fraction(double current_acceleration, double g_limit_min, double g_limit_max);
    vector::worldspace limit_g_forces(vector::worldspace unlimited_inputs, vector::localspace limited_inputs, double current_acceleration, double g_limit_min, double g_limit_max);
    vector::worldspace get_enemy_velocity(vector::worldspace current_detection_worldspace, vector::worldspace last_detection_worldspace);
    double get_time_to_impact_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent);
    double find_smallest_positive_root(double a, double b, double c);
    void update(physics_object::object* parent) override;
    vector::localspace get_guidance_direct(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain);
    vector::localspace get_guidance_target_velocity(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain);
    vector::localspace get_guidance_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain);
    vector::localspace get_guidance_proportional_navigation(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain);
}

}

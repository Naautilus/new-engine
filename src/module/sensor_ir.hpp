#pragma once
#include "module.hpp"
#include "../ground/ground_logic.hpp"
#include "../math/pid.hpp"

// x = forward distance, y and z = perspectivified
//struct vector::scopespace : vector::localspace {};

namespace module {

struct signal_point {
    vector::scopespace position_scopespace;
    double signal_strength;
    double distance_weight = 0;
    signal_point();
    signal_point(vector::worldspace target_position, vector::worldspace sensor_position, Eigen::Quaterniond rotation, double base_signal_strength);
    signal_point(double distance_, double scope_x_, double scope_y_, double signal_strength_);
    std::string str();
};

struct sensor_point : Eigen::Vector2i {
    using Eigen::Vector2i::Vector2i;
};

struct sensor_cell_grid {
    private:
    int size;
    double min;
    double max;

    /* 
    what indices of the cell grid would be contained within a
    circle of a given radius positioned at (0, 0) (has negatives)
    */
    std::unordered_map<double, std::vector<sensor_point>> circle_radius_indices_offsets;

    public:
    std::vector<std::vector<signal_point>> points;
    sensor_cell_grid(int size_, double view_cone_halfarc);
    void increase_signals_in_circle(double center_x, double center_y, double radius, double distance, double signal);
    std::vector<sensor_point> remove_invalid_indices(std::vector<sensor_point> indices);
    void calculate_indices_in_circle(double radius);
    std::vector<sensor_point> get_points_in_circle(double center_x, double center_y, double radius);
    sensor_point point_from_coordinates(double x, double y);
    int index_from_coordinates(double x, double y);
    double get_value_from_axis_index(int index);
    signal_point get_largest_signal();
    double distance_between_grid_cells();
    void print(vector::scopespace target, vector::scopespace center);
};

struct sensor_ir : module {
    double gimbal_cone_halfarc;
    double view_cone_halfarc;
    double target_recognition_cone_halfarc;
    vector::worldspace last_detection_worldspace;
    vector::scopespace last_detection_scopespace;
    double last_detection_signal_strength;
    double record_target_distance = 1e10;
    pid pid_pitch = pid(    1,    0,  0.1,  0.5);
    pid pid_yaw   = pid(    1,    0,  0.1,  0.5);
    pid pid_roll  = pid(    1,    0,  0.1,  1.0); // fed angular velocity, so P is D really
    double time_since_launch = 0;
    sensor_ir(double gimbal_cone_halfarc_, double view_cone_halfarc_, double target_recognition_cone_halfarc_, Eigen::Quaterniond rotation_, vector::localspace position_, double length, double width, double health_);
    vector::worldspace get_worldspace_position(physics_object::object* parent);
    void update(physics_object::object* parent) override;
    double get_g_limit_fraction(double current_acceleration, double g_limit_min, double g_limit_max);
    vector::worldspace limit_g_forces(vector::worldspace unlimited_inputs, vector::localspace limited_inputs, double current_acceleration, double g_limit_min, double g_limit_max);
    vector::worldspace get_enemy_velocity(vector::worldspace current_detection_worldspace, vector::worldspace last_detection_worldspace);
    double get_time_to_impact_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent);
    double find_smallest_positive_root(double a, double b, double c);
    vector::localspace get_guidance_direct(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain);
    vector::localspace get_guidance_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain);
    vector::localspace get_guidance_proportional_navigation(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain);
    std::vector<signal_point> get_signals_from_physics_objects(physics_object::object* parent);
    vector::scopespace get_target_direction(physics_object::object* parent);
    vector::worldspace get_target_position(physics_object::object* parent);
};

}

#pragma once
#include "shapes.cpp"
#include "collider_type.h"
#include "collision_data.cpp"
#include "../eigen_pca/eigen-pca.hpp"
#include "../renderer/mesh.h"

namespace collision {

struct collider {
    vector::worldspace position, velocity;
    Eigen::Quaterniond rotation;
    collider_type type;
    std::vector<triangle> model_data_unrotated;
    std::vector<triangle> model_data;
    vector::worldspace bounding_box_max;
    vector::worldspace bounding_box_min;
    double bounding_box_width_squared;
    private:
    bool check_collision_point_to_point(collider& c);
    bool check_collision_point_to_model(collider& c);
    bool check_collision_bounding_boxes(collider& c);
    bool _check_collision_model_to_model_triangle(collider& c);
    bool _check_collision_model_to_model_prism(collider& c);
    bool check_collision_model_to_model(collider& c);
    vector::worldspace get_collision_position_point_to_model(collider& c);
    std::optional<vector::worldspace> get_collision_normal_point_to_model(collider& c);
    std::vector<line_segment> _get_line_segments_of_intersection_model_to_model(collider& c);
    std::optional<vector::worldspace> get_collision_position_model_to_model(collider& c, std::vector<line_segment> intersections);
    std::optional<vector::worldspace> get_collision_normal_model_to_model(collider& c, std::vector<line_segment> intersections);
    void rotate_model_data(vector::worldspace position_, Eigen::Quaterniond rotation_);
    void set_bounding_box();
    public:
    collider();
    collider(std::vector<triangle> model_data_unrotated_);
    collider(mesh m);
    void update(vector::worldspace position_, vector::worldspace velocity_, Eigen::Quaterniond rotation_);
    bool check_collision(collider& c);
    std::optional<collision_data> get_collision_data(collider& c);
};

}
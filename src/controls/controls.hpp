#pragma once
#include "../constants/constants.hpp"

namespace controls {
  	
enum axis {
    roll, // rotation X
    pitch, // rotation Y
    yaw, // rotation Z
    engine1,
    gun1
};
enum response_type {
    instant,
    trim_resetting,
    trim_not_resetting
};
enum mouse_axis {
    MOUSE_X,
    MOUSE_Y
};
struct key {
    int key_number;
    double axis_response;
    key(int key_number_, double axis_response_);
};
struct mouse_position_input {
    mouse_axis mouse_axis_;
    double axis_response;
    mouse_position_input(mouse_axis mouse_axis__, double axis_response_);
};
struct input {
    axis axis_;
    response_type response_type_;
    double minimum, maximum, response_unmultiplied, response_multiplied, inherent_multiplier;
    std::vector<key> key_inputs;
    std::vector<mouse_position_input> mouse_position_inputs;
    input(axis axis__, response_type response_type__, double minimum_, double maximum_, double inherent_multiplier_);
    void add_key(int key_number, double axis_response);
    void add_mouse_position_input(mouse_axis mouse_axis__, double axis_response);
    double get_control_multiplier_for_health_fraction(double health_fraction);
};

}
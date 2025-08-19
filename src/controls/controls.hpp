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
struct key {
    int key_number;
    double axis_response;
    key(int key_number_, double axis_response_);
};
struct input {
    axis axis_;
    response_type response_type_;
    double minimum;
    double maximum;
    double inherent_multiplier;
    double response_unmultiplied;
    double response_multiplied;
    input(axis axis__, response_type response_type__, double minimum_, double maximum_, double inherent_multiplier_);
    void add_key(int key_number, double axis_response);
    double get_control_multiplier_for_health_fraction(double health_fraction);
};

}
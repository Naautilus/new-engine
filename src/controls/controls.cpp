#pragma once
#include "controls.hpp"

namespace controls {
    key::key(int key_number_, double axis_response_) {
        key_number = key_number_;
        axis_response = axis_response_;
    }
    input::input(axis axis__, response_type response_type__, double minimum_, double maximum_, double inherent_multiplier_) {
        axis_ = axis__;
        response_type_ = response_type__;
        minimum = minimum_;
        maximum = maximum_;
        inherent_multiplier = inherent_multiplier_;
        response_unmultiplied = 0;
        response_multiplied = 0;
    }
    void input::add_key(int key_number, double axis_response) {
        key_inputs.push_back(key(key_number, axis_response));
    }
    double input::get_control_multiplier_for_health_fraction(double health_fraction) {
        return fmax(0.0, (health_fraction-constants::UNCONTROLLABLE_HEALTH_FRACTION)*(1/(1-constants::UNCONTROLLABLE_HEALTH_FRACTION)));
    }
}
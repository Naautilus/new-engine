#pragma once
#include "../constants/constants.cpp"

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
  	  	key(int key_number_, double axis_response_) {
  	  	  	key_number = key_number_;
  	  	  	axis_response = axis_response_;
  	  	}
  	};
  	struct input {
  	  	axis axis_;
  	  	response_type response_type_;
  	  	double minimum, maximum, response_unmultiplied, response_multiplied, inherent_multiplier;
  	  	std::vector<key> key_inputs;
  	  	input(axis axis__, response_type response_type__, double minimum_, double maximum_, double inherent_multiplier_) {
  	  	  	axis_ = axis__;
  	  	  	response_type_ = response_type__;
  	  	  	minimum = minimum_;
  	  	  	maximum = maximum_;
  	  	  	inherent_multiplier = inherent_multiplier_;
  	  	  	response_unmultiplied = 0;
  	  	  	response_multiplied = 0;
  	  	}
  	  	void add_key(int key_number, double axis_response) {
  	  	  	key_inputs.push_back(key(key_number, axis_response));
  	  	}
  	  	double get_control_multiplier_for_health_fraction(double health_fraction) {
  	  	  	return fmax(0.0, (health_fraction-constants::UNCONTROLLABLE_HEALTH_FRACTION)*(1/(1-constants::UNCONTROLLABLE_HEALTH_FRACTION)));
  	  	}
  	};
}
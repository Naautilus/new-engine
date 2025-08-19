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
struct key;
struct input;

}
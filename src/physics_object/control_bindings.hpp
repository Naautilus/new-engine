#pragma once
#include "../math/pid.hpp"
#include "../controls/controls.hpp"

struct control_bindings {
    std::vector<controls::input> inputs;
    double get_response(controls::axis axis__);
    controls::input* get_input_for_axis(controls::axis axis__);
};
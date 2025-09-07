#pragma once
#include "../math/pid.hpp"
#include "../controls/controls.hpp"

struct control_bindings {
    std::vector<controls::input> inputs;
    double get_response(controls::axis axis__, controls::input_destination destination_);
    controls::input* get_first_input(controls::axis axis__, controls::input_destination destination_);
    void add_bindings(control_bindings other);
};
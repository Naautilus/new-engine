#pragma once
#include "../math/pid.cpp"
#include "../controls/controls.cpp"

struct control_bindings {
    std::vector<controls::input> inputs;
    controls::input* get_input(controls::axis axis__);
};
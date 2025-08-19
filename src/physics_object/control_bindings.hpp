#pragma once
#include "../math/pid.hpp"
#include "../controls/controls.hpp"

struct control_bindings {
    std::vector<controls::input> inputs;
    controls::input* get_input(controls::axis axis__);
};
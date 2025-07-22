#pragma once
#include "../math/pid.cpp"
#include "../controls/controls.cpp"

struct control_bindings {
    std::vector<controls::input> inputs;
    controls::input* get_input(controls::axis axis__) {
        for (int i = 0; i < inputs.size(); i++) {
            if (inputs[i].axis_ == axis__) return &inputs[i];
        }
        return nullptr;
    }
};
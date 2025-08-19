#pragma once
#include "../math/pid.cpp"
#include "../controls/controls.cpp"

controls::input* control_bindings::get_input(controls::axis axis__) {
    for (int i = 0; i < inputs.size(); i++) {
        if (inputs[i].axis_ == axis__) return &inputs[i];
    }
    return nullptr;
}
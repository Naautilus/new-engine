#include "control_bindings.hpp"

controls::input* control_bindings::get_input(controls::axis axis__) {
    for (int i = 0; i < inputs.size(); i++) {
        if (inputs[i].axis_ == axis__) return &inputs[i];
    }
    return nullptr;
}
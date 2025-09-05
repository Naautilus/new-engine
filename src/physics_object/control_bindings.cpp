#include "control_bindings.hpp"

double control_bindings::get_response(controls::axis axis__) {
    double response = 0;
    for (int i = 0; i < inputs.size(); i++) {
        if (inputs[i].axis_ == axis__) response += inputs[i].response_multiplied;
    }
    return response;
}

controls::input* control_bindings::get_input_for_axis(controls::axis axis__) {
    for (int i = 0; i < inputs.size(); i++) {
        if (inputs[i].axis_ == axis__) return &inputs[i];
    }
    return nullptr;
}
#include "control_bindings.hpp"

double control_bindings::get_response(controls::axis axis__, controls::input_destination destination_) {
    double response = 0;
    for (int i = 0; i < inputs.size(); i++) {
        if (inputs[i].axis_ == axis__ && inputs[i].destination == destination_) response += inputs[i].response_multiplied;
    }
    //std::cout << "response for axis " << axis__ << " and destination " << destination_ << ": " << response << "\n";
    return response;
}

controls::input* control_bindings::get_first_input(controls::axis axis__, controls::input_destination destination_) {
    for (int i = 0; i < inputs.size(); i++) {
        if (inputs[i].axis_ == axis__ && inputs[i].destination == destination_) return &inputs[i];
    }
    return nullptr;
}

void control_bindings::add_bindings(control_bindings other) {
    for (controls::input input_ : other.inputs) {
        inputs.push_back(input_);
    }
}
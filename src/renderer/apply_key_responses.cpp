#include "apply_key_responses.hpp"
#include "../physics_object/object.hpp"

namespace {

double _get_response_for_resetting_trim(double renderer_dt, controls::input& c, double response_) {
    int search_direction = c.response_unmultiplied > 0 ? 1 : -1;

    double best_key_candidate_response_speed = 0;
    for (controls::key& k : c.key_inputs) {
        if (k.axis_response * search_direction > best_key_candidate_response_speed) {
            best_key_candidate_response_speed = k.axis_response * search_direction;
        }
    }

    best_key_candidate_response_speed *= renderer_dt;

    double output = c.response_unmultiplied;
    output += std::clamp(-c.response_unmultiplied, -best_key_candidate_response_speed, best_key_candidate_response_speed);
    return output;
}

double _get_response_for_response_type(double renderer_dt, controls::input& c, double response_) {
    if (c.response_type_ == controls::instant) return response_;
    if (c.response_type_ == controls::trim_not_resetting) return c.response_unmultiplied + response_ * renderer_dt;
    if (c.response_type_ == controls::trim_resetting && response_ != 0) return c.response_unmultiplied + response_ * renderer_dt;
    return _get_response_for_resetting_trim(renderer_dt, c, response_);
}

void _process_control_input(GLFWwindow* window, double renderer_dt, controls::input& c, physics_object::object& o) {
    if (c.key_inputs.size() == 0) return;

    double response_ = 0;
    for (controls::key k : c.key_inputs) {
        if (renderer::key_pressed(window, k.key_number)) response_ += k.axis_response;
    }

    c.response_unmultiplied = _get_response_for_response_type(renderer_dt, c, response_);
    c.response_unmultiplied = std::clamp(c.response_unmultiplied, c.minimum, c.maximum);
    c.response_multiplied = c.response_unmultiplied;
    c.response_multiplied *= c.inherent_multiplier;
    c.response_multiplied *= c.get_control_multiplier_for_health_fraction(o.physics_state.health / o.physics_state.mass);
}

}

/*
NOTE: it could be useful to track this in the physics part of the engine and apply responses there for better between-frame scaling
*/
void renderer::apply_key_responses(GLFWwindow* window, double renderer_dt) {
    if (globals::paused) return;

    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();

    if (!glfwGetWindowAttrib(window, GLFW_FOCUSED)) return;
    for (int i = 0; i < physics_objects_.size(); i++) {
        auto& o = physics_objects_[i];
        if (!o) return;
        if (o->mutex) o->mutex->lock();
        for (controls::input& c : o->control_bindings.inputs) {
            _process_control_input(window, renderer_dt, c, *o);
        }
        if (o->mutex) o->mutex->unlock();
    }
}
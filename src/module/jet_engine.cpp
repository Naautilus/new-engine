#pragma once
#include "jet_engine.hpp"

namespace module {
    
jet_engine::jet_engine(double max_thrust_, double minimum_throttle_, double maximum_throttle_, vector::localspace thrust_direction_, vector::localspace position_, double length_, double width, double health_) {
    max_thrust = max_thrust_;
    minimum_throttle = minimum_throttle_;
    maximum_throttle = maximum_throttle_;
    thrust_direction = thrust_direction_;
    position = position_;
    length = length_;
    std::vector<collision::triangle> model = collision::generate_cylinder(length, width);
    collider = collision::collider(model);
    health = health_;
}
void jet_engine::update(physics_object::object* parent) override {
    if (health <= 0) return;
    controls::input* throttle = parent->control_bindings.get_input(controls::engine1);
    double response = 0;
    if (throttle) response = throttle->response_multiplied;
    response = fmax(response, minimum_throttle);
    response = fmin(response, maximum_throttle);
    double thrust = response * max_thrust;
    parent->queue_force(position, thrust * thrust_direction);
    models.clear();
    double plume_size = cbrt(thrust / 1e4);
    double plume_size_multiplier_normalized = random(0, 1);
    double plume_size_multiplier = 0.5 + 1 * plume_size_multiplier_normalized;
    plume_size *= plume_size_multiplier;
    models.push_back(visual_model(models::flame_trail, position + vector::localspace(-0.5*length, 0, 0), vector::localspace(plume_size, plume_size, plume_size)));
    parent->physics_state.base_signal_strength = 0.9*thrust + 0.1*max_thrust;
}

}

void physics_object::object::add_jet_engine(module::jet_engine j) {
    std::shared_ptr<module::module> m = std::make_shared<module::jet_engine>(j);
    properties.modules.push_back(m);
}
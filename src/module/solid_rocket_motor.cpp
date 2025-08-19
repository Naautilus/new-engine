#include "solid_rocket_motor.hpp"

namespace module {
    
solid_rocket_motor::solid_rocket_motor(double thrust_, double burntime_seconds_remaining_, vector::localspace thrust_direction_, vector::localspace position_, double length, double width, double health_) {
    thrust = thrust_;
    burntime_seconds_remaining = burntime_seconds_remaining_;
    thrust_direction = thrust_direction_;
    position = position_;
    std::vector<collision::triangle> model = collision::generate_cylinder(length, width);
    collider = collision::collider(model);
    health = health_;
}
void solid_rocket_motor::update(physics_object::object* parent) override {
    //if (health <= 0) return;
    if (burntime_seconds_remaining <= 0) thrust = 0;
    parent->queue_force(position, thrust * thrust_direction);
    burntime_seconds_remaining -= constants::DELTA_T;
    models.clear();
    double plume_size = cbrt(thrust / 1e4);
    double plume_size_multiplier_normalized = random(0, 1);
    double plume_size_multiplier = 0.5 + 1 * plume_size_multiplier_normalized;
    plume_size *= plume_size_multiplier;
    models.push_back(visual_model(models::flame_trail, position, vector::localspace(plume_size, plume_size, plume_size)));
    parent->physics_state.base_signal_strength = 0;
}

}

void physics_object::object::add_solid_rocket_motor(module::solid_rocket_motor s) {
	std::shared_ptr<module::module> m = std::make_shared<module::solid_rocket_motor>(s);
	properties.modules.push_back(m);
}
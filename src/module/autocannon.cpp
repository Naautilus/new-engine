#include "autocannon.hpp"
#include "../physics_object/object.hpp"

namespace module {

autocannon::autocannon(physics_object::object (*physics_object_creator_)(), double projectile_velocity_, double rounds_per_second_, vector::localspace firing_direction_, vector::localspace position_, double length, double width, double health_) {
    physics_object_creator = physics_object_creator_;
    projectile_velocity = projectile_velocity_;
    rounds_per_second = rounds_per_second_;
    firing_direction = firing_direction_;
    position = position_;
    std::vector<collision::triangle> model = collision::generate_cylinder(length, width);
    collider = collision::collider(model);
    health = health_;
}
void autocannon::update(physics_object::object* parent) {

    ticks_since_last_fired++;
    if (health <= 0) return;
    double trigger_response = parent->control_bindings.get_response(controls::gun1);
    if (trigger_response == 0) return;
    if (ticks_since_last_fired < 1.0/(rounds_per_second*constants::DELTA_T)) return;
    ticks_since_last_fired = 0;

    auto projectile_ = std::make_shared<physics_object::object>(physics_object_creator());
    projectile_->physics_state.position = position.to_worldspace_positional(parent->physics_state.rotation, parent->physics_state.position);
    projectile_->physics_state.rotation = parent->physics_state.rotation;
    projectile_->physics_state.position += vector::localspace(fire_ahead_distance, 0, 0).to_worldspace(parent->physics_state.rotation);
    projectile_->physics_state.velocity = parent->physics_state.velocity;
    projectile_->physics_state.velocity += vector::localspace(projectile_velocity, 0, 0).to_worldspace(parent->physics_state.rotation);
    parent->queue_impulse(position, -projectile_->physics_state.mass * projectile_velocity * firing_direction);
    globals::physics_objects_mutex.lock();
    globals::physics_objects.push_back(projectile_);
    globals::physics_objects_mutex.unlock();

}

}

void physics_object::object::add_autocannon(module::autocannon a) {
    std::shared_ptr<module::module> m = std::make_shared<module::autocannon>(a);
    properties.modules.push_back(m);
}
#pragma once
#include "module.cpp"

namespace module {
    struct autocannon : module {
        physics_object::object (*physics_object_creator)();
        double projectile_velocity;
        double rounds_per_second;
        double fire_ahead_distance = 30;
        int ticks_since_last_fired = 0;
        vector::localspace firing_direction;
        autocannon(physics_object::object (*physics_object_creator_)(), double projectile_velocity_, double rounds_per_second_, vector::localspace firing_direction_, vector::localspace position_, double length, double width, double health_);
        void update(physics_object::object* parent) override;
    };
}

void physics_object::object::add_autocannon(module::autocannon a);
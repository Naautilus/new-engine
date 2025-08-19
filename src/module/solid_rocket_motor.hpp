#pragma once
#include "module.cpp"

namespace module {
    struct solid_rocket_motor : module {
        double thrust;
        double burntime_seconds_remaining;
        vector::localspace thrust_direction;
        solid_rocket_motor(double thrust_, double burntime_seconds_remaining_, vector::localspace thrust_direction_, vector::localspace position_, double length, double width, double health_);
        void update(physics_object::object* parent) override;
    };
}

void physics_object::object::add_solid_rocket_motor(module::solid_rocket_motor s);
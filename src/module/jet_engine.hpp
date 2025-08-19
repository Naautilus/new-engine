#pragma once
#include "module.cpp"

namespace module {
    struct jet_engine : module {
        double max_thrust;
        double minimum_throttle;
        double maximum_throttle;
        vector::localspace thrust_direction;
        double length;
        jet_engine(double max_thrust_, double minimum_throttle_, double maximum_throttle_, vector::localspace thrust_direction_, vector::localspace position_, double length_, double width, double health_);
        void update(physics_object::object* parent) override;
    };
}

void physics_object::object::add_jet_engine(module::jet_engine j);
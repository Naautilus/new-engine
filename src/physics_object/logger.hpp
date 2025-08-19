#pragma once
#include "../simulation_state.cpp"

namespace physics_object {
    // below is for ui/readout purposes
    void object::calculate_acceleration();
    double object::calculate_aoa();
    void object::log();
}
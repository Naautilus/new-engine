#pragma once
#include "object.cpp"

namespace physics_object {
    namespace blueprints {

        object aim9x();
        object bullet(double diameter);
        object bullet_20mm();
        object debris(double mass);
        control_bindings plane_control_bindings_wasd();
        control_bindings plane_control_bindings_ijkl();
        object f16_simple_forces_model();
        object sun();
        object axes(vector::worldspace position_);
        
    }
}
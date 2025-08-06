#pragma once
#include "object.cpp"
/*
namespace physics_object {
    namespace blueprints {

        template <typename T>
        struct named_blueprint {
            std::string name;
            T (*blueprint_creator)();
        };

        // named_blueprints can only hold blueprint creators without inputs, so only inputless creators will be added to these lists
        std::vector<named_blueprint<object>> named_objects_;
        std::vector<named_blueprint<control_bindings>> named_control_bindings_;

        object aim9x();
        object bullet(double diameter);
        object bullet_20mm();
        object debris(double mass);
        control_bindings plane_control_bindings_wasd();
        control_bindings plane_control_bindings_ijkl();
        object f16_simple_forces_model();
        object sun();
        object axes(vector::worldspace position_);
        object collider_visual(vector::worldspace position_, collision::collider collider_);
        
    }
}
    */
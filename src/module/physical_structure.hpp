#pragma once
#include "module.cpp"

namespace module {
    struct physical_structure : module {
        physical_structure(collision::collider collider_, std::shared_ptr<mesh> model_);
        physical_structure(collision::collider collider_, std::shared_ptr<mesh> model_, vector::localspace model_position_, vector::localspace model_scaling_);
        void update(physics_object::object* parent) override;
    };
}

void physics_object::object::add_physical_structure(module::physical_structure s);
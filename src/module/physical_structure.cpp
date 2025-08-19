#pragma once
#include "module.cpp"

namespace module {
    physical_structure::physical_structure(collision::collider collider_, std::shared_ptr<mesh> model_) {
        collider = collider_;
        models.push_back(visual_model(model_));
    }
    physical_structure::physical_structure(collision::collider collider_, std::shared_ptr<mesh> model_, vector::localspace model_position_, vector::localspace model_scaling_) {
        collider = collider_;
        models.push_back(visual_model(model_, model_position_, model_scaling_));
    }
    void physical_structure::update(physics_object::object* parent) override {}
}

void physics_object::object::add_physical_structure(module::physical_structure s) {
    std::shared_ptr<module::module> m = std::make_shared<module::physical_structure>(s);
    properties.modules.push_back(m);
}
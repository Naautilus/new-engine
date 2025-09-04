// top of cpp marker
#include "module.hpp"
#include "../renderer/models.hpp"
#include "../simulation_state.hpp"
#include "../math/random.hpp"

namespace module {
    
visual_model::visual_model(std::shared_ptr<mesh> mesh_data_, vector::localspace position_, vector::localspace scaling_, Eigen::Quaterniond rotation_) {
    mesh_data = mesh_data_;
    position = position_;
    scaling = scaling_;
    rotation = rotation_;
}
visual_model::visual_model(std::shared_ptr<mesh> mesh_data_, vector::localspace position_, vector::localspace scaling_) {
    mesh_data = mesh_data_;
    position = position_;
    scaling = scaling_;
    rotation = Eigen::Quaterniond(1,0,0,0);
}
visual_model::visual_model(std::shared_ptr<mesh> mesh_data_) {
    mesh_data = mesh_data_;
    position = vector::localspace(0,0,0);
    scaling = vector::localspace(1,1,1);
    rotation = Eigen::Quaterniond(1,0,0,0);
}

}
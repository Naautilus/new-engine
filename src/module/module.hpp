#pragma once
#include "../renderer/models.hpp"
#include "../simulation_state.hpp"
#include "../physics_object/object.hpp"
#include "../math/random.hpp"
#include "../collision/meshes.hpp"
#include "../collision/collider.hpp"

/*
module is a general-purpose struct whose children
are things like aerodynamic surfaces, engines, etc.
anything outside the raw physics object which takes
in inputs and exerts forces, etc.

children are stored in a shared modules list with
the datatype std::shared_ptr<module>.

children should:
- be in a unique file module_this_module_type.cpp
- extend : module
- have a void update(physics_object::object* parent) override {} function
- have a void physics_object::object::add_this_module_type(this_module_type t) {} function outside the struct definition
- have a void add_this_module_type(this_module_type t); defined incompletely in the physics_object.cpp file
- have an incomplete type defined at the end of this file
*/

namespace module {
    
struct visual_model {
    std::shared_ptr<mesh> mesh_data;
    vector::localspace position;
    vector::localspace scaling;
    Eigen::Quaterniond rotation;
    visual_model(std::shared_ptr<mesh> mesh_data_, vector::localspace position_, vector::localspace scaling_, Eigen::Quaterniond rotation_);
    visual_model(std::shared_ptr<mesh> mesh_data_, vector::localspace position_, vector::localspace scaling_);
    visual_model(std::shared_ptr<mesh> mesh_data_);
};

struct module {
    double health;
    vector::localspace position;
    Eigen::Quaterniond rotation;
    collision::collider collider;
    std::vector<visual_model> models;
    virtual ~module() = default;
    virtual void update(physics_object::object* parent) = 0;
};

struct aerodynamic_surface;
struct autocannon;
struct jet_engine;
struct physical_structure;
struct sensor_ir;
struct solid_rocket_motor;

}
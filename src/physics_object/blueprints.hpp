#pragma once
#include "../module/module.hpp"
#include "../module/aerodynamic_surface.hpp"
#include "../module/autocannon.hpp"
#include "../module/jet_engine.hpp"
#include "../module/physical_structure.hpp"
#include "../module/sensor_ir.hpp"
#include "../module/solid_rocket_motor.hpp"
#include "../renderer/glad.h"
#include "../math/random.hpp"
#include "../collision/meshes.hpp"
#include "control_bindings.hpp"
#include <GLFW/glfw3.h>

/*
physics_object_blueprints.cpp stores some commonly used physics objects so they don't
need to be entirely reconstructed every time they are created later on.

initialized at origin with 0 velocity, no name, and no control_bindings.
*/

namespace physics_object {
    
namespace blueprints {

template <typename T>
struct named_blueprint {
    T (*blueprint_creator)();
    std::string name;
};

// named_blueprints can only hold blueprint creators without inputs, so only inputless creators will be added to these lists
std::vector<named_blueprint<object>> named_objects;
std::vector<named_blueprint<control_bindings>> named_control_bindings;

object aim9x();
object bullet(double diameter);
object bullet_20mm();
Eigen::Quaterniond random_quaternion();
object debris(double mass);
object debris_1kg();
control_bindings plane_control_bindings_wasd();
control_bindings plane_control_bindings_ijkl();
object f16_simple_forces_model();
object sun();
object axes(vector::worldspace position_, double scale);
object sphere(vector::worldspace position_, double scale);
object cube(vector::worldspace position_, double scale);
object collider_visual(vector::worldspace position_, collision::collider collider_);
void initialize_blueprints();
    
}

}
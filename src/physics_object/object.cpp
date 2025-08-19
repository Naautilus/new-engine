#pragma once
#include "object.hpp"

namespace physics_object {

object::object() {
    properties.name = "unnamed";
    //mutex = std::make_unique<std::mutex>();
}

object::object(std::string name_) {
    properties.name = name_;
    //mutex = std::make_unique<std::mutex>();
}

// below is for applying forces - both force and torque
void object::apply_force(vector::worldspace origin, vector::worldspace direction) {
    origin -= physics_state.position;
    physics_state.velocity += 1/physics_state.mass * constants::DELTA_T * direction;
    vector::worldspace torque = origin.cross(direction);
    Eigen::Matrix3d local_inertia_matrix; // localspace to worldspace rotational inertia calculation made by ChatGPT
    local_inertia_matrix << physics_state.rotational_inertia.x(), 0, 0,
                            0, physics_state.rotational_inertia.y(), 0,
                            0, 0, physics_state.rotational_inertia.z();
    Eigen::Matrix3d rotation_matrix = physics_state.rotation.toRotationMatrix();
    Eigen::Matrix3d world_inertia_matrix = rotation_matrix * local_inertia_matrix * rotation_matrix.transpose();
    Eigen::Vector3d angular_acceleration = world_inertia_matrix.inverse() * torque;
    physics_state.angular_velocity -= constants::DELTA_T * angular_acceleration;
}
void object::queue_force(vector::worldspace origin, vector::worldspace direction) {
    properties.force_queue.push_back(std::pair<vector::worldspace, vector::worldspace>(origin, direction));
}
void object::apply_force(vector::localspace origin, vector::localspace direction) {
    apply_force(origin.to_worldspace_positional(physics_state.rotation, physics_state.position), direction.to_worldspace(physics_state.rotation));
}
void object::queue_force(vector::localspace origin, vector::localspace direction) {
    queue_force(origin.to_worldspace_positional(physics_state.rotation, physics_state.position), direction.to_worldspace(physics_state.rotation));
}

// below is for applying impulses - impulse just applies a force with its magnitude * constants::DELTA_T
void object::apply_impulse(vector::worldspace origin, vector::worldspace direction) {
    apply_force(origin, direction / constants::DELTA_T);
}
void object::queue_impulse(vector::worldspace origin, vector::worldspace direction) {
    queue_force(origin, direction / constants::DELTA_T);
}
void object::apply_impulse(vector::localspace origin, vector::localspace direction) {
    apply_force(origin, direction / constants::DELTA_T);
}
void object::queue_impulse(vector::localspace origin, vector::localspace direction) {
    queue_force(origin, direction / constants::DELTA_T);
}

// below is for force calculation
void object::apply_queued_forces() { // forces must be queued so the result of one does not affect another in the same update
    for (int i = 0; i < properties.force_queue.size(); i++) {
        vector::worldspace origin = properties.force_queue[i].first;
        vector::worldspace direction = properties.force_queue[i].second;
        apply_force(origin, direction);
    }   
    properties.force_queue.clear();
}

void object::update_modules() {
    for (std::shared_ptr<module::module>& m : properties.modules) {
        m->collider.update(physics_state.position, physics_state.velocity, physics_state.rotation);
        m->update(this);
    }
}

void add_aerodynamic_surface(module::aerodynamic_surface a);
void add_autocannon(module::autocannon a);
void add_jet_engine(module::jet_engine j);
void add_physical_structure(module::physical_structure s);
void add_sensor_ir(module::sensor_ir s);
void add_solid_rocket_motor(module::solid_rocket_motor s);

void calculate_acceleration();
double calculate_aoa();
void log();


}
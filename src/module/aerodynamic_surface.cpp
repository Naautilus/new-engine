#include "aerodynamic_surface.hpp"
#include "../physics_object/object.hpp"

namespace module {

aerodynamic_surface::aerodynamic_surface(double s, vector::localspace d, vector::localspace p) {
    surface_area = s;
    unrotated_direction = d;
    position = p;
    response_axes = vector::localspace(0, 0, 0);
    rotation_axis = vector::localspace(0, 0, 1);
    angle_range = 0;
}
aerodynamic_surface::aerodynamic_surface(double s, vector::localspace d, vector::localspace p, vector::localspace response_axes_, vector::localspace rotation_axis_, double a) {
    surface_area = s;
    unrotated_direction = d;
    position = p;
    response_axes = response_axes_;
    rotation_axis = rotation_axis_;
    angle_range = a;
}
void aerodynamic_surface::update(physics_object::object* parent) {
    if (angle_range == 0) update_static_surface(parent);
    else update_dynamic_surface(parent);
}
void aerodynamic_surface::update_static_surface(physics_object::object* parent) {
    vector::worldspace surface_velocity = parent->physics_state.velocity - parent->physics_state.angular_velocity.cross(position.to_worldspace(parent->physics_state.rotation));
    if (surface_velocity.squaredNorm() < SPEED_EPSILON) return;
    //vector::worldspace surface_velocity = parent->physics_state.velocity; // higher performance, a little less accurate
    vector::localspace v = surface_velocity.to_localspace(parent->physics_state.rotation);
    double force = v.dot(unrotated_direction) / v.norm();
    vector::localspace force2 = -force * 0.5 * globals::current_simulation_state->air_density * surface_area * unrotated_direction * v.squaredNorm();
    vector::localspace pos_ = position;
    parent->queue_force(pos_, force2);
}
void aerodynamic_surface::update_dynamic_surface(physics_object::object* parent) {
    vector::localspace rotation_drives(0, 0, 0);
    controls::input* pitch = parent->control_bindings.get_input(controls::pitch);
    if (pitch) rotation_drives.y() = pitch->response_multiplied;
    controls::input* yaw = parent->control_bindings.get_input(controls::yaw);
    if (yaw) rotation_drives.z() = yaw->response_multiplied;
    controls::input* roll = parent->control_bindings.get_input(controls::roll);
    if (roll) rotation_drives.x() = roll->response_multiplied;

    double response = response_axes.dot(rotation_drives);
    response *= angle_range;
    response *= std::numbers::pi / 180;
    rotation = Eigen::AngleAxis(response, rotation_axis);
    rotated_direction = rotation * unrotated_direction;

    vector::worldspace surface_velocity = parent->physics_state.velocity - parent->physics_state.angular_velocity.cross(position.to_worldspace(parent->physics_state.rotation));
    if (surface_velocity.squaredNorm() < SPEED_EPSILON) return;
    vector::localspace v = surface_velocity.to_localspace(parent->physics_state.rotation);
    double force = v.dot(rotated_direction) / v.norm();
    vector::localspace force2 = -force * 0.5 * globals::current_simulation_state->air_density * surface_area * rotated_direction * v.squaredNorm();
    vector::localspace pos_ = position;
    parent->queue_force(pos_, force2);
}

}

void physics_object::object::add_aerodynamic_surface(module::aerodynamic_surface a) {
    std::shared_ptr<module::module> m = std::make_shared<module::aerodynamic_surface>(a);
    properties.modules.push_back(m);
}
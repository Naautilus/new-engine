#include "aerodynamic_surface.hpp"
#include "../physics_object/object.hpp"
#include "../ground/ground_logic.hpp"

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
    if (surface_velocity.squaredNorm() < std::numeric_limits<double>::epsilon()) return;
    vector::localspace v = surface_velocity.to_localspace(parent->physics_state.rotation);
    double force = v.dot(unrotated_direction) / v.norm();
    vector::localspace force2 = -force * 0.5 * ground::fluid_density(position.to_worldspace_positional(parent->physics_state.rotation, parent->physics_state.position).z()) * surface_area * unrotated_direction * v.squaredNorm();
    vector::localspace pos_ = position;
    parent->queue_force(pos_, force2);
}
void aerodynamic_surface::update_dynamic_surface(physics_object::object* parent) {
    vector::localspace rotation_drives(0, 0, 0);
    rotation_drives.y() = parent->control_bindings.get_response(controls::pitch, controls::external);
    rotation_drives.z() = parent->control_bindings.get_response(controls::yaw, controls::external);
    rotation_drives.x() = parent->control_bindings.get_response(controls::roll, controls::external);

    double response = response_axes.dot(rotation_drives);
    response *= angle_range;
    response *= std::numbers::pi / 180;
    rotation = Eigen::AngleAxisd(response, rotation_axis);
    rotated_direction = rotation * unrotated_direction;

    vector::worldspace surface_velocity = parent->physics_state.velocity - parent->physics_state.angular_velocity.cross(position.to_worldspace(parent->physics_state.rotation));
    if (surface_velocity.squaredNorm() < std::numeric_limits<double>::epsilon()) return;
    vector::localspace v = surface_velocity.to_localspace(parent->physics_state.rotation);
    double force = v.dot(rotated_direction) / v.norm();
    vector::localspace force2 = -force * 0.5 * ground::fluid_density(position.to_worldspace_positional(parent->physics_state.rotation, parent->physics_state.position).z()) * surface_area * rotated_direction * v.squaredNorm();
    vector::localspace pos_ = position;
    parent->queue_force(pos_, force2);
}

}

void physics_object::object::add_aerodynamic_surface(module::aerodynamic_surface a) {
    std::shared_ptr<module::module> m = std::make_shared<module::aerodynamic_surface>(a);
    properties.modules.push_back(m);
}
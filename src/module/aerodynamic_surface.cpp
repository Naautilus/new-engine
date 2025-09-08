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
    int SUBFORCES = 100;
    rotate_surface(parent);
    apply_aerodynamic_force(parent, 0);
}

int MAX_FORCE_ITERATIONS = 6;
void aerodynamic_surface::apply_aerodynamic_force(physics_object::object* parent, int iteration) {
    if (iteration >= MAX_FORCE_ITERATIONS) return;

    vector::worldspace original_velocity = parent->physics_state.velocity;
    vector::worldspace original_angular_velocity = parent->physics_state.angular_velocity;
    
    int subdivisions = 1;
    for (int i = 0; i < iteration; i++) subdivisions *= 2;
    double multiplier = 1.0 / subdivisions;
    bool max_deviation_exceeded = false;

    for (int i = 0; i < subdivisions; i++) {
        vector::worldspace surface_velocity = parent->physics_state.velocity.add_angular_velocity(position.to_worldspace(parent->physics_state.rotation), parent->physics_state.angular_velocity);
        if (surface_velocity.squaredNorm() < std::numeric_limits<double>::epsilon()) return;
        vector::localspace v = surface_velocity.to_localspace(parent->physics_state.rotation);
        double force = v.dot(rotated_direction) / v.norm();
        vector::localspace force2 = -force * 0.5 * ground::fluid_density(position.to_worldspace_positional(parent->physics_state.rotation, parent->physics_state.position).z() * -1) * surface_area * rotated_direction * v.squaredNorm();
        
        double MAX_ALLOWED_DOT_PRODUCT_DEVIATION = 0.5;
        vector::worldspace initial_surface_velocity = parent->physics_state.velocity.add_angular_velocity(position.to_worldspace(parent->physics_state.rotation), parent->physics_state.angular_velocity);
        parent->apply_force(position, force2 * multiplier);
        vector::worldspace new_surface_velocity = parent->physics_state.velocity.add_angular_velocity(position.to_worldspace(parent->physics_state.rotation), parent->physics_state.angular_velocity);
        double dot_product = initial_surface_velocity.normalized().dot(new_surface_velocity / initial_surface_velocity.norm());
        
        if (fabs(dot_product - 1.0) > MAX_ALLOWED_DOT_PRODUCT_DEVIATION) {
            max_deviation_exceeded = true;
            break;
        }
    }

    if (max_deviation_exceeded) {
        parent->physics_state.velocity = original_velocity;
        parent->physics_state.angular_velocity = original_angular_velocity;
        iteration++;
        apply_aerodynamic_force(parent, iteration);
    }

}

void aerodynamic_surface::rotate_surface(physics_object::object* parent) {

    if (angle_range == 0) {
        rotated_direction = unrotated_direction;
        return;
    }

    vector::localspace rotation_drives(0, 0, 0);
    rotation_drives.x() = parent->control_bindings.get_response(controls::roll, controls::external);
    rotation_drives.y() = parent->control_bindings.get_response(controls::pitch, controls::external);
    rotation_drives.z() = parent->control_bindings.get_response(controls::yaw, controls::external);

    double response = response_axes.dot(rotation_drives);
    response *= angle_range;
    response *= std::numbers::pi / 180;
    rotation = Eigen::AngleAxisd(response, rotation_axis);
    rotated_direction = rotation * unrotated_direction;
}

}

void physics_object::object::add_aerodynamic_surface(module::aerodynamic_surface a) {
    std::shared_ptr<module::module> m = std::make_shared<module::aerodynamic_surface>(a);
    properties.modules.push_back(m);
}
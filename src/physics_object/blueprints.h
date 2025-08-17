#pragma once

namespace physics_object {
    namespace blueprints {
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
    }
}
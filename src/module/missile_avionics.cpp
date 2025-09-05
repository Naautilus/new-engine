#include "missile_avionics.hpp"
#include "sensor_ir.hpp"
#include "../physics_object/object.hpp"

namespace module {

missile_avionics::missile_avionics(double p, double i, double d, Eigen::Quaterniond rotation_, vector::localspace position_) {
    pid_pitch = pid(p, i, d, 1.0);
    pid_yaw = pid(p, i, d, 1.0);
    pid_roll = pid(p, i, d, 1.0);
    rotation = rotation_;
    position = position_;
}

vector::worldspace missile_avionics::get_worldspace_position(physics_object::object* parent) {
    return position.to_worldspace_positional(parent->physics_state.rotation, parent->physics_state.position);
}

enum guidance_mode {
    NONE,
    INITIAL,
    FAR,
    CLOSE
};

void missile_avionics::update(physics_object::object* parent) {
    sensor_ir* sensor_ptr;
    for (std::shared_ptr<module>& module_ : parent->properties.modules) {
        if (sensor_ptr = dynamic_cast<sensor_ir*>(&(*module_))) break;
    }
    if (!sensor_ptr) return;

    sensor_ptr->update_detection(parent);
    vector::worldspace current_detection_relative_worldspace = sensor_ptr->current_detection_relative_worldspace;
    vector::worldspace detection_velocity = sensor_ptr->detection_velocity;
    time_since_launch += constants::DELTA_T;

    double target_distance = current_detection_relative_worldspace.norm();
    if (target_distance != 0) record_target_distance = fmin(record_target_distance, target_distance);

    guidance_mode guidance_mode_;
    if (time_since_launch < 0.3) {
        guidance_mode_ = NONE;
    } else if (get_time_to_impact_first_degree_prediction(current_detection_relative_worldspace, detection_velocity, parent) < 1.0) {
        guidance_mode_ = CLOSE;
    } else if (time_since_launch > 1.0) {
        guidance_mode_ = FAR;
    } else {
        guidance_mode_ = INITIAL;
    }

    double acceleration = parent->physics_state.recorded_acceleration.norm();
    if (globals::SHOW_MISSILE_ACCELERATION) std::cout << "M/S^2: " << acceleration << std::string((int)(acceleration / constants::STANDARD_GRAVITY), '#') << "\n";
    double gain_limiter = 1-get_g_limit_fraction(acceleration, 40*constants::STANDARD_GRAVITY, 60*constants::STANDARD_GRAVITY);
    vector::localspace guidance_pid_inputs;
    switch(guidance_mode_) {
        case INITIAL:
            guidance_pid_inputs = get_guidance_first_degree_prediction(current_detection_relative_worldspace, detection_velocity, parent, 10.0 * gain_limiter, 275);
            break;
        case FAR:
            guidance_pid_inputs = 
                get_guidance_proportional_navigation(current_detection_relative_worldspace, detection_velocity, parent, 30.0 * gain_limiter) +
                get_guidance_direct(current_detection_relative_worldspace, parent, 5.0 * gain_limiter);
            break;
        case CLOSE:
            guidance_pid_inputs = get_guidance_proportional_navigation(current_detection_relative_worldspace, detection_velocity, parent, 50.0 * gain_limiter);
            break;
        default:
            guidance_pid_inputs = vector::worldspace(0, 0, 0);
            break;
    }
    
    pid_roll.update(guidance_pid_inputs.x());
    pid_pitch.update(guidance_pid_inputs.y());
    pid_yaw.update(guidance_pid_inputs.z());
    
    if (std::isnan(guidance_pid_inputs.x())
    || std::isnan(guidance_pid_inputs.y())
    || std::isnan(guidance_pid_inputs.z())
    || std::isnan(pid_roll.output)
    || std::isnan(pid_pitch.output)
    || std::isnan(pid_yaw.output)) {
        std::cout << "NaN detected in pid inputs/outputs of sensor_ir " << this << "\n";
    }

    controls::input* pitch = parent->control_bindings.get_input(controls::pitch);
    if (pitch) pitch->response_unmultiplied = pid_pitch.output;
    controls::input* yaw = parent->control_bindings.get_input(controls::yaw);
    if (yaw) yaw->response_unmultiplied = pid_yaw.output;
    controls::input* roll = parent->control_bindings.get_input(controls::roll);
    if (roll) roll->response_unmultiplied = pid_roll.output;
}

double missile_avionics::get_g_limit_fraction(double current_acceleration, double g_limit_min, double g_limit_max) {
    double g_limit_fraction = (current_acceleration-g_limit_min) / (g_limit_max-g_limit_min);
    g_limit_fraction = std::clamp(g_limit_fraction, 0.0, 1.0);
    return g_limit_fraction;
}

vector::worldspace missile_avionics::limit_g_forces(vector::worldspace unlimited_inputs, vector::localspace limited_inputs, double current_acceleration, double g_limit_min, double g_limit_max) {
    double g_limit_fraction = get_g_limit_fraction(current_acceleration, g_limit_min, g_limit_max);
    return (1 - g_limit_fraction) * unlimited_inputs + g_limit_fraction * limited_inputs;
}

double missile_avionics::get_time_to_impact_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, vector::worldspace detection_velocity, physics_object::object* parent) {
    vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
    vector::worldspace missile_velocity = parent->physics_state.velocity;
    vector::worldspace enemy_velocity = detection_velocity;
    vector::worldspace relative_velocity = enemy_velocity - missile_velocity;

    double time_to_impact = current_detection_relative_worldspace.norm() / relative_velocity.norm();
    return time_to_impact;
}

double missile_avionics::find_smallest_positive_root(double a, double b, double c) {
    double determinant = b*b - 4*a*c;
    if (determinant < 0) return -1.0;
    if (a == 0) return -1.0;
    double root1 = (-b + sqrt(determinant)) / (2*a);
    double root2 = (-b - sqrt(determinant)) / (2*a);
    if (root1 < 0 && root2 < 0) return -1.0;
    if (root1 > 0 && root2 > 0) return fmin(root1, root2);
    if (root1 > 0) return root1;
    return root2;
}

vector::localspace missile_avionics::get_guidance_direct(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain) {
    vector::scopespace current_detection = signal_point(
        current_detection_relative_worldspace,
        vector::worldspace(0, 0, 0),
        rotation * parent->physics_state.rotation,
        1.0 // unimportant
    ).position_scopespace;
    if (std::isnan(current_detection.distance()) || std::isnan(current_detection.scope_x()) || std::isnan(current_detection.scope_y())) {
        current_detection = vector::scopespace();
    }
    current_detection.scope_x() *= gain;
    current_detection.scope_y() *= gain;
    vector::localspace output(
        -parent->physics_state.angular_velocity.to_localspace(parent->physics_state.rotation).x(),
        -current_detection.scope_y(),
        current_detection.scope_x()
    );
    return output;
}

vector::localspace missile_avionics::get_guidance_target_velocity(vector::worldspace current_detection_relative_worldspace, vector::worldspace detection_velocity, physics_object::object* parent, double gain) {
    vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
    vector::worldspace enemy_velocity = detection_velocity;
    vector::worldspace aimpoint = enemy_velocity;
    return get_guidance_direct(aimpoint, parent, gain);
}

vector::localspace missile_avionics::get_guidance_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, vector::worldspace detection_velocity, physics_object::object* parent, double gain, double speed_adjustment) {
    vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
    vector::worldspace missile_velocity = parent->physics_state.velocity;
    missile_velocity *= (missile_velocity.norm() + speed_adjustment) / missile_velocity.norm();
    vector::worldspace enemy_velocity = detection_velocity;
    vector::worldspace relative_position = current_detection_relative_worldspace;

    double quadratic_c = relative_position.dot(relative_position);
    double quadratic_b = 2 * relative_position.dot(enemy_velocity);
    double quadratic_a = enemy_velocity.dot(enemy_velocity) - missile_velocity.dot(missile_velocity);

    double time = find_smallest_positive_root(quadratic_a, quadratic_b, quadratic_c);
    if (time == -1.0) return vector::localspace(0, 0, 0);

    vector::worldspace aimpoint = current_detection_relative_worldspace + time * enemy_velocity;

    return get_guidance_direct(aimpoint, parent, gain);
}

vector::localspace missile_avionics::get_guidance_proportional_navigation(vector::worldspace current_detection_relative_worldspace, vector::worldspace detection_velocity, physics_object::object* parent, double gain) {
    double LOOK_AHEAD_TIME = 1; // for turning an acceleration request into an aimpoint request
    vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
    vector::worldspace missile_velocity = parent->physics_state.velocity;
    vector::worldspace enemy_velocity = detection_velocity;
    vector::worldspace relative_velocity = enemy_velocity - missile_velocity;
    vector::worldspace relative_position = current_detection_relative_worldspace;
    vector::worldspace line_of_sight_rotation_vector = (relative_position.cross(relative_velocity)) / (relative_position.dot(relative_position));
    vector::worldspace desired_acceleration = -gain * ((relative_velocity.norm()) * (missile_velocity / missile_velocity.norm())).cross(line_of_sight_rotation_vector);
    vector::worldspace aimpoint = LOOK_AHEAD_TIME * missile_velocity + 0.5 * LOOK_AHEAD_TIME * LOOK_AHEAD_TIME * desired_acceleration;
    vector::scopespace current_detection = signal_point(aimpoint, rotation * parent->physics_state.rotation).position_scopespace;
    return get_guidance_direct(aimpoint, parent, gain);
}

}

void physics_object::object::add_missile_avionics(module::missile_avionics a) {
	std::shared_ptr<module::module> m = std::make_shared<module::missile_avionics>(a);
	properties.modules.push_back(m);
}
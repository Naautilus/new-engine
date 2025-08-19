#include "logger.hpp"

namespace physics_object {
    
// below is for ui/readout purposes
void object::calculate_acceleration() {
    vector::worldspace v_change = physics_state.velocity - physics_state.previous_velocity;
    physics_state.recorded_acceleration = v_change.to_localspace(physics_state.rotation) / constants::DELTA_T;
    physics_state.previous_velocity = physics_state.velocity;
}
double object::calculate_aoa() {
    vector::localspace v = physics_state.velocity.to_localspace(physics_state.rotation);
    return atan2(v.z(), vector::localspace(v.x(), v.y(), 0).norm()) * 180 / std::numbers::pi;
}
void object::log() {
    std::cout << properties.name;
    std::cout << std::string(std::max(10 - (int)properties.name.length(), 0), ' ') << ": ";
    std::cout << "position {" << std::format(constants::FORMAT_STRING_POSITION, physics_state.position.x()) << ", " << std::format(constants::FORMAT_STRING_POSITION, physics_state.position.y()) << ", " << std::format(constants::FORMAT_STRING_POSITION, physics_state.position.z()) << "} ";
    std::cout << "velocity {" << std::format(constants::FORMAT_STRING_VELOCITY, physics_state.velocity.x()) << ", " << std::format(constants::FORMAT_STRING_VELOCITY, physics_state.velocity.y()) << ", " << std::format(constants::FORMAT_STRING_VELOCITY, physics_state.velocity.z()) << "} ";
    //Eigen::Vector3d euler = rotation.toRotationMatrix().eulerAngles(2, 1, 0);
    //std::cout << "rotation YPR {"
    //          << std::format(FORMAT_STRING_ROTATION, euler[0] * 180.0 / M_PI) << "°, "  // Yaw (around Z axis)
    //          << std::format(FORMAT_STRING_ROTATION, euler[1] * 180.0 / M_PI) << "°, "  // Pitch (around Y axis)
    //          << std::format(FORMAT_STRING_ROTATION, euler[2] * 180.0 / M_PI) << "°} ";  // Roll (around X axis)
    std::cout << "forward vector {"
                        << std::format(constants::FORMAT_STRING_UNIT, vector::localspace(1,0,0).to_worldspace(physics_state.rotation).x()) << ", "
                        << std::format(constants::FORMAT_STRING_UNIT, vector::localspace(1,0,0).to_worldspace(physics_state.rotation).y()) << ", "
                        << std::format(constants::FORMAT_STRING_UNIT, vector::localspace(1,0,0).to_worldspace(physics_state.rotation).z()) << "} ";
    std::cout << "angular_velocity PYR {"
                        << std::format(constants::FORMAT_STRING_ANGULAR_VELOCITY, physics_state.angular_velocity.y() * 180.0 / std::numbers::pi) << "°, " 
                        << std::format(constants::FORMAT_STRING_ANGULAR_VELOCITY, physics_state.angular_velocity.z() * 180.0 / std::numbers::pi) << "°, " 
                        << std::format(constants::FORMAT_STRING_ANGULAR_VELOCITY, physics_state.angular_velocity.x() * 180.0 / std::numbers::pi) << "°} ";
    std::cout << "M/S: " << std::format(constants::FORMAT_STRING_SPEED, physics_state.velocity.norm()) << " ";
    std::cout << "G: " 
                        << std::format(constants::FORMAT_STRING_G_FORCE, physics_state.recorded_acceleration.norm() / constants::STANDARD_GRAVITY) << " {"
                        << std::format(constants::FORMAT_STRING_G_FORCE, physics_state.recorded_acceleration.x() / constants::STANDARD_GRAVITY) << ", "
                        << std::format(constants::FORMAT_STRING_G_FORCE, physics_state.recorded_acceleration.y() / constants::STANDARD_GRAVITY) << ", "
                        << std::format(constants::FORMAT_STRING_G_FORCE, physics_state.recorded_acceleration.z() / constants::STANDARD_GRAVITY) << "} ";
    std::cout << "rotation drives PYR { ";
    controls::input* pitch = control_bindings.get_input(controls::pitch);
    if (pitch) std::cout << std::format(constants::FORMAT_STRING_UNIT, pitch->response_unmultiplied) << ", ";
    else std::cout << std::string(5, ' ') << ", ";
    controls::input* yaw = control_bindings.get_input(controls::yaw);
    if (yaw) std::cout << std::format(constants::FORMAT_STRING_UNIT, yaw->response_unmultiplied) << ", ";
    else std::cout << std::string(5, ' ') << ", ";
    controls::input* roll = control_bindings.get_input(controls::roll);
    if (roll) std::cout << std::format(constants::FORMAT_STRING_UNIT, roll->response_unmultiplied) << "} ";
    else std::cout << std::string(5, ' ') << "} ";
    controls::input* thrust = control_bindings.get_input(controls::engine1);
    if (thrust) std::cout << "thrust " << std::format(constants::FORMAT_STRING_UNIT, thrust->response_unmultiplied) << " ";
    else std::cout << std::string(13, ' ');
    std::cout << "AoA " << std::format(constants::FORMAT_STRING_AOA, calculate_aoa()) << " ";
    std::cout << "ALT " << std::format(constants::FORMAT_STRING_ALTITUDE, physics_state.position.z()) << " ";
    std::cout << "@ " << std::format(constants::FORMAT_STRING_SPEED, physics_state.velocity.z()) << " ";
    std::cout << "health " << std::format(constants::FORMAT_STRING_SPEED, physics_state.health) << " ";
    std::cout << "\n";
}

}
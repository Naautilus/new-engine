#include "flight_controller.hpp"
#include "../physics_object/object.hpp"
#include "../controls/controls.hpp"

namespace module {

flight_controller::flight_controller(
    pid pid_roll_, double max_roll_rate_, 
    pid pid_pitch_, double max_pitch_rate_, 
    pid pid_yaw_, double max_yaw_rate_,
    physics_object::object* parent) {
    
    std::cout << "flight_controller creation\n";

    pid_roll = pid_roll_;
    pid_pitch = pid_pitch_;
    pid_yaw = pid_yaw_;

    max_roll_rate = max_roll_rate_;
    max_pitch_rate = max_pitch_rate_;
    max_yaw_rate = max_yaw_rate_;

    collider = collision::collider();
    
    controls::input roll = controls::input(
        controls::roll, controls::response_type::instant, controls::input_destination::external, -1.0, 1.0, 1.0);
    parent->control_bindings.inputs.push_back(roll);

    controls::input pitch = controls::input(
        controls::pitch, controls::response_type::instant, controls::input_destination::external, -1.0, 1.0, 1.0);
    parent->control_bindings.inputs.push_back(pitch);

    controls::input yaw = controls::input(
        controls::yaw, controls::response_type::instant, controls::input_destination::external, -1.0, 1.0, 1.0);
    parent->control_bindings.inputs.push_back(yaw);
}

void flight_controller::update(physics_object::object* parent) {

    double desired_roll_rate = max_roll_rate * parent->control_bindings.get_response(controls::roll, controls::flight_controller);
    double desired_pitch_rate = max_pitch_rate * parent->control_bindings.get_response(controls::pitch, controls::flight_controller);
    double desired_yaw_rate = max_yaw_rate * parent->control_bindings.get_response(controls::yaw, controls::flight_controller);

    std::cout << "desired_roll_rate: " << desired_roll_rate << "\n";
    std::cout << "desired_pitch_rate: " << desired_pitch_rate << "\n";
    std::cout << "desired_yaw_rate: " << desired_yaw_rate << "\n";
    
    desired_rotation = Eigen::AngleAxisd(desired_roll_rate, vector::worldspace::UnitX()) * desired_rotation;
    desired_rotation = Eigen::AngleAxisd(desired_pitch_rate, vector::worldspace::UnitY()) * desired_rotation;
    desired_rotation = Eigen::AngleAxisd(desired_yaw_rate, vector::worldspace::UnitZ()) * desired_rotation;

    parent->physics_state.rotation = desired_rotation;
    
    std::cout << "parent rotation: " << parent->physics_state.rotation << "\n";
    std::cout << "desired rotation: " << desired_rotation << "\n";
    Eigen::Quaterniond rotation_error = parent->physics_state.rotation * desired_rotation.conjugate();
    std::cout << "rotation error: " << rotation_error << "\n";

    Eigen::Vector3d rotation_error_euler_angles = rotation_error.toRotationMatrix().canonicalEulerAngles(2, 1, 0);

    double roll_error = rotation_error_euler_angles.z();
    double pitch_error = rotation_error_euler_angles.y();
    double yaw_error = rotation_error_euler_angles.x();

    /*
    std::cout << "flight_controller desired_roll_rate: " << desired_roll_rate << "\n";
    std::cout << "flight_controller desired_pitch_rate: " << desired_pitch_rate << "\n";
    std::cout << "flight_controller desired_yaw_rate: " << desired_yaw_rate << "\n";
    std::cout << "flight_controller actual_roll_rate: " << actual_roll_rate << "\n";
    std::cout << "flight_controller actual_pitch_rate: " << actual_pitch_rate << "\n";
    std::cout << "flight_controller actual_yaw_rate: " << actual_yaw_rate << "\n";
    */

    std::cout << "roll_error: " << roll_error << "\n";
    std::cout << "pitch_error: " << pitch_error << "\n";
    std::cout << "yaw_error: " << yaw_error << "\n";

    pid_roll.update(roll_error);
    pid_pitch.update(pitch_error);
    pid_yaw.update(yaw_error);

    /*
    std::cout << "flight_controller roll output: " << pid_roll.output << "\n";
    std::cout << "flight_controller pitch output: " << pid_pitch.output << "\n";
    std::cout << "flight_controller yaw output: " << pid_yaw.output << "\n";
    */

    controls::input* roll = parent->control_bindings.get_first_input(controls::roll, controls::external);
    if (roll) roll->response_unmultiplied = pid_roll.output;
    else std::cout << "flight_controller on " << parent->properties.name << ": roll input not found\n";
    controls::input* pitch = parent->control_bindings.get_first_input(controls::pitch, controls::external);
    if (pitch) pitch->response_unmultiplied = pid_pitch.output;
    else std::cout << "flight_controller on " << parent->properties.name << ": pitch input not found\n";
    controls::input* yaw = parent->control_bindings.get_first_input(controls::yaw, controls::external);
    if (yaw) yaw->response_unmultiplied = pid_yaw.output;
    else std::cout << "flight_controller on " << parent->properties.name << ": yaw input not found\n";
}

}

void physics_object::object::add_flight_controller(module::flight_controller f) {
	std::shared_ptr<module::module> m = std::make_shared<module::flight_controller>(f);
	properties.modules.push_back(m);
}
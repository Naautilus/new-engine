#include "flight_controller.hpp"
#include "../physics_object/object.hpp"
#include "../controls/controls.hpp"

namespace module {

flight_controller::flight_controller(
    roll_controller  roll_, 
    pitch_controller pitch_, 
    yaw_controller   yaw_,
    max_setpoint_deviation max_setpoint_deviation__,
    physics_object::object* parent) {
    
    std::cout << "flight_controller creation\n";

    roll  = roll_;
    pitch = pitch_;
    yaw   = yaw_;

    max_setpoint_deviation_.degrees = max_setpoint_deviation__.degrees;

    collider = collision::collider();
    
    controls::input roll_input = controls::input(
        controls::roll, controls::response_type::instant, controls::input_destination::external, -1.0, 1.0, 1.0);
    parent->control_bindings.inputs.push_back(roll_input);

    controls::input pitch_input = controls::input(
        controls::pitch, controls::response_type::instant, controls::input_destination::external, -1.0, 1.0, 1.0);
    parent->control_bindings.inputs.push_back(pitch_input);

    controls::input yaw_input = controls::input(
        controls::yaw, controls::response_type::instant, controls::input_destination::external, -1.0, 1.0, 1.0);
    parent->control_bindings.inputs.push_back(yaw_input);
}

void flight_controller::update(physics_object::object* parent) {

    double desired_roll_rate  = roll .rate_limit_.angular_velocity * parent->control_bindings.get_response(controls::roll,  controls::flight_controller);
    double desired_pitch_rate = pitch.rate_limit_.angular_velocity * parent->control_bindings.get_response(controls::pitch, controls::flight_controller);
    double desired_yaw_rate   = yaw  .rate_limit_.angular_velocity * parent->control_bindings.get_response(controls::yaw,   controls::flight_controller);
    
    vector::localspace velocity_localspace = parent->physics_state.velocity.to_localspace(parent->physics_state.rotation);

    double pitch_aoa = 180.0 / std::numbers::pi * atan2(velocity_localspace.z(), velocity_localspace.x());
    double yaw_aoa   = 180.0 / std::numbers::pi * atan2(velocity_localspace.y(), velocity_localspace.x());
    double pitch_multiplier_for_aoa = (fabs(pitch_aoa) - pitch.aoa_limit_.degrees_start) / (pitch.aoa_limit_.degrees_end - pitch.aoa_limit_.degrees_start);
    double yaw_multiplier_for_aoa   = (fabs(yaw_aoa)   - yaw  .aoa_limit_.degrees_start) / (yaw  .aoa_limit_.degrees_end - yaw  .aoa_limit_.degrees_start);
    pitch_multiplier_for_aoa = std::clamp(1.0 - pitch_multiplier_for_aoa, 0.0, 1.0);
    yaw_multiplier_for_aoa   = std::clamp(1.0 - yaw_multiplier_for_aoa  , 0.0, 1.0);
    desired_pitch_rate *= pitch_multiplier_for_aoa;
    desired_yaw_rate   *= yaw_multiplier_for_aoa;

    desired_pitch_rate -= velocity_localspace.z() * fabs(velocity_localspace.z()) * pitch.artificial_stability_.angular_velocity_per_v_squared;
    desired_yaw_rate   += velocity_localspace.y() * fabs(velocity_localspace.y()) * yaw  .artificial_stability_.angular_velocity_per_v_squared;

    desired_rotation = desired_rotation * Eigen::AngleAxisd(desired_roll_rate,  vector::worldspace::UnitX());
    desired_rotation = desired_rotation * Eigen::AngleAxisd(desired_pitch_rate, vector::worldspace::UnitY());
    desired_rotation = desired_rotation * Eigen::AngleAxisd(desired_yaw_rate,   vector::worldspace::UnitZ());

    Eigen::Quaterniond rotation_error = desired_rotation * parent->physics_state.rotation.conjugate();

    Eigen::Quaterniond rotation_error_no_roll;
    {
        Eigen::Vector3d rotation_error_euler_angles = rotation_error.toRotationMatrix().canonicalEulerAngles(2, 1, 0);
        rotation_error_no_roll = Eigen::AngleAxisd(rotation_error_euler_angles[0], vector::worldspace::UnitZ()) *
                                 Eigen::AngleAxisd(rotation_error_euler_angles[1], vector::worldspace::UnitY());

    }

    Eigen::AngleAxisd rotation_error_angle_axis(rotation_error);
    Eigen::AngleAxisd rotation_error_no_roll_angle_axis(rotation_error_no_roll);
    Eigen::AngleAxisd rotation_error_blended_roll_angle_axis(rotation_error_no_roll.slerp(max_setpoint_deviation_.roll_importance_multiplier, rotation_error));

    double angle_saturation = 1 - (fmin(rotation_error_blended_roll_angle_axis.angle(), max_setpoint_deviation_.degrees * std::numbers::pi / 180.0) / rotation_error_blended_roll_angle_axis.angle());
    if (rotation_error_blended_roll_angle_axis.angle() == 0) angle_saturation = 0;

    rotation_error_angle_axis.angle() *= (1 - angle_saturation);

    std::cout << "angle_saturation: " << angle_saturation << "\n";

    Eigen::Quaterniond limited_rotation_error = Eigen::Quaterniond::Identity() * rotation_error_angle_axis;
    desired_rotation = limited_rotation_error * parent->physics_state.rotation;

    Eigen::Vector3d rotation_error_euler_angles = rotation_error.toRotationMatrix().canonicalEulerAngles(2, 1, 0);

    double roll_error  = rotation_error_euler_angles.z();
    double pitch_error = rotation_error_euler_angles.y();
    double yaw_error   = rotation_error_euler_angles.x();

    roll .pid_.update(roll_error );
    pitch.pid_.update(pitch_error);
    yaw  .pid_.update(yaw_error  );

    controls::input* roll_input  = parent->control_bindings.get_first_input(controls::roll,  controls::external);
    controls::input* pitch_input = parent->control_bindings.get_first_input(controls::pitch, controls::external);
    controls::input* yaw_input   = parent->control_bindings.get_first_input(controls::yaw,   controls::external);

    if (roll_input)  roll_input ->response_unmultiplied = roll .pid_.output;
    if (pitch_input) pitch_input->response_unmultiplied = pitch.pid_.output;
    if (yaw_input)   yaw_input  ->response_unmultiplied = yaw  .pid_.output;

    if (!roll_input)  std::cout << "flight_controller on " << parent->properties.name << ": roll input not found\n";
    if (!pitch_input) std::cout << "flight_controller on " << parent->properties.name << ": pitch input not found\n";
    if (!yaw_input)   std::cout << "flight_controller on " << parent->properties.name << ": yaw input not found\n";
}

}

void physics_object::object::add_flight_controller(module::flight_controller f) {
	std::shared_ptr<module::module> m = std::make_shared<module::flight_controller>(f);
	properties.modules.push_back(m);
}
#include "camera_properties.hpp"

camera_properties::camera_properties(std::string camera_target_name_, bool camera_target_search_direction_, vector::localspace camera_target_offset_) {
    camera_target_name = camera_target_name_;
    camera_target_search_direction = camera_target_search_direction_;
    camera_target_offset = camera_target_offset_;
}
void camera_properties::update(Eigen::Quaterniond rotation, vector::worldspace position, vector::worldspace velocity, track_mode track_mode_) {
    vector::worldspace camera_position_;
    if (track_mode_ == TRACKING) camera_position_ = camera_target_offset.to_worldspace_positional(rotation, position);
    else camera_position_ = position;
    camera_position[0] =  camera_position_.y();
    camera_position[1] =  camera_position_.z();
    camera_position[2] = -camera_position_.x();
    vector::worldspace camera_up_direction = vector::localspace(0, 0, 1).to_worldspace(rotation);
    camera_y_direction[0] =  camera_up_direction.y();
    camera_y_direction[1] =  camera_up_direction.z();
    camera_y_direction[2] = -camera_up_direction.x();
    vector::worldspace camera_forward_direction = vector::localspace(1, 0, 0).to_worldspace(rotation);
    camera_z_direction[0] =  camera_forward_direction.y();
    camera_z_direction[1] =  camera_forward_direction.z();
    camera_z_direction[2] = -camera_forward_direction.x();
    last_camera_position = camera_position_;
    last_camera_target_velocity = velocity;
}
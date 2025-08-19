#pragma once
#include "../simulation_logic/physics_step_logic.cpp"
#include "camera_properties.cpp"
#include "models.cpp"
#include "../collision/collider_type.h"
#include "../collision/collider.cpp"
#include "renderer.cpp"


vector::localspace camera_offset;
struct renderer;

// chatGPT code below
Eigen::Quaterniond adjust_quaternion_angle(Eigen::Quaterniond q, double angle_multiplier);
// https://math.stackexchange.com/questions/61146/averaging-quaternions
Eigen::Quaterniond average_approx(std::vector<Eigen::Quaterniond>& quats);
mesh get_ground_model_sub(double vertical_offset, double original_tile_size, double tile_size, int count, int count_deadzone, bool color_variation, int x_, int y_, vector::localspace& last_camera_position_);

const int GROUND_LODS = 16;
const double GROUND_INITIAL_TILE_SIZE = 0.4;//25.0;
int GROUND_TILE_COUNT = 32;
const int GROUND_DEADZONE_TILES = 2;

std::vector<mesh> get_ground_model(bool color_variation, vector::localspace last_camera_position_);

/*
camera_track_physics_object returns a pointer to the object being tracked because its
mutex is locked to prevent it from being moved inbetween here and the
model rendering and the mutex needs to be unlocked later
*/
std::shared_ptr<physics_object::object> camera_track_physics_object(camera_properties& camera_properties_);

vector::worldspace get_triangle_normal(int i, mesh& model);
float three_point_interpolate(double value_at_neg1, double value_at0, double value_at_pos1, double x);
float linear_interpolate(double value_at0, double value_at1, double x);
float smoothstep(double edge0, double edge1, double x);
void set_vertex_colors_by_brightness(vertex& v, double brightness_unfiltered);
void apply_sunlight_to_model(mesh& model);
void recalculate_ground(bool& new_ground_ready_, std::vector<mesh>& ground_, vector::worldspace last_camera_position, vector::worldspace last_camera_target_velocity);
int ground_models_start_point = -1;

void renderer::create_ground_models(std::vector<mesh>& models, camera_properties& camera_properties_, std::vector<mesh>& ground, bool& new_ground_ready);
void renderer::create_models_from_physics_objects(std::vector<mesh>& models, camera_properties& camera_properties_, bool& new_ground_ready);
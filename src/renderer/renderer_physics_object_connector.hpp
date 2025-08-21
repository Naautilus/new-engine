#pragma once
#include "../simulation_logic/physics_step_logic.hpp"
#include "camera_properties.hpp"
#include "models.hpp"
#include "../collision/collider_type.hpp"
#include "../collision/collider.hpp"
#include "renderer.hpp"


extern vector::localspace camera_offset;

struct renderer;

// chatGPT code below
Eigen::Quaterniond adjust_quaternion_angle(Eigen::Quaterniond q, double angle_multiplier);
// https://math.stackexchange.com/questions/61146/averaging-quaternions
Eigen::Quaterniond average_approx(std::vector<Eigen::Quaterniond>& quats);
mesh get_ground_model_sub(double vertical_offset, double original_tile_size, double tile_size, int count, int count_deadzone, bool color_variation, int x_, int y_, vector::localspace& last_camera_position_);

extern const int GROUND_LODS;
extern const double GROUND_INITIAL_TILE_SIZE;
extern int GROUND_TILE_COUNT;
extern const int GROUND_DEADZONE_TILES;

std::vector<mesh> get_ground_model(bool color_variation, vector::localspace last_camera_position_);

/*
camera_track_physics_object returns a pointer to the object being tracked because its
mutex is locked to prevent it from being moved inbetween here and the
model rendering and the mutex needs to be unlocked later
*/
std::shared_ptr<physics_object::object> camera_track_physics_object(camera_properties& camera_properties_);

vector::worldspace get_triangle_normal(int i, mesh& model);
double three_point_interpolate(double value_at_neg1, double value_at0, double value_at_pos1, double x);
double linear_interpolate(double value_at0, double value_at1, double x);
double smoothstep(double edge0, double edge1, double x);
void set_vertex_colors_by_brightness(vertex& v, double brightness_unfiltered);
void apply_sunlight_to_model(mesh& model);
void recalculate_ground(bool& new_ground_ready_, std::vector<mesh>& ground_, vector::worldspace last_camera_position, vector::worldspace last_camera_target_velocity);
extern int ground_models_start_point;
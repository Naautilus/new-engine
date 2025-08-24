// top of cpp marker
#include "renderer_physics_object_connector.hpp"

vector::localspace camera_offset;

// chatGPT code below
Eigen::Quaterniond adjust_quaternion_angle(Eigen::Quaterniond q, double angle_multiplier) {
		double theta = 2 * acos(q.w());  // Original angle in radians
		double sin_half_theta = sqrt(1 - q.w() * q.w());

		// Axis of the original quaternion
		vector::localspace axis;
		if (sin_half_theta > 0.0001) {  // Check to avoid division by zero
				axis.x() = q.x() / sin_half_theta;
				axis.y() = q.y() / sin_half_theta;
				axis.z() = q.z() / sin_half_theta;
		} else {  // Quaternion is approximately a no-rotation, axis doesn't matter
				axis.x() = 1.0;
				axis.y() = 0.0;
				axis.z() = 0.0;
		}

		Eigen::Quaterniond output;

		// Adjust the angle
		double new_theta = theta * angle_multiplier;
		output.w() = cos(new_theta / 2);
		output.x() = axis.x() * sin(new_theta / 2);
		output.y() = axis.y() * sin(new_theta / 2);
		output.z() = axis.z() * sin(new_theta / 2);
		return output;
}

// https://math.stackexchange.com/questions/61146/averaging-quaternions
Eigen::Quaterniond average_approx(std::vector<Eigen::Quaterniond>& quats) {
		Eigen::Quaterniond q_avg = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
		for (int i = 0; i < quats.size(); i++) {
				Eigen::Quaterniond q = quats[i];
				double weight = 1.0;
				
				// Correct for double cover, by ensuring that dot product
				// of quats[i] and quats[0] is positive
				if (i > 0 && quats[i].dot(quats[0]) < 0.0) {
						weight = -weight;
				}
				
				q_avg.w() += weight * q.w();
				q_avg.x() += weight * q.x();
				q_avg.y() += weight * q.y();
				q_avg.z() += weight * q.z();
		}
		q_avg.normalize();
		return q_avg;
}

namespace {

mesh get_terrain_model_tile(double vertical_offset, double original_tile_size, double tile_size, int count, int count_deadzone, bool color_variation, int x_, int y_, vector::localspace& last_camera_position_, bool water) {
    mesh model;

	if (color_variation) {
        model = *models::ground_color_varying;
	} else {
        model = *models::ground_color_uniform;
	}
    
    bool is_edge_tile = (abs(x_) <= count_deadzone && abs(y_) <= count_deadzone && count_deadzone != 0);

    bool is_x_edge_tile = (abs(x_) == count_deadzone);
    bool is_y_edge_tile = (abs(y_) == count_deadzone);

    bool is_x_positive_edge_tile = (is_x_edge_tile && x_ > 0);
    bool is_x_negative_edge_tile = (is_x_edge_tile && x_ < 0);
    bool is_y_positive_edge_tile = (is_y_edge_tile && y_ > 0);
    bool is_y_negative_edge_tile = (is_y_edge_tile && y_ < 0);

    if (is_edge_tile) {
        for (vertex& v : model.vertices) {     

            bool is_x_positive_edge_vertex = (v.x != 0);
            bool is_x_negative_edge_vertex = (v.x == 0);
            bool is_y_positive_edge_vertex = (v.y != 0);
            bool is_y_negative_edge_vertex = (v.y == 0);

            if (is_x_positive_edge_tile && is_x_positive_edge_vertex) continue;
            if (is_x_negative_edge_tile && is_x_negative_edge_vertex) continue;
            if (is_y_positive_edge_tile && is_y_positive_edge_vertex) continue;
            if (is_y_negative_edge_tile && is_y_negative_edge_vertex) continue;

            v.z -= 0.1 * tile_size;
        }
    }

	//double x_center = last_camera_position.x() + last_camera_forward_direction.x() * tile_size * count * 0;
	//double y_center = last_camera_position.y() + last_camera_forward_direction.y() * tile_size * count * 0;
	//double x_pos = (x_ + round(x_center/tile_size)) * tile_size;
	//double y_pos = (y_ + round(y_center/tile_size)) * tile_size;
	double x_pos = (x_ + round(last_camera_position_.x()/tile_size)) * tile_size;
	double y_pos = (y_ + round(last_camera_position_.y()/tile_size)) * tile_size;
	double z_pos = vertical_offset;
	for (vertex& v : model.vertices) {
		v.x *= tile_size;
		v.y *= tile_size;
	}
	for (vertex& v : model.vertices) {
		v.x += x_pos;
		v.y += y_pos;
		v.z += z_pos;
	}
    if (!water) {
        for (vertex& v : model.vertices) {
            int sample_points_squared = round(fmin(tile_size/original_tile_size, 3.0)); //std::round(sqrt(tile_size / original_tile_size));
            v.z += ground::get_ground_altitude_averaged(v.x, v.y, tile_size, sample_points_squared);
            color color_ = ground::get_ground_color_averaged(v.x, v.y, tile_size, sample_points_squared);
            v.r += color_.r;
            v.g += color_.g;
            v.b += color_.b;
        }
    } else {
        for (vertex& v : model.vertices) {
            v.z += constants::WATER_LEVEL;

            /*
            //for reflective overcast appearance:
            v.r = 0;
            v.g = 0.175;
            v.b = 0.20;
            */

            //for reflective clear-sky appearance:
            v.r = 0;
            v.g = 0.065;
            v.b = 0.1;
        }
    }

    for (vertex& v : model.vertices) {
        double x_relative = v.x - last_camera_position_.x();
        double y_relative = v.y - last_camera_position_.y();
        double distance = sqrt(x_relative * x_relative + y_relative * y_relative);
        if (distance > constants::PLANET_RADIUS) v.z += std::numeric_limits<double>::quiet_NaN();
        v.z += sqrt(constants::PLANET_RADIUS * constants::PLANET_RADIUS - distance * distance) - constants::PLANET_RADIUS;
    }

    // convert to opengl coordinate system
	for (vertex& v : model.vertices) {
		double x_ = v.x;
		double y_ = v.y;
		double z_ = v.z;
		v.x = y_;
		v.y = z_;
		v.z = -x_;
	}
	return model;
}

std::vector<mesh> get_terrain_model_lods(bool color_variation, vector::localspace last_camera_position_, const int LODS, const double INITIAL_TILE_SIZE, int TILE_COUNT, const int DEADZONE_TILES, bool water) {
	std::vector<mesh> output;
	double tile_size = INITIAL_TILE_SIZE;
	double tile_deadzone = TILE_COUNT / 2 - DEADZONE_TILES;
	for (int i = 0; i < LODS; i++) {
		for (int x_ = -TILE_COUNT; x_ < TILE_COUNT; x_++) {
			for (int y_ = -TILE_COUNT; y_ < TILE_COUNT; y_++) {
				if (i > 0 && abs(x_) < tile_deadzone && abs(y_) < tile_deadzone) continue;
				mesh model = get_terrain_model_tile(0, INITIAL_TILE_SIZE, tile_size, TILE_COUNT, i==0?0:tile_deadzone, color_variation, x_, y_, last_camera_position_, water);
				output.push_back(model);
			}
		}
		tile_size *= 2;
	}
	return output;
}

const int GROUND_LODS = 18;
const double GROUND_INITIAL_TILE_SIZE = 1.6;
int GROUND_TILE_COUNT = 32;
const int GROUND_DEADZONE_TILES = 2;

const int WATER_LODS = 9;
const double WATER_INITIAL_TILE_SIZE = 1000;
int WATER_TILE_COUNT = 20;
const int WATER_DEADZONE_TILES = 2;

}

std::vector<mesh> get_terrain_model(bool color_variation, vector::localspace last_camera_position_) {
	std::vector<mesh> ground = get_terrain_model_lods(color_variation, last_camera_position_, GROUND_LODS, GROUND_INITIAL_TILE_SIZE, GROUND_TILE_COUNT, GROUND_DEADZONE_TILES, false);
	std::vector<mesh> water = get_terrain_model_lods(color_variation, last_camera_position_, WATER_LODS, WATER_INITIAL_TILE_SIZE, WATER_TILE_COUNT, WATER_DEADZONE_TILES, true);
	ground.reserve(ground.size() + water.size());
    for (mesh& m : water) ground.push_back(m);
    return ground;
}

/*
camera_track_physics_object returns a pointer to the object being tracked because its
mutex is locked to prevent it from being moved inbetween here and the
model rendering and the mutex needs to be unlocked later
*/
std::shared_ptr<physics_object::object> camera_track_physics_object(camera_properties& camera_properties_) {
    if (globals::free_camera) return nullptr;

	std::shared_ptr<physics_object::object> o;
	if (camera_properties_.camera_target_search_direction) o = get_physics_object_from_vector(camera_properties_.camera_target_name);
	else o = get_physics_object_from_vector_reversed(camera_properties_.camera_target_name);
	if (!o) return o;

	Eigen::Quaterniond vehicle_point_direction = o->physics_state.rotation;
    vector::worldspace vehicle_point_vector = vehicle_point_direction.conjugate() * vector::worldspace::UnitX();
    //Eigen::Quaterniond vehicle_travel_direction = Eigen::Quaterniond::FromTwoVectors(o->physics_state.velocity, vector::worldspace(1, 0, 0));
    Eigen::Quaterniond vehicle_travel_direction = Eigen::Quaterniond::Identity();
    vehicle_travel_direction = vehicle_travel_direction * Eigen::AngleAxisd(atan2(vehicle_point_vector.z(), sqrt(vehicle_point_vector.x()*vehicle_point_vector.x() + vehicle_point_vector.y()*vehicle_point_vector.y())), vector::worldspace::UnitY()); // pitch
    vehicle_travel_direction = vehicle_travel_direction * Eigen::AngleAxisd(-atan2(vehicle_point_vector.y(), vehicle_point_vector.x()), vector::worldspace::UnitZ()); // yaw
    /*
    std::vector<Eigen::Quaterniond> average_;
    average_.push_back(vehicle_point_direction);
    average_.push_back(vehicle_travel_direction);
    camera_properties_.previous_camera_rotations.push_back(average_approx(average_));
    */
    camera_properties_.previous_camera_rotations.push_back(vehicle_travel_direction);

	if (camera_properties_.previous_camera_rotations.size() > 30) {
		camera_properties_.previous_camera_rotations.erase(camera_properties_.previous_camera_rotations.begin());
	}
	Eigen::Quaterniond averaged_camera_rotation = average_approx(camera_properties_.previous_camera_rotations);
	//Eigen::Quaterniond averagedCameraRotation = o->physics_state.rotation;
	camera_properties_.update(averaged_camera_rotation, o->physics_state.position, o->physics_state.velocity);
    return o;
}

vector::worldspace get_triangle_normal(int i, mesh& model) {
	vector::worldspace v1 = vector::worldspace(model.vertices[model.indices[i+0]].x, model.vertices[model.indices[i+0]].y, model.vertices[model.indices[i+0]].z);
	vector::worldspace v2 = vector::worldspace(model.vertices[model.indices[i+1]].x, model.vertices[model.indices[i+1]].y, model.vertices[model.indices[i+1]].z);
	vector::worldspace v3 = vector::worldspace(model.vertices[model.indices[i+2]].x, model.vertices[model.indices[i+2]].y, model.vertices[model.indices[i+2]].z);
	vector::worldspace a = v2 - v1;
	vector::worldspace b = v3 - v1;
	vector::worldspace normal;
	normal.x() = a.y() * b.z() - a.z() * b.y();
	normal.y() = a.z() * b.x() - a.x() * b.z();
	normal.z() = a.x() * b.y() - a.y() * b.x();
	normal /= normal.norm();
	return normal;
}

double three_point_interpolate(double value_at_neg1, double value_at0, double value_at_pos1, double x) {
	if (x > 0) {
		return (1-x)*value_at0 + x*value_at_pos1;
	} else {
		return (1+x)*value_at0 - x*value_at_neg1;
	}
}

double linear_interpolate(double value_at0, double value_at1, double x) {
	return (1-x)*value_at0 + x*value_at1;
}

double smoothstep(double edge0, double edge1, double x) {
		// scale, bias and saturate x to 0..1 range
		x = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
		// evaluate polynomial
		return x * x * (3 - 2 * x);
}

void set_vertex_colors_by_brightness(vertex& v, double brightness_unfiltered) {
    
    /*
	double brightness = smoothstep(-1, 1, 0.8*brightness_unfiltered)*2-1;
	v.r = (1-v.sun_factor)*v.r + 2 * v.sun_factor*v.r*three_point_interpolate(0.5, 1, 1.5, brightness);
	v.g = (1-v.sun_factor)*v.g + 2 * v.sun_factor*v.g*three_point_interpolate(0.6, 1, 1.4, brightness);
	v.b = (1-v.sun_factor)*v.b + 2 * v.sun_factor*v.b*three_point_interpolate(0.8, 1, 1.3, brightness);
    */
    
    ///*
    double brightness = three_point_interpolate(0.25, 0.4, 1, brightness_unfiltered); // ground albedo is about 0.3
    v.r = (1-v.sun_factor)*v.r + v.sun_factor*v.r*brightness*1.5;
    v.g = (1-v.sun_factor)*v.g + v.sun_factor*v.g*brightness*1.4;
    v.b = (1-v.sun_factor)*v.b + v.sun_factor*v.b*brightness*1.3;
    //*/
}

void apply_sunlight_to_model(mesh& model) {
	for (int i = 0; i < model.indices.size(); i += 3) {
		vector::worldspace normal = get_triangle_normal(i, model);
		double brightness = normal.dot(globals::SUN_DIRECTION);
		set_vertex_colors_by_brightness(model.vertices[model.indices[i+0]], brightness);
		set_vertex_colors_by_brightness(model.vertices[model.indices[i+1]], brightness);
		set_vertex_colors_by_brightness(model.vertices[model.indices[i+2]], brightness);
	}
}

void recalculate_ground(bool& new_ground_ready_, std::vector<mesh>& ground_, vector::worldspace last_camera_position, vector::worldspace last_camera_target_velocity) {
	ground_ = get_terrain_model(false, last_camera_position + 0.5*last_camera_target_velocity);
	for (mesh& m : ground_) {
		apply_sunlight_to_model(m);
	}
	new_ground_ready_ = true;
}

int ground_models_start_point = -1;

void renderer::create_ground_models(std::vector<mesh>& models, camera_properties& camera_properties_, std::vector<mesh>& ground, bool& new_ground_ready) {
    if (ground_models_start_point == -1) {
		models.clear();
		auto models_ = get_terrain_model(false, camera_properties_.last_camera_position + 1.5*camera_properties_.last_camera_target_velocity);
		for (int i = 0; i < models_.size(); i++) {
			models.push_back(models_[i]);
		}
		ground_models_start_point = models_.size();
		new_ground_ready = true;
	}

	if (new_ground_ready) {
		//auto time_initial = std::chrono::high_resolution_clock::now();
		models.clear();
		for (int i = 0; i < ground.size(); i++) {
			models.push_back(ground[i]);
		}
		std::thread t(recalculate_ground, std::ref(new_ground_ready), std::ref(ground), camera_properties_.last_camera_position, camera_properties_.last_camera_target_velocity);
		t.detach();
		//auto time_taken = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_initial);
		//std::cout << "time taken (new_ground_ready swap): " << time_taken << std::endl;
		new_ground_ready = false;
	}

	//std::cout << "ground_models_start_point: " << ground_models_start_point << "\t";
	//std::cout << "models.size() before reduction: " << models.size() << "\t";

    // clear models, but keep around the part of the vector that the ground recalculation will use
	while (models.size() > ground_models_start_point) {
		models.pop_back();
	}

	/*std::cout << "models.size() after reduction: " << models.size() << "\t";
	long total_vertices = 0;
	for (mesh& m : models) {
		total_vertices += m.indices.size();
	}
	std::cout << "total_vertices: " << total_vertices << "\n";
    */
}

void renderer::create_models_from_physics_objects(std::vector<mesh>& models, camera_properties& camera_properties_, bool& new_ground_ready) {

    globals::physics_objects_mutex.lock();    
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();
    
	std::shared_ptr<physics_object::object> camera_tracked_object = camera_track_physics_object(camera_properties_);

    //if (camera_tracked_object && camera_tracked_object->mutex) camera_tracked_object->mutex->lock();
    //for (auto o : physics_objects_) if (o->mutex) o->mutex->lock();

	std::vector<module::visual_model> original_models;
	std::vector<vector::worldspace> original_positions;
	std::vector<Eigen::Quaterniond> original_rotations;

	for (auto o : physics_objects_) {
        if (o->mutex) o->mutex->lock();
		for (std::shared_ptr<module::module>& module_ : o->properties.modules) {
            if (!module_) {
                std::cout << "void renderer::create_models_from_physics_objects(...): std::shared_ptr<module::module> is a nullptr\n";
                std::cout << "deleting module from object " << o << " (named \"" << o->properties.name << "\")\n";
                o->properties.modules.erase(std::find(o->properties.modules.begin(), o->properties.modules.end(), module_));
                continue;
            }
            for (module::visual_model& m : module_->models) {
				original_models.push_back(m);
				original_positions.push_back(o->physics_state.position);
				original_rotations.push_back(o->physics_state.rotation);
			}
            /*
            if (module_->collider.type != collision::collider_type::model_collider) continue;
            mesh m = mesh(module_->collider, 0.8, 0, 0, 0.8);
            for (vertex& v : m.vertices) {
                double x_ = v.x;
			    double y_ = v.y;
			    double z_ = v.z;
			    v.x = y_;
			    v.y = z_;
			    v.z = -x_;
            }
            apply_sunlight_to_model(m);
		    models.push_back(m);
            */
		}
        if (o->mutex) o->mutex->unlock();
	}

    //if (camera_tracked_object && camera_tracked_object->mutex) camera_tracked_object->mutex->unlock();
    //for (auto o : physics_objects_) if (o->mutex) o->mutex->unlock();
    
	for (int i = 0; i < original_models.size(); i++) {
		module::visual_model& m = original_models[i];
		mesh model_ = *(m.mesh_data);
		mesh model_rotated = model_;
		for (vertex& v : model_rotated.vertices) {
			v.x *= m.scaling.x();
			v.y *= m.scaling.y();
			v.z *= m.scaling.z();
		}
		for (vertex& v : model_rotated.vertices) {
			v.x += m.position.x();
			v.y += m.position.y();
			v.z += m.position.z();
		}
		for (vertex& v : model_rotated.vertices) {
			vector::localspace v_unrotated = vector::localspace(v.x, v.y, v.z);
			vector::worldspace v_rotated = v_unrotated.to_worldspace(original_rotations[i]);
			v.x = v_rotated.x();
			v.y = v_rotated.y();
			v.z = v_rotated.z();
		}
		for (vertex& v : model_rotated.vertices) {
			v.x += original_positions[i].x();
			v.y += original_positions[i].y();
			v.z += original_positions[i].z();
		}
		for (vertex& v : model_rotated.vertices) {
			double x_ = v.x;
			double y_ = v.y;
			double z_ = v.z;
			v.x = y_;
			v.y = z_;
			v.z = -x_;
		}
		apply_sunlight_to_model(model_rotated);
		models.push_back(model_rotated);
	}


}
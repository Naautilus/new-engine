#pragma once
#include "../collision/logic.cpp"
#include "../simulation_state.cpp"
#include "../physics_object/logger.cpp"

void step_physics_object_controls() {
    for (physics_object::object& o : globals::physics_objects) {
        for (controls::input& c : o.control_bindings.inputs) {
            c.response_unmultiplied = std::clamp(c.response_unmultiplied, c.minimum, c.maximum);
            c.response_multiplied = c.response_unmultiplied * c.inherent_multiplier * c.get_control_multiplier_for_health_fraction(o.physics_state.health / o.physics_state.mass);
        }
    }
}

void step_physics_object_lifespans() {
    for (physics_object::object& o : globals::physics_objects) {
        if (o.properties.ticks_lifetime_remaining > 0) o.properties.ticks_lifetime_remaining--;
        if (o.properties.ticks_lifetime_remaining == 0) o.physics_state.health = 0;
    }
}

void step_physics_object_collisions() {
    std::vector<int> functional_physics_object_indices;
    for (int i = 0; i < globals::physics_objects.size(); i++) {
        if (globals::physics_objects[i].properties.functional) functional_physics_object_indices.push_back(i);
    }
    for (int i_ = 0; i_ < functional_physics_object_indices.size(); i_++) {
        for (int j_ = 0; j_ < i_; j_++) {
            int i = functional_physics_object_indices[i_];
            int j = functional_physics_object_indices[j_];
            if ((globals::physics_objects[i].physics_state.position - globals::physics_objects[j].physics_state.position).squaredNorm() > 100*100) continue;
            for (std::shared_ptr<module::module>& i_module : globals::physics_objects[i].properties.modules) {
                for (std::shared_ptr<module::module>& j_module : globals::physics_objects[j].properties.modules) {
                    if (i_module->collider.check_collision(j_module->collider)) {
                        collision::process_colliding_physics_objects(i_module->collider, j_module->collider, globals::physics_objects[i], globals::physics_objects[j]);
                    }
                }
            }
        }
    }
}

void step_physics_object_ground_collisions() {
    for (int i = 0; i < globals::physics_objects.size(); i++) {
        if (globals::physics_objects[i].properties.fixed) continue;
        collision::process_ground_collision(globals::physics_objects[i]);
    }
}

void step_physics_object_deletion() {
    for (int i = globals::physics_objects.size() - 1; i >= 0; i--) {
        if (globals::physics_objects[i].physics_state.health <= 0) {
            globals::physics_objects.erase(globals::physics_objects.begin() + i);
        }
    }
}

void step_physics_object_movement_and_modules() {
    for (physics_object::object& o : globals::physics_objects) {
        o.update_modules();
        if (o.properties.fixed) continue;
        o.calculate_acceleration();
        o.apply_queued_forces();
        o.physics_state.velocity += constants::DELTA_T * vector::worldspace(0, 0, -globals::current_simulation_state->gravity);
        o.physics_state.position += constants::DELTA_T * o.physics_state.velocity;
        Eigen::Quaterniond angular_velocity_quaternion = 
        Eigen::AngleAxisd(constants::DELTA_T * o.physics_state.angular_velocity.x(), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(constants::DELTA_T * o.physics_state.angular_velocity.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(constants::DELTA_T * o.physics_state.angular_velocity.z(), Eigen::Vector3d::UnitZ());
        o.physics_state.rotation *= angular_velocity_quaternion;
    }
}

void step_physics_objects() {
    step_physics_object_controls();
    step_physics_object_lifespans();
    step_physics_object_collisions();
    step_physics_object_ground_collisions();
    step_physics_object_deletion();
    step_physics_object_movement_and_modules();
    globals::tick++;
}

void log_physics_objects(std::vector<physics_object::object>& p) {
	/*
	for (physics_object::object& o : p) {
		if (o.properties.name == "") continue;
		if (o.properties.name == "bullet") continue;
		o.log();
	}
	std::cout << "globals::physics_objects size: " << globals::physics_objects.size() << std::endl;
	for (physics_object::object& o : p) {
		//std::cout << "modules: " << o.properties.modules.size() << std::endl;
	}
	std::cout << std::endl;
	*/
}

int log_counter = 0;
int log_counter_interval = 100;

void wait_delta_t() {
	globals::last_time += std::chrono::nanoseconds(static_cast<long>(constants::DELTA_T * 1e9 / globals::current_simulation_state->timescale));
	auto now = std::chrono::high_resolution_clock::now();
	if (globals::last_time > now) {
		while (globals::last_time > now) now = std::chrono::high_resolution_clock::now();
	} else {
		if (log_counter == 0) {
			std::cout << "OUT OF TIME!";
			std::cout << " Time over: ";
			std::cout << (100*std::chrono::duration_cast<std::chrono::nanoseconds>(now - globals::last_time).count() / (1e9 * constants::DELTA_T));
			std::cout << " %\n";
		}
		globals::last_time = now;
	}
	log_counter++;
	log_counter %= log_counter_interval;
}

physics_object::object* get_physics_object_from_vector(std::vector<physics_object::object>& p, std::string name_) {
	for (int i = 0; i < p.size(); i++) {
		if (p[i].properties.name == name_) return &p[i];
	}
	return nullptr;
}

physics_object::object* get_physics_object_from_vector_reversed(std::vector<physics_object::object>& p, std::string name_) {
	for (int i = p.size() - 1; i >= 0; i--) {
		if (p[i].properties.name == name_) return &p[i];
	}
	return nullptr;
}
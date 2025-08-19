#pragma once
#include "physics_step_logic.hpp"

void set_functional_physics_objects_list() {
    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();
    
    globals::functional_physics_objects_mutex.lock();
    globals::functional_physics_objects.clear();
    for (auto o : physics_objects_) {
        if (o->properties.functional) globals::functional_physics_objects.push_back(o);
    }
    globals::functional_physics_objects_mutex.unlock();
}

void step_physics_object_controls() {
    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();

    for (auto o : physics_objects_) {
        if (o->mutex) std::lock_guard<std::mutex> lock(*o->mutex);
        for (controls::input& c : o->control_bindings.inputs) {
            c.response_unmultiplied = std::clamp(c.response_unmultiplied, c.minimum, c.maximum);
            c.response_multiplied = c.response_unmultiplied * c.inherent_multiplier * c.get_control_multiplier_for_health_fraction(o->physics_state.health / o->physics_state.mass);
        }
    }
}

void step_physics_object_lifespans() {
    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();

    for (auto o : physics_objects_) {
        if (o->mutex) std::lock_guard<std::mutex> lock(*o->mutex);
        if (o->properties.ticks_lifetime_remaining > 0) o->properties.ticks_lifetime_remaining--;
        if (o->properties.ticks_lifetime_remaining == 0) o->physics_state.health = 0;
    }
}

void step_physics_object_collisions() {
    globals::functional_physics_objects_mutex.lock();
    auto functional_physics_objects_ = globals::functional_physics_objects;
    globals::functional_physics_objects_mutex.unlock();

    for (int i_ = 0; i_ < functional_physics_objects_.size(); i_++) {
        for (int j_ = 0; j_ < i_; j_++) {
            auto i = functional_physics_objects_[i_];
            auto j = functional_physics_objects_[j_];
            if ((i->physics_state.position - j->physics_state.position).squaredNorm() > 100*100) continue;
            for (std::shared_ptr<module::module>& i_module : i->properties.modules) {
                for (std::shared_ptr<module::module>& j_module : j->properties.modules) {
                    if (i_module->collider.check_collision(j_module->collider)) {
                        if (i->mutex) std::lock_guard<std::mutex> lock_i(*i->mutex);
                        if (j->mutex) std::lock_guard<std::mutex> lock_j(*j->mutex);
                        collision::process_colliding_physics_objects(i_module->collider, j_module->collider, *i, *j);
                    }
                }
            }
        }
    }
}

void step_physics_object_ground_collisions() {
    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();

    for (int i = 0; i < physics_objects_.size(); i++) {
        if (physics_objects_[i]->properties.fixed) continue;
        //if (!physics_objects_[i].properties.functional) continue;
        collision::process_ground_collision(*physics_objects_[i]);
    }
}

void step_physics_object_deletion() {
    globals::physics_objects_mutex.lock();
    for (int i = globals::physics_objects.size() - 1; i >= 0; i--) {
        if (globals::physics_objects[i]->physics_state.health <= 0) {
            globals::physics_objects.erase(globals::physics_objects.begin() + i);
        }
    }
    globals::physics_objects_mutex.unlock();
}

void step_physics_object_movement_and_modules() {
    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();

    for (auto o : physics_objects_) {
        if (o->mutex) std::lock_guard<std::mutex> lock(*o->mutex);
        o->update_modules();
        if (o->properties.fixed) continue;
        o->calculate_acceleration();
        o->apply_queued_forces();
        o->physics_state.velocity += constants::DELTA_T * vector::worldspace(0, 0, -globals::current_simulation_state->gravity);
        o->physics_state.position += constants::DELTA_T * o->physics_state.velocity;
        Eigen::Quaterniond angular_velocity_quaternion = 
        Eigen::AngleAxisd(constants::DELTA_T * o->physics_state.angular_velocity.x(), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(constants::DELTA_T * o->physics_state.angular_velocity.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(constants::DELTA_T * o->physics_state.angular_velocity.z(), Eigen::Vector3d::UnitZ());
        o->physics_state.rotation *= angular_velocity_quaternion;
    }
}

void step_physics_objects() {

    set_functional_physics_objects_list();
    
    //globals::physics_objects_mutex.lock();
    step_physics_object_controls();
    //globals::physics_objects_mutex.unlock();

    //globals::physics_objects_mutex.lock();
    step_physics_object_lifespans();
    //globals::physics_objects_mutex.unlock();

    //globals::physics_objects_mutex.lock();
    step_physics_object_collisions();
    //globals::physics_objects_mutex.unlock();

    //globals::physics_objects_mutex.lock();
    step_physics_object_ground_collisions();
    //globals::physics_objects_mutex.unlock();

    //globals::physics_objects_mutex.lock();
    step_physics_object_deletion();
    //globals::physics_objects_mutex.unlock();

    //globals::physics_objects_mutex.lock();
    step_physics_object_movement_and_modules();
    //globals::physics_objects_mutex.unlock();

    globals::tick++;
}

void log_physics_objects() {
    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();

	//std::cout << "-----------------\n";
	for (auto o : physics_objects_) {
        //if (o->mutex) std::lock_guard<std::mutex> lock(*o->mutex);
		//o->log();
        //std::cout << o << "\n";
	}
	
}

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

    globals::pause_mutex.lock();
    globals::pause_mutex.unlock();
    
}

std::shared_ptr<physics_object::object> get_physics_object_from_vector(std::string name_) {
    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();

	for (int i = 0; i < physics_objects_.size(); i++) {
		if (physics_objects_[i]->properties.name == name_) return physics_objects_[i];
	}
	return nullptr;
}

std::shared_ptr<physics_object::object> get_physics_object_from_vector_reversed(std::string name_) {
    globals::physics_objects_mutex.lock();
    auto physics_objects_ = globals::physics_objects;
    globals::physics_objects_mutex.unlock();

	for (int i = physics_objects_.size() - 1; i >= 0; i--) {
		if (physics_objects_[i]->properties.name == name_) return physics_objects_[i];
	}
	return nullptr;
}
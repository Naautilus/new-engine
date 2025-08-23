#pragma once
#include "../collision/logic.hpp"
#include "../simulation_state.hpp"
#include "../physics_object/logger.hpp"

void step_physics_objects();

void log_physics_objects();

void wait_delta_t();

std::shared_ptr<physics_object::object> get_physics_object_from_vector(std::string name_);
std::shared_ptr<physics_object::object> get_physics_object_from_vector_reversed(std::string name_);
#pragma once
#include "../collision/logic.cpp"
#include "../simulation_state.cpp"
#include "../physics_object/logger.cpp"

void set_functional_physics_objects_list();
void step_physics_object_controls();
void step_physics_object_lifespans();
void step_physics_object_collisions();
void step_physics_object_ground_collisions();
void step_physics_object_deletion();
void step_physics_object_movement_and_modules();
void step_physics_objects();

int log_counter = 0;
int log_counter_interval = 100;
void log_physics_objects();

void wait_delta_t();

std::shared_ptr<physics_object::object> get_physics_object_from_vector(std::string name_);
std::shared_ptr<physics_object::object> get_physics_object_from_vector_reversed(std::string name_);
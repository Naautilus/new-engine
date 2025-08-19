#pragma once
#include "../constants/constants.hpp"
#include "../vector/vector_spaces.hpp"
#include "../timer/timer.h"
#include <mutex>

namespace globals {

auto last_time;

bool free_camera;
bool paused;
std::mutex pause_mutex;

std::vector<std::shared_ptr<physics_object::object>> physics_objects;
std::mutex physics_objects_mutex;

/* a subset of physics_objects with only functional physics objects
to avoid recalculating this for things that only affect functional
physics objects */
std::vector<std::shared_ptr<physics_object::object>> functional_physics_objects;
std::mutex functional_physics_objects_mutex;

std::default_random_engine rng;
std::unique_ptr<simulation_state> current_simulation_state;
vector::worldspace SUN_DIRECTION;
int tick;
int error_count;

timer::timer timer_;

}
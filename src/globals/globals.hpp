#pragma once
#include "../constants/constants.hpp"
#include "../vector/vector_spaces.hpp"
#include "../timer/timer.hpp"
#include "../simulation_state.hpp"
#include <mutex>

namespace globals {

extern std::chrono::time_point<std::chrono::high_resolution_clock> last_time;

extern bool free_camera;
extern bool paused;
extern std::mutex pause_mutex;

extern std::vector<std::shared_ptr<physics_object::object>> physics_objects;
extern std::mutex physics_objects_mutex;

/* a subset of physics_objects with only functional physics objects
to avoid recalculating this for things that only affect functional
physics objects */
extern std::vector<std::shared_ptr<physics_object::object>> functional_physics_objects;
extern std::mutex functional_physics_objects_mutex;

extern std::default_random_engine rng;
extern std::unique_ptr<simulation_state> current_simulation_state;
extern vector::worldspace SUN_DIRECTION;
extern int tick;
extern int error_count;

extern timer::timer timer_;

}
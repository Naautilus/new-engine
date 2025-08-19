// top of cpp marker
#include "globals.hpp"

namespace globals {

std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();

bool free_camera;
bool paused;
std::mutex pause_mutex;

std::vector<std::shared_ptr<physics_object::object>> physics_objects;
std::mutex physics_objects_mutex;

std::vector<std::shared_ptr<physics_object::object>> functional_physics_objects;
std::mutex functional_physics_objects_mutex;

std::default_random_engine rng;
std::unique_ptr<simulation_state> current_simulation_state;
vector::worldspace SUN_DIRECTION = vector::worldspace(-1, -1, -1) / vector::worldspace(-1, -1, -1).norm();
int tick = 0;
int error_count = 0;

timer::timer timer_ = timer::timer("timer", timer::timer::NANOSECONDS);

}
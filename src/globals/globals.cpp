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
vector::worldspace SUN_DIRECTION = vector::worldspace(1, 1, -1) / vector::worldspace(1, 1, 1).norm();
int tick = 0;
int error_count = 0;

timer::timer timer_ = timer::timer("timer", timer::timer::NANOSECONDS);

std::vector<std::vector<double>> sensor_ir_activations;
std::mutex sensor_ir_activations_mutex;

bool MISSILE_CAMERA = false;
bool SHOW_COLLISION_DEBUGGING = false;
bool PAUSE_ON_COLLISION = false;
bool SHOW_MISSILE_ACCELERATION = false;
bool VERBOSE_LOG = false;
bool SHOW_FPS = false;

const color SKY_COLOR = color{0.08, 0.45, 1.44};
double GRAVITY = 9.81;
double TIMESCALE = 1.0;

}
#pragma once
#include "../constants/constants.cpp"
#include "../vector/vector_spaces.cpp"
#include <mutex>

namespace globals {
    auto last_time = std::chrono::high_resolution_clock::now();
    std::vector<physics_object::object> physics_objects;
    std::mutex physics_objects_mutex;
    std::default_random_engine rng;
    std::unique_ptr<simulation_state> current_simulation_state;
    vector::worldspace SUN_DIRECTION = vector::worldspace(-1, -1, -1) / vector::worldspace(-1, -1, -1).norm();
    int tick = 0;
    int error_count = 0;
}
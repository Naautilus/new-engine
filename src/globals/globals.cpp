#pragma once
#include "../constants/constants.cpp"
#include "../vector/vector_spaces.cpp"
#include "../timer/timer.h"
#include <mutex>

namespace globals {
    auto last_time = std::chrono::high_resolution_clock::now();

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
    vector::worldspace SUN_DIRECTION = vector::worldspace(-1, -1, -1) / vector::worldspace(-1, -1, -1).norm();
    int tick = 0;
    int error_count = 0;

    timer::timer timer_("timer", timer::timer::NANOSECONDS);

}
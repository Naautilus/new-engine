#pragma once
#include "globals.hpp"

namespace globals {
    
last_time = std::chrono::high_resolution_clock::now();
vector::worldspace SUN_DIRECTION = vector::worldspace(-1, -1, -1) / vector::worldspace(-1, -1, -1).norm();
tick = 0;
error_count = 0;
timer_ = timer::timer("timer", timer::timer::NANOSECONDS);

}
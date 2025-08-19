#include "globals.hpp"

auto last_time = std::chrono::high_resolution_clock::now();
vector::worldspace SUN_DIRECTION = vector::worldspace(-1, -1, -1) / vector::worldspace(-1, -1, -1).norm();
int tick = 0;
int error_count = 0;
auto timer_ = timer::timer("timer", timer::timer::NANOSECONDS);
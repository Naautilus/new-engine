// top of cpp marker
#include "globals.hpp"

std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
vector::worldspace SUN_DIRECTION = vector::worldspace(-1, -1, -1) / vector::worldspace(-1, -1, -1).norm();
int tick = 0;
int error_count = 0;
timer::timer timer_ = timer::timer("timer", timer::timer::NANOSECONDS);
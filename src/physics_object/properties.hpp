#pragma once
#include "physics_state.hpp"
#include "../module/module.hpp"

struct properties {
	std::string name;
	std::vector<std::shared_ptr<module::module>> modules;
	std::vector<std::pair<vector::worldspace, vector::worldspace>> force_queue;
	extern int ticks_lifetime_remaining;
	extern bool fixed;
	extern bool functional;
};
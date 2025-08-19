#pragma once
#include "physics_state.hpp"
#include "../module/module.hpp"

struct properties {
	std::string name;
	std::vector<std::shared_ptr<module::module>> modules;
	std::vector<std::pair<vector::worldspace, vector::worldspace>> force_queue;
	int ticks_lifetime_remaining = -1;
	bool fixed = false;
	bool functional = true;
};
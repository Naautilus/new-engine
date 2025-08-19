#pragma once
#include "../physics_object/blueprints.cpp"
#include <nlohmann/json.hpp>
#include <fstream>
using json = nlohmann::json;

// SOURCES
// 1: https://www.researchgate.net/figure/Geometrical-mass-and-inertial-properties-of-the-F-16_tbl1_268169601

void initialize_physics_objects(std::vector<std::string> args);
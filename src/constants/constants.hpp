#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <format>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <functional>
#include <numeric>
#include <random>
#include <fstream>
#include <functional>
#include <numbers>

namespace vector {
    
struct worldspace;
struct localspace;

}

namespace physics_object {
    
struct object;

}

struct simulation_state;

namespace constants {

const std::string FORMAT_STRING_POSITION = "{:10.1f}";
const std::string FORMAT_STRING_VELOCITY = "{:10.1f}";
const std::string FORMAT_STRING_ROTATION = "{:4.0f}";
const std::string FORMAT_STRING_UNIT = "{:5.2f}";
const std::string FORMAT_STRING_ANGULAR_VELOCITY = "{:6.1f}";
const std::string FORMAT_STRING_SPEED = "{:4.0f}";
const std::string FORMAT_STRING_G_FORCE = "{:6.2f}";
const std::string FORMAT_STRING_AOA = "{:5.1f}";
const std::string FORMAT_STRING_ALTITUDE = "{:5.0f}";
const double DELTA_T = 0.005;
const double LOG_INTERVAL = 0.005;
const double TIME_LIMIT = -1;
const double DAMAGE_MULTIPLIER = 2e-4;
const double UNCONTROLLABLE_HEALTH_FRACTION = 0.8;
const double SAFE_COLLISION_SPEED = 5;
const double CYLINDER_VERTICES = 8;
const double QUADRATIC_PLANET_CURVATURE_COEFFICIENT = 7.85e-8;// * 1e7; // the 1e7 is to make it extremely severe for testing ground collision mesh
const double STANDARD_GRAVITY = 9.81;

}
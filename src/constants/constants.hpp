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
#include <memory>
#include <optional>

namespace vector {
    
struct worldspace;
struct localspace;

}

namespace physics_object {
    
struct object;

}

struct simulation_state;

namespace constants {

extern constexpr std::string FORMAT_STRING_POSITION;
extern constexpr std::string FORMAT_STRING_VELOCITY;
extern constexpr std::string FORMAT_STRING_ROTATION;
extern constexpr std::string FORMAT_STRING_UNIT;
extern constexpr std::string FORMAT_STRING_ANGULAR_VELOCITY;
extern constexpr std::string FORMAT_STRING_SPEED;
extern constexpr std::string FORMAT_STRING_G_FORCE;
extern constexpr std::string FORMAT_STRING_AOA;
extern constexpr std::string FORMAT_STRING_ALTITUDE;
extern const double DELTA_T;
extern const double LOG_INTERVAL;
extern const double TIME_LIMIT;
extern const double DAMAGE_MULTIPLIER;
extern const double UNCONTROLLABLE_HEALTH_FRACTION;
extern const double SAFE_COLLISION_SPEED;
extern const double CYLINDER_VERTICES;
extern const double QUADRATIC_PLANET_CURVATURE_COEFFICIENT;
extern const double STANDARD_GRAVITY;

}
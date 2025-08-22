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

constexpr std::string FORMAT_STRING_POSITION = "{:10.1f}";
constexpr std::string FORMAT_STRING_VELOCITY = "{:10.1f}";
constexpr std::string FORMAT_STRING_ROTATION = "{:4.0f}";
constexpr std::string FORMAT_STRING_UNIT = "{:5.2f}";
constexpr std::string FORMAT_STRING_ANGULAR_VELOCITY = "{:6.1f}";
constexpr std::string FORMAT_STRING_SPEED = "{:4.0f}";
constexpr std::string FORMAT_STRING_G_FORCE = "{:6.2f}";
constexpr std::string FORMAT_STRING_AOA = "{:5.1f}";
constexpr std::string FORMAT_STRING_ALTITUDE = "{:5.0f}";
extern const double DELTA_T;
extern const double LOG_INTERVAL;
extern const double TIME_LIMIT;
extern const double DAMAGE_MULTIPLIER;
extern const double UNCONTROLLABLE_HEALTH_FRACTION;
extern const double SAFE_COLLISION_SPEED;
extern const double CYLINDER_VERTICES;
extern const double QUADRATIC_PLANET_CURVATURE_COEFFICIENT;
extern const double STANDARD_GRAVITY;
extern const double WATER_LEVEL;
extern const double AIR_DENSITY;
extern const double AIR_DENSITY_1_OVER_E_FALLOFF_DISTANCE;
extern const double WATER_DENSITY;

}
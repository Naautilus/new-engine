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

    constexpr std::string FORMAT_STRING_POSITION;
    constexpr std::string FORMAT_STRING_VELOCITY;
    constexpr std::string FORMAT_STRING_ROTATION;
    constexpr std::string FORMAT_STRING_UNIT;
    constexpr std::string FORMAT_STRING_ANGULAR_VELOCITY;
    constexpr std::string FORMAT_STRING_SPEED;
    constexpr std::string FORMAT_STRING_G_FORCE;
    constexpr std::string FORMAT_STRING_AOA;
    constexpr std::string FORMAT_STRING_ALTITUDE;
    const double DELTA_T;
    const double LOG_INTERVAL;
    const double TIME_LIMIT;
    const double DAMAGE_MULTIPLIER;
    const double UNCONTROLLABLE_HEALTH_FRACTION;
    const double SAFE_COLLISION_SPEED;
    const double CYLINDER_VERTICES;
    const double QUADRATIC_PLANET_CURVATURE_COEFFICIENT;
    const double STANDARD_GRAVITY;

}
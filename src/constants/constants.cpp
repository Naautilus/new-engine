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

namespace constants {

    FORMAT_STRING_POSITION = "{:10.1f}";
    FORMAT_STRING_VELOCITY = "{:10.1f}";
    FORMAT_STRING_ROTATION = "{:4.0f}";
    FORMAT_STRING_UNIT = "{:5.2f}";
    FORMAT_STRING_ANGULAR_VELOCITY = "{:6.1f}";
    FORMAT_STRING_SPEED = "{:4.0f}";
    FORMAT_STRING_G_FORCE = "{:6.2f}";
    FORMAT_STRING_AOA = "{:5.1f}";
    FORMAT_STRING_ALTITUDE = "{:5.0f}";
    DELTA_T = 0.005;
    LOG_INTERVAL = 0.005;
    TIME_LIMIT = -1;
    DAMAGE_MULTIPLIER = 2e-4;
    UNCONTROLLABLE_HEALTH_FRACTION = 0.8;
    SAFE_COLLISION_SPEED = 5;
    CYLINDER_VERTICES = 8;
    QUADRATIC_PLANET_CURVATURE_COEFFICIENT = 7.85e-8;// * 1e7; // the 1e7 is to make it extremely severe for testing ground collision mesh
    STANDARD_GRAVITY = 9.81;

}
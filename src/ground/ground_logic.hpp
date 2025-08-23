#pragma once
#include "PerlinNoise.hpp"
#include "../constants/constants.hpp"
#include "../globals/globals.hpp"
#include "../renderer/color.hpp"
#include <mutex>

namespace ground {

double get_ground_altitude(double x, double y);
color get_ground_color(double x, double y);

double get_ground_altitude_averaged(double x, double y, double width, int count);
color get_ground_color_averaged(double x, double y, double width, int count);

vector::worldspace get_surface_normal(double x, double y, double epsilon);
vector::worldspace get_surface_normal(double x, double y);

bool line_of_sight(vector::worldspace a, vector::worldspace& b);

double fluid_density(double altitude);

}
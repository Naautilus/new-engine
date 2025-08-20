#pragma once
#include "PerlinNoise.hpp"
#include "../constants/constants.hpp"
#include "../globals/globals.hpp"
#include "../renderer/color.hpp"
#include <mutex>

namespace ground {

struct ground_info {
    double x, y, width;
    int count;
    bool operator==(const ground_info& g) const;
};

struct ground_info_hash {
    std::size_t operator()(const ground_info& g) const;
};

extern const siv::PerlinNoise::seed_type seed;
const siv::PerlinNoise perlin{ seed };
extern std::vector<double> PERLIN_WIDTH;
extern std::vector<double> PERLIN_HEIGHT_EFFECT;
extern std::vector<double> PERLIN_COLOR_EFFECT;
extern std::unordered_map<ground_info, double, ground_info_hash> ground_altitude_averaged;
extern std::unordered_map<ground_info, color, ground_info_hash> ground_color_averaged;
extern std::mutex ground_altitude_averaged_mutex;
extern std::mutex ground_color_averaged_mutex;
extern double WATER_LEVEL;

double get_ground_altitude(double x, double y);
color get_ground_color(double x, double y);

double get_ground_altitude_averaged(double x, double y, double width, int count);
color get_ground_color_averaged(double x, double y, double width, int count);

vector::worldspace get_surface_normal(double x, double y, double epsilon);
vector::worldspace get_surface_normal(double x, double y);

bool line_of_sight(vector::worldspace a, vector::worldspace& b);

}
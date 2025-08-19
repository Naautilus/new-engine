#pragma once
#include "PerlinNoise.hpp"
#include "../constants/constants.cpp"
#include "../globals/globals.cpp"
#include "../renderer/color.h"
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

    const siv::PerlinNoise::seed_type seed = 123456u;
    const siv::PerlinNoise perlin{ seed };
    std::vector<double> PERLIN_WIDTH;
    std::vector<double> PERLIN_HEIGHT_EFFECT;
    std::vector<double> PERLIN_COLOR_EFFECT;
    std::unordered_map<ground_info, double, ground_info_hash> ground_altitude_averaged;
    std::unordered_map<ground_info, color, ground_info_hash> ground_color_averaged;
    std::mutex ground_altitude_averaged_mutex;
    std::mutex ground_color_averaged_mutex;

    double get_ground_altitude(double x, double y);
    color get_ground_color(double x, double y);

    double get_ground_altitude_averaged(double x, double y, double width, int count);
    color get_ground_color_averaged(double x, double y, double width, int count);

    vector::worldspace get_surface_normal(double x, double y, double epsilon);
    vector::worldspace get_surface_normal(double x, double y);

    bool line_of_sight(vector::worldspace a, vector::worldspace& b);
}
#pragma once
#include "PerlinNoise.hpp"
#include "../constants/constants.cpp"
#include "../globals/globals.cpp"

namespace ground {

    struct ground_info {
        double x, y, width;
        int count;
        bool operator==(const ground_info& g) const {
            return
                (x == g.x) &&
                (y == g.y) &&
                (width == g.width) &&
                (count == g.count);
        }
    };

    struct ground_info_hash {
        std::size_t operator()(const ground_info& g) const {
            std::hash<double> hash_fn;
            std::size_t h1 = hash_fn(g.x);
            std::size_t h2 = hash_fn(g.y);
            std::size_t h3 = hash_fn(g.width);
            std::size_t h4 = hash_fn(g.count);
            std::size_t output = h1;
            output ^= h2 + 0x9e3779b9 + (output<<6) + (output>>2);
            output ^= h3 + 0x9e3779b9 + (output<<6) + (output>>2);
            output ^= h4 + 0x9e3779b9 + (output<<6) + (output>>2);
            return output;
        }
    };

    const siv::PerlinNoise::seed_type seed = 123456u;
    const siv::PerlinNoise perlin{ seed };
    std::vector<double> PERLIN_WIDTH =          {    25,  1000, 10000, 100000};
    std::vector<double> PERLIN_HEIGHT_EFFECT =  {     5,   100,  5000,   5000};
    //std::vector<double> PERLIN_HEIGHT_EFFECT =  {     0,      0,     0,     0};
    std::vector<double> PERLIN_COLOR_EFFECT =   { 0.025, 0.025,  0.15,      0};
    std::unordered_map<ground_info, double, ground_info_hash> ground_altitude;
    std::unordered_map<ground_info, double, ground_info_hash> ground_color;

    double get_ground_altitude(double x, double y) {
        double sum = 0;
        for (int i = 0; i < PERLIN_WIDTH.size(); i++) {
            double noise = perlin.octave2D_01((x / PERLIN_WIDTH[i]), (y / PERLIN_WIDTH[i]), 4);
            sum += noise * PERLIN_HEIGHT_EFFECT[i];
        }
        sum -= constants::QUADRATIC_PLANET_CURVATURE_COEFFICIENT * (x*x + y*y);
        return sum;
    }

    double get_ground_color(double x, double y) {
        double sum = 0;
        for (int i = 0; i < PERLIN_WIDTH.size(); i++) {
            double noise = perlin.octave2D_01((x / PERLIN_WIDTH[i]), (y / PERLIN_WIDTH[i]), 4);
            sum += noise * PERLIN_COLOR_EFFECT[i];
        }
        return sum;
    }

    double get_ground_altitude_averaged(double x, double y, double width, int count) {
        ground_info ground_info_(x, y, width, count);
        if(ground_altitude.find(ground_info_) != ground_altitude.end()) {
            return ground_altitude[ground_info_];
        }
        double sum = 0;
        for (int x_ = 0; x_ < count; x_++) {
            for (int y_ = 0; y_ < count; y_++) {
                sum += get_ground_altitude(x + x_*width/count, y + y_*width/count);
            }
        }
        sum /= (count * count);
        ground_altitude[ground_info_] = sum;
        return sum;
    }

    double get_ground_color_averaged(double x, double y, double width, int count) {
        ground_info ground_info_(x, y, width, count);
        if(ground_color.find(ground_info_) != ground_color.end()) {
            return ground_color[ground_info_];
        }
        double sum = 0;
        for (int x_ = 0; x_ < count; x_++) {
            for (int y_ = 0; y_ < count; y_++) {
                sum += get_ground_color(x + x_*width/count, y + y_*width/count);
            }
        }
        sum /= (count * count);
        ground_color[ground_info_] = sum;
        return sum;
    }

    vector::worldspace get_surface_normal(double x, double y) {
        double epsilon = 0.001;
        double m_x = -(get_ground_altitude(x + epsilon, y) - get_ground_altitude(x, y))/epsilon;
        double m_y = -(get_ground_altitude(x, y + epsilon) - get_ground_altitude(x, y))/epsilon;
        double z = 1;
        vector::worldspace normal = vector::worldspace(m_x, m_y, z);
        normal /= normal.norm();
        return normal;
    }

    bool line_of_sight(vector::worldspace a, vector::worldspace& b) {
        vector::worldspace position_difference = b - a;
        vector::worldspace traveler_probe_location;
        double max_distance_per_altitude = 1;
        double distance_total = position_difference.norm();
        double distance_traveled = 0;
        while (distance_traveled < distance_total) {
            double fraction_traveled = distance_traveled / distance_total;
            traveler_probe_location = (1-fraction_traveled) * a + fraction_traveled * b;
            double altitude = traveler_probe_location.z() - get_ground_altitude(traveler_probe_location.x(), traveler_probe_location.y());
            if (altitude < 5.0) return false;
            distance_traveled += altitude * max_distance_per_altitude;
        }
        return true;
    }
}
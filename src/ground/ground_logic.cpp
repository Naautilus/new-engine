// top of cpp marker
#include "ground_logic.hpp"
#include "../renderer/color.hpp"

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

const siv::PerlinNoise::seed_type seed = 1234567u;
const siv::PerlinNoise perlin{ seed };
std::vector<double> PERLIN_WIDTH =          {    25,  1000, 10000, 100000};
std::vector<double> PERLIN_HEIGHT_EFFECT =  {     5,   100,  5000,  20000};
std::vector<double> PERLIN_COLOR_EFFECT =   { 0.025, 0.025,     0,      0};
vector::worldspace TERRAIN_OFFSET(35000, -2000, -10000);
std::unordered_map<ground_info, double, ground_info_hash> ground_altitude_averaged;
std::unordered_map<ground_info, color, ground_info_hash> ground_color_averaged;
std::unordered_map<double, double> fluid_density_map;
const double FLUID_DENSITY_MAP_INTERVAL = 25;
std::mutex ground_altitude_averaged_mutex;
std::mutex ground_color_averaged_mutex;
std::mutex fluid_density_map_mutex;

// relative to water level
std::vector<std::pair<double, color>> ground_color_heightmap = {
    {    0, color{0.22, 0.12, 0.04}},
    {   49, color{0.22, 0.12, 0.04}},
    {   50, color{0.1, 0.1, 0.1}},
    {   51, color{0.0, 0.08, 0.0}},
    { 1500, color{0.0, 0.06, 0.0}},
    { 3500, color{0.1, 0.1, 0.1}},
    { 5000, color{0.1, 0.1, 0.1}},
    { 5100, color{1.0, 1.0, 1.0}}
};

color get_ground_color_from_heightmap(double z) {
    z -= constants::WATER_LEVEL;
    //std::cout << "z: " << z << "\n";

    /*
    std::cout << "heightmap:\n";
    for (auto& pair : ground_color_heightmap) {
        std::cout << pair.first << " -> {" << pair.second.r << ", " << pair.second.g << ", " << pair.second.b << "}\n";
    }
    */

    if (z < ground_color_heightmap[0].first) return ground_color_heightmap[0].second;
    int last = ground_color_heightmap.size() - 1;
    if (z > ground_color_heightmap[last].first) return ground_color_heightmap[last].second;

    int lower_index = 0;
    while (z > ground_color_heightmap[lower_index].first) lower_index++;
    lower_index--;

    double& lower = ground_color_heightmap[lower_index].first;
    double& higher = ground_color_heightmap[lower_index + 1].first;
    double fraction = (z - lower) / (higher - lower);

    //std::cout << "lower_index: " << lower_index << "\n";
    //std::cout << "fraction: " << fraction << "\n";

    color& lower_color = ground_color_heightmap[lower_index].second;
    color& higher_color = ground_color_heightmap[lower_index + 1].second;

    color output = lower_color * (1 - fraction) + higher_color * (fraction);
    //std::cout << "output: {" << output.r << ", " << output.g << ", " << output.b << "}\n";
    return output;
}

// NOTE: this function is too simple for hashing to help (~1437.5 ns with vs. ~637 ns without)
double get_ground_altitude(double x_, double y_) {
    double x = x_ + TERRAIN_OFFSET.x();
    double y = y_ + TERRAIN_OFFSET.y();
    double sum = 0;
    for (int i = 0; i < PERLIN_WIDTH.size(); i++) {
        double noise = perlin.octave2D_01((x / PERLIN_WIDTH[i]), (y / PERLIN_WIDTH[i]), 4);
        sum += noise * PERLIN_HEIGHT_EFFECT[i];
    }
    return sum + TERRAIN_OFFSET.z();
}

// NOTE: this function is too simple for hashing to help (~1437.5 ns with vs. ~637 ns without)
color get_ground_color(double x_, double y_) {
    double x = x_ + TERRAIN_OFFSET.x();
    double y = y_ + TERRAIN_OFFSET.y();
    float sum = 0;
    for (int i = 0; i < PERLIN_WIDTH.size(); i++) {
        float noise = (float)perlin.octave2D_01((x / PERLIN_WIDTH[i]), (y / PERLIN_WIDTH[i]), 4);
        sum += noise * PERLIN_COLOR_EFFECT[i];
    }
    return get_ground_color_from_heightmap(get_ground_altitude(x_, y_));// * (1 + sum);
}

double get_ground_altitude_averaged(double x, double y, double width, int count) {

    ///*
    ground_info ground_info_(x, y, width, count);
    ground_altitude_averaged_mutex.lock();
    if(ground_altitude_averaged.find(ground_info_) != ground_altitude_averaged.end()) {
        double output = ground_altitude_averaged[ground_info_];
        ground_altitude_averaged_mutex.unlock();
        return output;
    }
    ground_altitude_averaged_mutex.unlock();
    //*/
    
    double sum = 0;
    for (int x_ = 0; x_ < count; x_++) {
        for (int y_ = 0; y_ < count; y_++) {
            sum += get_ground_altitude(x + x_*width/count, y + y_*width/count);
        }
    }
    sum /= (count * count);

    ///*
    ground_altitude_averaged_mutex.lock();
    ground_altitude_averaged[ground_info_] = sum;
    ground_altitude_averaged_mutex.unlock();
    //*/
    return sum;
}

color get_ground_color_averaged(double x, double y, double width, int count) {

    ///*
    ground_info ground_info_(x, y, width, count);
    ground_color_averaged_mutex.lock();
    if(ground_color_averaged.find(ground_info_) != ground_color_averaged.end()) {
        color output = ground_color_averaged[ground_info_];
        ground_color_averaged_mutex.unlock();
        return output;
    }
    ground_color_averaged_mutex.unlock();
    //*/

    color sum;
    for (int x_ = 0; x_ < count; x_++) {
        for (int y_ = 0; y_ < count; y_++) {
            sum += get_ground_color(x + x_*width/count, y + y_*width/count);
        }
    }
    sum /= (count * count);

    ///*
    ground_color_averaged_mutex.lock();
    ground_color_averaged[ground_info_] = sum;
    ground_color_averaged_mutex.unlock();
    //*/
    return sum;
}

vector::worldspace get_surface_normal(double x, double y, double epsilon) {
    double m_x = -(get_ground_altitude(x + epsilon, y) - get_ground_altitude(x, y))/epsilon;
    double m_y = -(get_ground_altitude(x, y + epsilon) - get_ground_altitude(x, y))/epsilon;
    double z = 1;
    vector::worldspace normal = vector::worldspace(m_x, m_y, z);
    normal /= normal.norm();
    return normal;
}

vector::worldspace get_surface_normal(double x, double y) {
    return get_surface_normal(x, y, 0.001);
}

bool line_of_sight(vector::worldspace a, vector::worldspace& b) {
    //std::cout << a.x() << "," << a.y() << "," << a.z() << "\n";
    //std::cout << b.x() << "," << b.y() << "," << b.z() << "\n";
    vector::worldspace position_difference = b - a;
    vector::worldspace traveler_probe_location;
    double max_distance_per_altitude = 1;
    double distance_total = position_difference.norm();
    double distance_traveled = 0;
    while (distance_traveled < distance_total) {
        double fraction_traveled = distance_traveled / distance_total;
        traveler_probe_location = (1-fraction_traveled) * a + fraction_traveled * b;
        // seems fine: std::cout << traveler_probe_location.x() << "," << traveler_probe_location.y() << "," << traveler_probe_location.z() << "\n";
        // the error is here vvv
        double altitude = traveler_probe_location.z() - get_ground_altitude(traveler_probe_location.x(), traveler_probe_location.y());
        if (altitude < 5.0) return false;
        distance_traveled += altitude * max_distance_per_altitude;
    }
    return true;
}

double fluid_density(double altitude) {

    fluid_density_map_mutex.lock();
    int altitude_interval = floor(altitude / FLUID_DENSITY_MAP_INTERVAL);
    //std::cout << "altitude: " << altitude << "\n";
    //std::cout << "altitude_interval: " << altitude_interval << "\n";
    if(fluid_density_map.find(altitude_interval) != fluid_density_map.end()) {
        double output = fluid_density_map[altitude_interval];
        //std::cout << "hashed, density: " << output << "\n";
        fluid_density_map_mutex.unlock();
        return output;
    }
    fluid_density_map_mutex.unlock();

    double density;
    if (altitude > constants::WATER_LEVEL) {
        density = constants::AIR_DENSITY * exp(-altitude / constants::AIR_DENSITY_1_OVER_E_FALLOFF_DISTANCE);
    } else {
        density = constants::WATER_DENSITY;
    }

    fluid_density_map_mutex.lock();
    //std::cout << "not hashed, density: " << density << "\n";
    fluid_density_map[altitude_interval] = density;
    fluid_density_map_mutex.unlock();
    return density;
}

}
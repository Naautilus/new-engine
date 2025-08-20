#pragma once
#include "mesh.hpp"
#include "../constants/constants.hpp"

Eigen::Quaterniond random_quaternion_biased(double angle);

std::vector<vertex> random_isoceles_triangle(double offset_xy_, double offset_z_);

namespace models {

extern std::shared_ptr<mesh> pyramid;
extern std::shared_ptr<mesh> ground_color_varying;
extern std::shared_ptr<mesh> ground_color_uniform;
extern std::shared_ptr<mesh> jet;
extern std::shared_ptr<mesh> f16_old;
extern std::shared_ptr<mesh> cube;
extern std::shared_ptr<mesh> flame_trail;
extern std::shared_ptr<mesh> aim9x;
extern std::shared_ptr<mesh> bullet_octahedron;
extern std::shared_ptr<mesh> debris_1kg;
extern std::shared_ptr<mesh> debris_1kg_bright;
extern std::shared_ptr<mesh> f16;
extern std::shared_ptr<mesh> f16_wrapped;
extern std::shared_ptr<mesh> sphere;
extern std::shared_ptr<mesh> axes;
extern std::shared_ptr<mesh> runway_base;
extern std::shared_ptr<mesh> runway_surface;

void initialize_models();

}
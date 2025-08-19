#pragma once
#include "model_definitions.cpp"

std::shared_ptr<mesh> pyramid;
std::shared_ptr<mesh> ground_color_varying;
std::shared_ptr<mesh> ground_color_uniform;
std::shared_ptr<mesh> jet;
std::shared_ptr<mesh> f16_old;
std::shared_ptr<mesh> cube;
std::shared_ptr<mesh> flame_trail;
std::shared_ptr<mesh> aim9x;
std::shared_ptr<mesh> bullet_octahedron;
std::shared_ptr<mesh> debris_1kg;
std::shared_ptr<mesh> debris_1kg_bright;
std::shared_ptr<mesh> f16;
std::shared_ptr<mesh> f16_wrapped;
std::shared_ptr<mesh> sphere;
std::shared_ptr<mesh> axes;

Eigen::Quaterniond random_quaternion_biased(double angle);

std::vector<vertex> random_isoceles_triangle(double offset_xy_, double offset_z_);

namespace models {

	static void initialize_models();
}
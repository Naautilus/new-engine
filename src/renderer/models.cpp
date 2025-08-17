#pragma once
#include "model_definitions.cpp"

Eigen::Quaterniond random_quaternion_biased(double angle) {
	double angle2 = random(0.0, 2.0*std::numbers::pi);
	angle *= std::numbers::pi / 180;
	angle *= random(0.0, 1.0);
	double angle_x = angle * sin(angle2);
	double angle_y = angle * cos(angle2);
	double angle_z = random(0.0, 2.0*std::numbers::pi);
	return Eigen::AngleAxis(angle_x, vector::localspace(1, 0, 0)) * Eigen::AngleAxis(angle_y, vector::localspace(0, 1, 0)) * Eigen::AngleAxis(angle_z, vector::localspace(0, 0, 1));
}

std::vector<vertex> random_isoceles_triangle(double offset_xy_, double offset_z_) {
	Eigen::Quaterniond rotation = random_quaternion_biased(10);
	std::vector<vector::localspace> isoceles_triangle = {
		rotation * vector::localspace(0     ,    1, 0),
		rotation * vector::localspace(0.866 , -0.5, 0),
		rotation * vector::localspace(-0.866, -0.5, 0)
	};
	std::vector<vertex> output;
	double offset_x = random(-offset_xy_, offset_xy_);
	double offset_y = random(-offset_xy_, offset_xy_);
	double offset_z = random(-offset_z_, offset_z_);
	for (vector::localspace& v : isoceles_triangle) {
		float scale = random(0.5, 1.5);
		float brightness = 0.275;
		output.push_back(vertex{
			(float) (v.x()*scale + offset_x),
			(float) (v.y()*scale + offset_y),
			(float) (v.z()*scale + offset_z),
			brightness, brightness, brightness,
			(float) 0.4
		});
	}
	return output;
}

namespace models {

	static void initialize_models() {

		pyramid = std::make_shared<mesh>(models::stl_to_mesh("pyramid", 1, 1, 1, 1));
		ground_color_varying = std::make_shared<mesh>(models::stl_to_mesh("ground_color_varying", 0, 0.25, 0, 1));
		ground_color_uniform = std::make_shared<mesh>(models::stl_to_mesh("ground_color_uniform", 0, 0.1, 0, 1));
		jet = std::make_shared<mesh>(models::stl_to_mesh("jet", 0.5, 0.5, 0.5, 1));
		f16_old = std::make_shared<mesh>(models::stl_to_mesh("f16_old", 0.5, 0.5, 0.5, 1));
		cube = std::make_shared<mesh>(models::stl_to_mesh("cube", 0.2, 0.2, 0.2, 1));
		flame_trail = std::make_shared<mesh>(models::stl_to_mesh("flame_trail", 1, 0.3, 0.2, 0));
		aim9x = std::make_shared<mesh>(models::stl_to_mesh("aim9x", 0.2, 0.2, 0.2, 1));
		bullet_octahedron = std::make_shared<mesh>(models::stl_to_mesh("bullet_octahedron", 1, 0.4, 0.4, 0));
		debris_1kg = std::make_shared<mesh>(models::stl_to_mesh("debris_1kg", 0.2, 0.2, 0.2, 0.4)); // old rgb 0.275
		debris_1kg_bright = std::make_shared<mesh>(models::stl_to_mesh("debris_1kg", 1, 0.8, 0.6, 0));
		f16 = std::make_shared<mesh>(models::stl_to_mesh("f16", 0.2, 0.2, 0.2, 1));
		f16_wrapped = std::make_shared<mesh>(models::stl_to_mesh("f16_wrapped", 0.2, 0.2, 0.2, 1));
		sphere = std::make_shared<mesh>(models::stl_to_mesh("sphere", 1, 0.8, 0.6, 0));
        axes = std::make_shared<mesh>(models::stl_to_mesh("axes", 1, 0.2, 0.2, 0.5));

	}
}
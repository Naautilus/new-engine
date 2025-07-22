#pragma once
#include "shapes.cpp"

namespace collision {

	std::vector<triangle> simple_jet_collider = {
		triangle(
			vector::worldspace(-4.3,  -5,   0),
			vector::worldspace(-4.3,   5,   0),
			vector::worldspace( 8.5,   0,   0)),
		triangle(
			vector::worldspace(-1.3,   0,   0),
			vector::worldspace(-4.3,   0,  -1),
			vector::worldspace(-4.3,   0,   3))
	};  

	std::vector<triangle> generate_circle(double width) {
		std::vector<triangle> output;
		for (int i = 0; i < constants::CYLINDER_VERTICES; i++) {
			double angle_1 = (2*std::numbers::pi/constants::CYLINDER_VERTICES)*(i);
			double angle_2 = (2*std::numbers::pi/constants::CYLINDER_VERTICES)*(i+1);
			output.push_back(triangle(
				vector::worldspace(0, 0, 0),
				vector::worldspace(0, cos(angle_1)*width, sin(angle_1)*width),
				vector::worldspace(0, cos(angle_2)*width, sin(angle_2)*width)
			));
		}
		return output;
	} 

	std::vector<triangle> generate_cylinder(double length, double width) {
		std::vector<triangle> output;
		for (int i = 0; i < constants::CYLINDER_VERTICES; i++) {
			double angle_1 = (2*std::numbers::pi/constants::CYLINDER_VERTICES)*(i);
			double angle_2 = (2*std::numbers::pi/constants::CYLINDER_VERTICES)*(i+1);
			output.push_back(triangle(
				vector::worldspace( length, 0, 0),
				vector::worldspace( length, cos(angle_1)*width, sin(angle_1)*width),
				vector::worldspace( length, cos(angle_2)*width, sin(angle_2)*width)
			));
			output.push_back(triangle(
				vector::worldspace(-length, 0, 0),
				vector::worldspace(-length, cos(angle_1)*width, sin(angle_1)*width),
				vector::worldspace(-length, cos(angle_2)*width, sin(angle_2)*width)
			));
			output.push_back(triangle(
				vector::worldspace( length, cos(angle_1)*width, sin(angle_1)*width),
				vector::worldspace( length, cos(angle_2)*width, sin(angle_2)*width),
				vector::worldspace(-length, cos(angle_1)*width, sin(angle_1)*width)
			));
			output.push_back(triangle(
				vector::worldspace(-length, cos(angle_1)*width, sin(angle_1)*width),
				vector::worldspace( length, cos(angle_2)*width, sin(angle_2)*width),
				vector::worldspace(-length, cos(angle_2)*width, sin(angle_2)*width)
			));
		}
		return output;
	}

}
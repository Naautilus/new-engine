#pragma once
#include "shapes.cpp"

/*
Colliders must be fully closed bodies for collision normals and
therefore all model-to-model collisions to work. It does not matter if
triangle normals are facing inwards or outwards.
*/

namespace collision {

    
    base0 = vector::worldspace(-4.3,  -5,   0);
    base1 = vector::worldspace(-4.3,   5,   0);
    base2 = vector::worldspace( 8.5,   0,   0);
    base3 = vector::worldspace( 2.1,   0,  -1);
    
    tail0 = vector::worldspace(-1.3,   0,   0);
    tail1 = vector::worldspace(-4.3,   0,  -1);
    tail2 = vector::worldspace(-4.3,   0,   3);
    tail3 = vector::worldspace( 2.8,   1,   0);
    
    std::vector<triangle> simple_jet_collider = {
        triangle(base0, base1, base2),
        triangle(base0, base1, base3),
        triangle(base0, base2, base3),
        triangle(base1, base2, base3),
        
        triangle(tail0, tail1, tail2),
        triangle(tail0, tail1, tail3),
        triangle(tail0, tail2, tail3),
        triangle(tail1, tail2, tail3)
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
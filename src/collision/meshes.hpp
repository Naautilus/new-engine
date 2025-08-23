#pragma once
#include "shapes.hpp"

/*
Colliders must be fully closed bodies for collision normals and
therefore all model-to-model collisions to work. It does not matter if
triangle normals are facing inwards or outwards.
*/

namespace collision {

std::vector<triangle> generate_circle(double width);
std::vector<triangle> generate_cylinder(double length, double width);
std::vector<triangle> generate_rectangle(double size_x, double size_y, double grid_size);

}
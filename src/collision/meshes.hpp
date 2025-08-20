#pragma once
#include "shapes.hpp"

/*
Colliders must be fully closed bodies for collision normals and
therefore all model-to-model collisions to work. It does not matter if
triangle normals are facing inwards or outwards.
*/

namespace collision {

extern vector::worldspace base0;
extern vector::worldspace base1;
extern vector::worldspace base2;
extern vector::worldspace base3;
extern vector::worldspace tail0;
extern vector::worldspace tail1;
extern vector::worldspace tail2;
extern vector::worldspace tail3;

extern std::vector<triangle> simple_jet_collider;

std::vector<triangle> generate_circle(double width);
std::vector<triangle> generate_cylinder(double length, double width);
std::vector<triangle> generate_rectangle(double size_x, double size_y, double grid_size);

}
#pragma once
#include "shapes.hpp"

/*
Colliders must be fully closed bodies for collision normals and
therefore all model-to-model collisions to work. It does not matter if
triangle normals are facing inwards or outwards.
*/

namespace collision {

vector::worldspace base0;
vector::worldspace base1;
vector::worldspace base2;
vector::worldspace base3;

vector::worldspace tail0;
vector::worldspace tail1;
vector::worldspace tail2;
vector::worldspace tail3;

std::vector<triangle> simple_jet_collider;

std::vector<triangle> generate_circle(double width);
std::vector<triangle> generate_cylinder(double length, double width);

}
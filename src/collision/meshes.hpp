#pragma once
#include "shapes.hpp"

/*
Colliders must be fully closed bodies for collision normals and
therefore all model-to-model collisions to work. It does not matter if
triangle normals are facing inwards or outwards.
*/

namespace collision {

vector::worldspace base0 = vector::worldspace(-4.3,  -5,   0);
vector::worldspace base1 = vector::worldspace(-4.3,   5,   0);
vector::worldspace base2 = vector::worldspace( 8.5,   0,   0);
vector::worldspace base3 = vector::worldspace( 2.1,   0,  -1);

vector::worldspace tail0 = vector::worldspace(-1.3,   0,   0);
vector::worldspace tail1 = vector::worldspace(-4.3,   0,  -1);
vector::worldspace tail2 = vector::worldspace(-4.3,   0,   3);
vector::worldspace tail3 = vector::worldspace( 2.8,   1,   0);

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

std::vector<triangle> generate_circle(double width);
std::vector<triangle> generate_cylinder(double length, double width);

}
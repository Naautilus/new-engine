#include "meshes.hpp"

/*
Colliders must be fully closed bodies for collision normals and
therefore all model-to-model collisions to work. It does not matter if
triangle normals are facing inwards or outwards.
*/

namespace collision {

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
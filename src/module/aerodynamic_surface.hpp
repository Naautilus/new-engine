#pragma once
#include "module.hpp"

namespace module {

struct aerodynamic_surface : module {
    double surface_area;
    vector::localspace unrotated_direction;
    vector::localspace rotated_direction;
    vector::localspace response_axes;
    vector::localspace rotation_axis;
    double angle_range;
    aerodynamic_surface(double s, vector::localspace d, vector::localspace p);
    aerodynamic_surface(double s, vector::localspace d, vector::localspace p, vector::localspace response_axes_, vector::localspace rotation_axis_, double a);
    void update(physics_object::object* parent) override;
    void apply_aerodynamic_force(physics_object::object* parent, int iteration);
    void rotate_surface(physics_object::object* parent);
};

}
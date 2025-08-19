#pragma once
#include "../vector/vector_spaces.hpp"
#include "../constants/constants.hpp"
#include "../physics_object/object.hpp"
#include "../physics_object/blueprints.hpp"
#include "../math/random.hpp"
#include "../timer/timer.h"
#include "collider.hpp"
#include "collision_data.hpp"
#include <chrono>

namespace collision {
	
void create_debris_for_objects(physics_object::object& a, physics_object::object& b, double desired_debris_mass, vector::worldspace& collision_point);
void move_colliding_physics_objects(vector::worldspace position_a, vector::worldspace position_b, collision::collider& a_collider, collision::collider& b_collider, physics_object::object& a, physics_object::object& b);

/*
do a binary search to find how far two colliding objects must be
separated before they are no longer colliding
*/
void separate_colliding_physics_objects(vector::worldspace direction, collision::collider& a_collider, collision::collider& b_collider, physics_object::object& a, physics_object::object& b);
void process_colliding_physics_objects(collision::collider& a_collider, collision::collider& b_collider, physics_object::object& a, physics_object::object& b);
double impact_velocity(vector::worldspace& velocity, vector::worldspace& ground_normal);
double angle_of_incidence(vector::worldspace& velocity, vector::worldspace& ground_normal);
collider collider_representing_ground(collider& c);
void process_ground_collision(physics_object::object& o);

}
#pragma once
#include "../vector/vector_spaces.hpp"
#include "../constants/constants.hpp"
#include "../physics_object/object.hpp"
#include "../physics_object/blueprints.hpp"
#include "../math/random.hpp"
#include "../timer/timer.hpp"
#include "collider.hpp"
#include "collision_data.hpp"
#include <chrono>

namespace collision {
	

/*
do a binary search to find how far two colliding objects must be
separated before they are no longer colliding
*/
void process_colliding_physics_objects(collision::collider& a_collider, collision::collider& b_collider, physics_object::object& a, physics_object::object& b);
void process_ground_collision(physics_object::object& o);

}
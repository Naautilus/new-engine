#pragma once
#include "../physics_object/blueprints.cpp"

// SOURCES
// 1: https://www.researchgate.net/figure/Geometrical-mass-and-inertial-properties-of-the-F-16_tbl1_268169601

void initialize_physics_objects() {

	{
		physics_object::object o = physics_object::blueprints::f16_simple_forces_model();
		o.properties.name = "plane1";

		o.physics_state.position = vector::worldspace(0, 0, 10000);
		o.physics_state.velocity = vector::worldspace(500, 0, 0);

        //o.physics_state.position = vector::worldspace(0, 0, globals::current_simulation_state->ground_function(0, 0) + 10);
		//o.physics_state.velocity = vector::worldspace(0, 0, 0);

		o.control_bindings = physics_object::blueprints::plane_control_bindings_wasd();
		globals::physics_objects.push_back(o);
	}

    
	for (int i = 0; i < 5; i++) {
		physics_object::object o = physics_object::blueprints::f16_simple_forces_model();
		o.properties.name = "plane2";
		o.physics_state.position = vector::worldspace(50+i*150, i*150, 10000);
		//o.physics_state.rotation = Eigen::Quaterniond(0, 0, 0, 1); // point backwards
		o.physics_state.rotation = Eigen::AngleAxisd(-0 * std::numbers::pi / 180, vector::worldspace(0, 0, 1)); // point right
		o.physics_state.velocity = vector::worldspace(500, 0, 0);
		o.physics_state.velocity = o.physics_state.rotation.inverse() * o.physics_state.velocity;
		o.control_bindings = physics_object::blueprints::plane_control_bindings_ijkl();
		globals::physics_objects.push_back(o);
	}
    

	{
		physics_object::object o = physics_object::blueprints::sun();
		globals::physics_objects.push_back(o);
	}

	std::cout << "number of physics objects: " + std::to_string(globals::physics_objects.size()) << std::endl;
}
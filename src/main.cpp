#include "renderer/renderer_thread.hpp"
#include "simulation_logic/initialize_physics_objects.hpp"
#include "renderer/renderer_physics_object_connector.hpp"
#include "globals/globals.hpp"
#include "simulation_logic/argument_interpreter.hpp"

// x forward, y right, z up

int main(int argc, char* argv[]) {

    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) args.push_back(argv[i]);

    std::cout << "args: " << "\n";
    for (std::string& arg : args) {
        std::cout << arg;
        std::cout << "\n";
    }
    std::cout << "done\n";
    
    models::initialize_models();
    physics_object::blueprints::initialize_blueprints();

    globals::paused = true;
    globals::pause_mutex.lock();

	globals::physics_objects.reserve(100000);

	renderer r;
	initialize_physics_objects(args);
    interpret_arguments(args);
	renderer_function();

	for (double time = 0; time < constants::TIME_LIMIT || constants::TIME_LIMIT == -1; time += constants::DELTA_T) {
		step_physics_objects();
		if (fmod(time + constants::DELTA_T/2, constants::LOG_INTERVAL) < constants::DELTA_T) {
			if (globals::VERBOSE_LOG) log_physics_objects();
		}
		wait_delta_t();
	}
	std::cout << "done\n";
}
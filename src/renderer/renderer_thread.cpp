#pragma once
#include "../renderer/renderer_physics_object_connector.cpp"

void renderer_function_sleep() {
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void renderer_function() {

	// Print GLFW version to diagnose potential version-related issues
	printf("GLFW version: %s\n", glfwGetVersionString());

	std::cout << "start\n";

	vec3 origin = {0, 0, 0};
	
	glfwInit();
	renderer_function_sleep();
	std::thread t1(renderer::run_window, 2880, 1920, 1440*0, 100, camera_properties("plane1", true, vector::localspace(-20, 0, 10)));
	renderer_function_sleep();
	t1.detach();
	renderer_function_sleep();
	//std::thread t2(renderer::run_window, 1440, 1920, 1440*1, 100, "plane2", true, vector::localspace(-20, 0, 10));
	//renderer_function_sleep();
	//t2.detach();
	//renderer_function_sleep();
	//std::thread t3(renderer::run_window, 800, 1000, 800*1, 100, "aim9x", true, vector::localspace(-15, 0, 5));
	//renderer_function_sleep();
	//t3.detach();
	
}
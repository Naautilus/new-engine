// top of cpp marker
#include "renderer_thread.hpp"

void renderer_function_sleep() {
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void renderer_function(std::vector<std::string> args) {

    bool missile_camera = false;

    if (std::find(args.begin(), args.end(), "-missile-cam") != args.end()) missile_camera = true;

	glfwInit();
    const GLFWvidmode* video_mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    int height = video_mode->height;
    int width = video_mode->width;

    if (missile_camera) {
        renderer_function_sleep();
        std::thread t1(renderer::run_window, width / 2, height, 0, 0, camera_properties("plane1", true, vector::localspace(-20, 0, 10)));
        renderer_function_sleep();
        t1.detach();
        renderer_function_sleep();
        std::thread t2(renderer::run_window, width / 2, height, width / 2, 0, camera_properties("aim9x", false, vector::localspace(-20, 0, 10)));
        renderer_function_sleep();
        t2.detach();
    } else {
        renderer_function_sleep();
        std::thread t1(renderer::run_window, width, height, 0, 0, camera_properties("plane1", true, vector::localspace(-20, 0, 10)));
        renderer_function_sleep();
        t1.detach();
    }
}
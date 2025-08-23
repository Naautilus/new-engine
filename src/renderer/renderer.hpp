#pragma once
#define GLFW_DLL
#include <iostream>
#include "linmath.h"
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include "vector"
#include <string>
#include "../simulation_logic/initialize_physics_objects.hpp"
#include "camera_properties.hpp"
#include "vertex.hpp"

static std::string vertex_shader_text;
static std::string fragment_shader_text;

struct renderer {

	renderer();

    private:

	static void error_callback(int error, const char* description);
    static void update_free_camera_state(GLFWwindow* window);
    static void update_pause_state(GLFWwindow* window);
    static void manual_camera_movement(GLFWwindow* window, double renderer_dt, camera_properties& camera_properties_);
	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void set_vertices(std::vector<vertex>& new_vertices, GLuint& vertex_buffer);
	static void set_vertices_by_models(GLuint& vertex_buffer, std::vector<mesh>& models);
    
    public:

    static bool key_pressed(GLFWwindow* window, int key);
    static void apply_key_responses(GLFWwindow* window, double renderer_dt);
	
	static void create_ground_models(std::vector<mesh>& models, camera_properties& camera_properties_, std::vector<mesh>& ground, bool& new_ground_ready);
	static void create_models_from_physics_objects(std::vector<mesh>& models, camera_properties& camera_properties_, bool& new_ground_ready);
	
    static void run_window(int window_size_x, int window_size_y, int window_pos_x, int window_pos_y, camera_properties camera_properties_);
};
// top of cpp marker
#include "renderer.hpp"
#include "../physics_object/object.hpp"
#include "mesh.hpp"
#include "apply_key_responses.hpp"

renderer::renderer(){
    std::cout << "renderer constructor called\n";
    std::string text;
    std::string line;
    std::ifstream in_vertex("../shaders/vertex_shader_text.txt");
    std::cout << in_vertex.is_open() << "\n";
    while(std::getline(in_vertex, line))
    {
        text += line + "\n";
    }
    vertex_shader_text = text;
    text = "";
    std::ifstream in_fragment("../shaders/fragment_shader_text.txt");
    std::cout << in_fragment.is_open() << "\n";
    while(std::getline(in_fragment, line))
    {
        text += line + "\n";
    }
    fragment_shader_text = text;
};

// callbacks

void renderer::error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}

bool renderer::key_pressed(GLFWwindow* window, int key) {
    bool result = glfwGetKey(window, key) == GLFW_PRESS;
    return result;
}

void renderer::update_free_camera_state(GLFWwindow* window) {
    if (!key_pressed(window, GLFW_KEY_0)) return;
    while (key_pressed(window, GLFW_KEY_0)) glfwPollEvents();
    globals::free_camera = !globals::free_camera;
}

void renderer::update_pause_state(GLFWwindow* window) {
    if (!key_pressed(window, GLFW_KEY_P)) return;
    while (key_pressed(window, GLFW_KEY_P)) glfwPollEvents();
    globals::paused = !globals::paused;

    if (globals::paused) bool _ = globals::pause_mutex.try_lock();
    else globals::pause_mutex.unlock();
}

void renderer::manual_camera_movement(GLFWwindow* window, double renderer_dt, camera_properties& camera_properties_) {
    const double TRANSLATION_SPEED_WALK = 20;
    const double TRANSLATION_SPEED_SPRINT = 100;
    const double ROTATION_SPEED = 1;
    const double FOV_MULTIPLIER_SPEED = 0.5;

    double translation_speed = TRANSLATION_SPEED_WALK;
    if (key_pressed(window, GLFW_KEY_LEFT_SHIFT)) translation_speed = TRANSLATION_SPEED_SPRINT;

    vector::localspace translation(0, 0, 0);
    if (key_pressed(window, GLFW_KEY_W)) translation.x()++;
    if (key_pressed(window, GLFW_KEY_S)) translation.x()--;
    if (key_pressed(window, GLFW_KEY_D)) translation.y()++;
    if (key_pressed(window, GLFW_KEY_A)) translation.y()--;
    if (key_pressed(window, GLFW_KEY_E)) translation.z()++;
    if (key_pressed(window, GLFW_KEY_Q)) translation.z()--;
    translation *= translation_speed * renderer_dt;
    
    vector::worldspace rotation_axis(0, 0, 0);
    if (key_pressed(window, GLFW_KEY_O)) rotation_axis.x()++;
    if (key_pressed(window, GLFW_KEY_U)) rotation_axis.x()--;
    if (key_pressed(window, GLFW_KEY_I)) rotation_axis.y()++;
    if (key_pressed(window, GLFW_KEY_K)) rotation_axis.y()--;
    if (key_pressed(window, GLFW_KEY_J)) rotation_axis.z()++;
    if (key_pressed(window, GLFW_KEY_L)) rotation_axis.z()--;
    rotation_axis *= ROTATION_SPEED * renderer_dt * (camera_properties_.fov / 90);
    
    double fov_multiplier = 0;
    if (key_pressed(window, GLFW_KEY_RIGHT_BRACKET)) fov_multiplier++;
    if (key_pressed(window, GLFW_KEY_LEFT_BRACKET)) fov_multiplier--;
    fov_multiplier *= FOV_MULTIPLIER_SPEED * renderer_dt;
    fov_multiplier += 1;

    Eigen::Quaterniond& rotation = camera_properties_.previous_camera_rotations.back();
    rotation = Eigen::AngleAxisd(rotation_axis.norm(), rotation_axis.normalized()) * rotation;

    vector::worldspace position = camera_properties_.last_camera_position;
    position += translation.to_worldspace(rotation);

    camera_properties_.fov *= fov_multiplier;
    
    camera_properties_.update(rotation, position, vector::worldspace(0, 0, 0), camera_properties::NOT_TRACKING);

}

void renderer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void renderer::set_vertices(std::vector<vertex>& new_vertices, GLuint& vertex_buffer) {
    vertex* vertices = new vertex[new_vertices.size()];
    copy(new_vertices.begin(), new_vertices.end(), vertices);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    if (vertex_buffer == 0) {
        glGenBuffers(1, &vertex_buffer);
    }
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, new_vertices.size() * sizeof(vertex), vertices, GL_STATIC_DRAW);
    delete[] vertices;
}

void renderer::set_vertices_by_models(GLuint& vertex_buffer, std::vector<mesh>& models) {
    std::vector<vertex> vertices;
    for (int i = 0; i < models.size(); i++) {
        //vertices.insert(vertices.end(), models[i].vertices.begin(), models[i].vertices.end());
        mesh& m = models[i];
        for (int j = 0; j < m.indices.size(); j++) {
            vertices.push_back(m.vertices[m.indices[j]]);
        }
    }
    set_vertices(vertices, vertex_buffer);
}

// window creation + looping
void renderer::run_window(int window_size_x, int window_size_y, int window_pos_x, int window_pos_y, camera_properties camera_properties_) {
    const float Z_NEAR = 10;
    const float Z_FAR = 3e8;

    // initialize
    GLFWwindow* window;
    GLuint vertex_shader, fragment_shader, program;
    GLint mvp_location, vpos_location, vcol_location;
    GLuint vertex_buffer;
    double renderer_dt = 0;
    double time = 0;
    std::vector<mesh> models;
    std::vector<mesh> ground;
    bool new_ground_ready;

    // create window
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    GLFWmonitor *monitor = NULL;
    // uncomment to go fullscreen
    //monitor = glfwGetPrimaryMonitor();
    window = glfwCreateWindow(window_size_x, window_size_y, "Window", monitor, NULL);
    if(!window) {
        glfwTerminate();
        return;
    }

    // setup window
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glfwSetErrorCallback(error_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSwapInterval(0);
    glfwSetWindowPos(window, window_pos_x, window_pos_y);
    glfwFocusWindow(window);

    glEnable(GL_DEPTH_TEST);

    std::cout << "vertex_shader_text: " << vertex_shader_text << "END\n";
    std::cout << "fragment_shader_text: " << fragment_shader_text << "END\n";

    // the part i really don't understand
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);

    static const char* vertex_shader_source = vertex_shader_text.c_str();
    static const char* fragment_shader_source = fragment_shader_text.c_str();

    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_source, NULL);
    glCompileShader(vertex_shader);

    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_source, NULL);
    glCompileShader(fragment_shader);

    GLint success;
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(fragment_shader, 512, nullptr, infoLog);
        std::cerr << "Fragment Shader Compilation Failed:\n" << infoLog << std::endl;
    }

    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    glUseProgram(program);

    const GLchar* shader_time_name = "time";
    auto shader_time = glGetUniformLocation(program, shader_time_name);

    const GLchar* shader_fov_name = "fov";
    auto shader_fov = glGetUniformLocation(program, shader_fov_name);

    const GLchar* shader_z_near_name = "z_near";
    auto shader_z_near = glGetUniformLocation(program, shader_z_near_name);
    glUniform1f(shader_z_near, Z_NEAR);
    
    const GLchar* shader_z_far_name = "z_far";
    auto shader_z_far = glGetUniformLocation(program, shader_z_far_name);
    glUniform1f(shader_z_far, Z_FAR);

    const GLchar* shader_sky_color_name = "sky_color";
    auto shader_sky_color = glGetUniformLocation(program, shader_sky_color_name);
    glUniform3f(shader_sky_color, globals::current_simulation_state->sky_color.r, globals::current_simulation_state->sky_color.g, globals::current_simulation_state->sky_color.b);

    const GLchar* shader_resolution_name = "resolution";
    auto shader_resolution = glGetUniformLocation(program, shader_resolution_name);
    glUniform2f(shader_resolution, window_size_x, window_size_y);

    const GLchar* shader_air_density_name = "air_density";
    auto shader_air_density = glGetUniformLocation(program, shader_air_density_name);

    // model view projection:
    // Model: modelspace --> worldspace
    // View: worldspace --> cameraspace
    // Projection: cameraspace --> perspectivified screenspace
    // vPos = vertex position, vCol = vertex color
    mvp_location = glGetUniformLocation(program, "MVP"); // model view projection matrix
    vpos_location = glGetAttribLocation(program, "vPos");
    vcol_location = glGetAttribLocation(program, "vCol");

    glEnableVertexAttribArray(vpos_location);
    glVertexAttribPointer(vpos_location, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*) 0);
    glEnableVertexAttribArray(vcol_location);
    glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*) (sizeof(float) * 3));

    // rgb(72, 116, 163)
    //glClearColor(72, 116, 163, 1.0f);
    double old_time = glfwGetTime();
    double current_time = glfwGetTime();
    int frame_count = 0;
    while (!glfwWindowShouldClose(window)) {

        create_ground_models(models, camera_properties_, ground, new_ground_ready);
        create_models_from_physics_objects(models, camera_properties_, new_ground_ready);
        set_vertices_by_models(vertex_buffer, models);

        // initialize variables
        double ratio;
        int width, height;
        mat4x4 m, v, p, vp, mvp;

        // view matrix (v) controls camera position and angle
        vec3 eye;
        vec3_dup(eye, camera_properties_.camera_position);
        vec3 center;
        vec3_add(center, camera_properties_.camera_position, camera_properties_.camera_z_direction);
        mat4x4_look_at(v,eye,center, camera_properties_.camera_y_direction);
        mat4x4_perspective(p, camera_properties_.fov * std::numbers::pi / 180, window_size_x/(double)window_size_y, Z_NEAR, Z_FAR); //FOV of 90°
        glUniform1f(shader_fov, camera_properties_.fov * std::numbers::pi / 180);
        //mat4x4_ortho(p, -100.f, 100.f, -100.f, 100.f, -10000.f, 10000.f);
        mat4x4_mul(vp, p, v);
        int vertex_index = 0;
        // identity means no rotation
        mat4x4_identity(m);
        mat4x4_mul(mvp, vp, m);
        glUniformMatrix4fv(mvp_location, 1, GL_FALSE, (const GLfloat*) mvp);
        for (int i = 0; i < models.size(); i++) {
            vertex_index += models[i].indices.size();
        }
        glDrawArrays(GL_TRIANGLES, 0, vertex_index);

        // camera + rotation
        glfwSwapBuffers(window);
        glfwPollEvents();
        update_pause_state(window);
        update_free_camera_state(window);
        if (!globals::free_camera) apply_key_responses(window, renderer_dt);
        else manual_camera_movement(window, renderer_dt, camera_properties_);

        /*
        std::cout << "ground::fluid_density(camera_properties_.camera_position[1]): " << ground::fluid_density(camera_properties_.camera_position[1]) << "\n";
        std::cout << "ground::fluid_density(constants::WATER_LEVEL + 1): " << ground::fluid_density(constants::WATER_LEVEL + 1) << "\n";
        */

        float fluid_density_fraction = (float)fmin(ground::fluid_density(camera_properties_.camera_position[1]) / ground::fluid_density(constants::WATER_LEVEL + 1), 1.0);
        ///*
        std::cout << "altitude: " << camera_properties_.camera_position[1] << "\n";
        std::cout << "fluid_density_fraction: " << fluid_density_fraction << "\n";
        //*/
        glUniform1f(shader_air_density, fluid_density_fraction);

        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (double) height;

        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        frame_count++;
        old_time = current_time;
        current_time = glfwGetTime();
        renderer_dt = current_time - old_time;
        time += renderer_dt;
        glUniform1f(shader_time, time);
        if (frame_count % 100 == 0) {
            double fps = 1/renderer_dt;
            //std::cout << "fps: " << fps << "\n";
        }
    }
    glfwTerminate();
}
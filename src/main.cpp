#include "renderer/renderer_thread.cpp"
#include "simulation_state.cpp"
#include "simulation_logic/initialize_physics_objects.cpp"
#include "renderer/renderer_physics_object_connector.cpp"

// x forward, y right, z up

int main(int argc, char* argv[]) {

    /*
    
    inputs pattern:

    ./main.exe -scenario scenario1.json
    
    */

    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) args.push_back(argv[i]);
    
    models::initialize_models();
    physics_object::blueprints::initialize_blueprints();

    /*
    collision::triangle t1, t2;
    collision::line_segment ls;

    t1 = collision::triangle(
        vector::worldspace(0.00, 0.00, 0.00),
        vector::worldspace(1.00, 1.00, 0.00),
        vector::worldspace(1.00, 0.00, 0.00)
    );
    t2 = collision::triangle(
        vector::worldspace(0.50, 0.00, 0.50),
        vector::worldspace(1.00, 0.00, 0.50),
        vector::worldspace(0.75, 1.00,-0.50)
    );
    ls = t1.intersection(t2);
    std::cout << "Intersection results:\n";
    std::cout << "start = " << ls.line_.point_along_line(0).transpose() << "\n";
    std::cout << "end = " << ls.line_.point_along_line(ls.length).transpose() << "\n";
    ls = t2.intersection(t1);
    std::cout << "Intersection results:\n";
    std::cout << "start = " << ls.line_.point_along_line(0).transpose() << "\n";
    std::cout << "end = " << ls.line_.point_along_line(ls.length).transpose() << "\n";

    t1 = collision::triangle(
        vector::worldspace(0.00, 0.00, 0.00),
        vector::worldspace(1.00, 1.00, 0.00),
        vector::worldspace(1.00, 0.00, 0.00)
    );
    t2 = collision::triangle(
        vector::worldspace(0.50, 0.00, 0.50),
        vector::worldspace(1.00, 0.00, 0.50),
        vector::worldspace(0.25, 1.00,-0.50)
    );
    ls = t1.intersection(t2);
    std::cout << "Intersection results:\n";
    std::cout << "start = " << ls.line_.point_along_line(0).transpose() << "\n";
    std::cout << "end = " << ls.line_.point_along_line(ls.length).transpose() << "\n";
    ls = t2.intersection(t1);
    std::cout << "Intersection results:\n";
    std::cout << "start = " << ls.line_.point_along_line(0).transpose() << "\n";
    std::cout << "end = " << ls.line_.point_along_line(ls.length).transpose() << "\n";
    return 0;
    */

    /*
	models::mesh_to_stl("test1", mesh(models::f16Vertices));
	mesh m1 = models::stl_to_mesh("test1", 0.2, 0.2, 0.2, 1);
	models::mesh_to_stl("test1", m1);
	models::mesh_to_stl("test2", mesh(models::cubeVertices));
	mesh m2 = models::stl_to_mesh("test2", 0.2, 0.2, 0.2, 1);
	models::mesh_to_stl("test2", m2);
	models::mesh_to_stl("test3", mesh(models::bulletOctahedron));
	mesh m3 = models::stl_to_mesh("test3", 0.2, 0.2, 0.2, 1);
	models::mesh_to_stl("test3", m3);
    
	models::mesh_to_stl("pyramidVertices", mesh(models::pyramidVertices));
	models::mesh_to_stl("groundVerticesColorVarying", mesh(models::groundVerticesColorVarying));
	models::mesh_to_stl("groundVerticesColorUniform", mesh(models::groundVerticesColorUniform));
	models::mesh_to_stl("jetVertices", mesh(models::jetVertices));okjmnjijkmnkm
	models::mesh_to_stl("f16Points_old", mesh(models::f16Points_old));
	models::mesh_to_stl("f16Vertices_old", mesh(models::f16Vertices_old));
	models::mesh_to_stl("cubeVertices", mesh(models::cubeVertices));
	models::mesh_to_stl("flameTrailVertices", mesh(models::flameTrailVertices));
	models::mesh_to_stl("aim9xVertices", mesh(models::aim9xVertices));
	models::mesh_to_stl("bulletOctahedron", mesh(models::bulletOctahedron));
	models::mesh_to_stl("debris_1kg", mesh(models::debris_1kg));
	models::mesh_to_stl("f16Vertices", mesh(models::f16Vertices));
	std::unique_ptr<simulation_state> s(new simulation_state());
    */
	
	globals::current_simulation_state = std::make_unique<simulation_state>(simulation_state());
	globals::physics_objects.reserve(100000);
	renderer r;
	initialize_physics_objects(args);
	std::thread t(renderer_function);
	for (double time = 0; time < constants::TIME_LIMIT || constants::TIME_LIMIT == -1; time += constants::DELTA_T) {
		step_physics_objects();
		if (fmod(time + constants::DELTA_T/2, constants::LOG_INTERVAL) < constants::DELTA_T) {
			log_physics_objects();
		}
		wait_delta_t();
	}
	std::cout << "done\n";
}
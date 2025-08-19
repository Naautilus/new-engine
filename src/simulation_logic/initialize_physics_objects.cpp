#include "initialize_physics_objects.hpp"
#include "../physics_object/object.hpp"
using json = nlohmann::json;

// SOURCES
// 1: https://www.researchgate.net/figure/Geometrical-mass-and-inertial-properties-of-the-F-16_tbl1_268169601

void initialize_physics_objects(std::vector<std::string> args) {

    auto scenario_index = std::find(args.begin(), args.end(), "-scenario");
    if (scenario_index == args.end()) {
        std::cerr << "error in void initialize_physics_objects(args): argument -scenario not found";
        std::abort();
    }
    if (scenario_index == args.end() - 1) {
        std::cerr << "error in void initialize_physics_objects(args): no argument past -scenario";
        std::abort();
    }
    scenario_index++;
    std::ifstream f("../scenarios/scenario_" + (*scenario_index) + ".json");
    json scenario_data = json::parse(f);
    if (!scenario_data.contains("physics_objects")) {
        std::cerr << "error in void initialize_physics_objects(args):" << "\n"
        << "scenario .json file does not contain array physics_objects; is it empty?";
        std::abort();
    }
    for (size_t i = 0; i < scenario_data["physics_objects"].size(); i++) {
        auto physics_object_ = scenario_data["physics_objects"][i];
        
        physics_object::object (*object_creator)();
        control_bindings (*control_bindings_creator)();
        
        size_t count = 1;
        Eigen::Quaterniond rotation;
        vector::worldspace velocity;
        vector::worldspace root_position;
        vector::worldspace clone_offset;
        
        if (physics_object_.contains("count")) count = physics_object_["count"];
        
        if ((!physics_object_.contains("position") || !physics_object_.contains("clone_offset")) && count != 1) {
            std::cerr << "error in void initialize_physics_objects(args):" << "\n"
            << "physics object in scenario .json is cloned multiple times but does not have both position and clone_offset";
            std::abort();
        }
        
        if (!physics_object_.contains("blueprint")) {
            std::cerr << "error in void initialize_physics_objects(args): physics object in scenario .json has no blueprint";
            std::abort();
        }
        {
            std::string blueprint_name = physics_object_["blueprint"];
            bool blueprint_found = false;
            for (auto named_object : physics_object::blueprints::named_objects) {
                if (named_object.name == blueprint_name) {
                    object_creator = named_object.blueprint_creator;
                    blueprint_found = true;
                }
            }
            if (!blueprint_found) {
                std::cerr << "error in void initialize_physics_objects(args): physics object blueprint \""
                << blueprint_name << "\" in scenario .json is invalid";
                std::abort();
            }
        }
        {
            bool control_bindings_found = false;
            if (physics_object_.contains("control_bindings")) {
                std::string control_bindings_name = physics_object_["control_bindings"];
                for (auto named_control_bindings : physics_object::blueprints::named_control_bindings) {
                    if (named_control_bindings.name == control_bindings_name) {
                        control_bindings_creator = named_control_bindings.blueprint_creator;
                        control_bindings_found = true;
                    }
                }
                if (!control_bindings_found) {
                    std::cerr << "error in void initialize_physics_objects(args): physics object control_bindings \""
                    << control_bindings_name << "\" in scenario .json is invalid";
                    std::abort();
                }
            }
        }

        if (physics_object_.contains("rotation")) {
            double degrees_around_axis = physics_object_["rotation"]["degrees_around_axis"];
            vector::worldspace axis;
            axis.x() = physics_object_["rotation"]["axis"]["x"];
            axis.y() = physics_object_["rotation"]["axis"]["y"];
            axis.z() = physics_object_["rotation"]["axis"]["z"];
            rotation = Eigen::AngleAxisd(degrees_around_axis * std::numbers::pi / 180, axis);
        }
        
        if (physics_object_.contains("velocity")) {
            velocity.x() = physics_object_["velocity"]["x"];
            velocity.y() = physics_object_["velocity"]["y"];
            velocity.z() = physics_object_["velocity"]["z"];
            if (physics_object_["velocity"]["localspace"]) {
                velocity = rotation.inverse() * velocity;
            }
        }

        if (physics_object_.contains("position")) {
            root_position.x() = physics_object_["position"]["x"];
            root_position.y() = physics_object_["position"]["y"];
            root_position.z() = physics_object_["position"]["z"];
            if (physics_object_["position"]["relative_to_ground"]) {
                root_position.z() += ground::get_ground_altitude(root_position.x(), root_position.y());
            }
        }

        if (physics_object_.contains("clone_offset")) {
            clone_offset.x() = physics_object_["clone_offset"]["x"];
            clone_offset.y() = physics_object_["clone_offset"]["y"];
            clone_offset.z() = physics_object_["clone_offset"]["z"];
            if (physics_object_["clone_offset"]["localspace"]) {
                clone_offset = rotation.inverse() * clone_offset;
            }
        }

        for (size_t j = 0; j < count; j++) {

            vector::worldspace position = root_position + j * clone_offset; 
            
            auto o = std::make_shared<physics_object::object>(object_creator());
            if (physics_object_.contains("name")) o->properties.name = physics_object_["name"];
            if (physics_object_.contains("rotation")) o->physics_state.rotation = rotation;
            if (physics_object_.contains("velocity")) o->physics_state.velocity = velocity;
            if (physics_object_.contains("position")) o->physics_state.position = position;
            if (physics_object_.contains("control_bindings")) o->control_bindings = control_bindings_creator();
            globals::physics_objects_mutex.lock();
            globals::physics_objects.push_back(o);
            globals::physics_objects_mutex.unlock();

        }
    }

	std::cout << "number of physics objects: " + std::to_string(globals::physics_objects.size()) << std::endl;
}
#pragma once
#include "properties.cpp"
#include <mutex>

namespace physics_object {
    struct object {

        control_bindings control_bindings;
        physics_state physics_state;
        properties properties;
        std::unique_ptr<std::mutex> mutex;

        object();
        object(std::string name_);

        // below is for applying forces - both force and torque
        void apply_force(vector::worldspace origin, vector::worldspace direction);
        void queue_force(vector::worldspace origin, vector::worldspace direction);
        void apply_force(vector::localspace origin, vector::localspace direction);
        void queue_force(vector::localspace origin, vector::localspace direction);

        // below is for applying impulses - impulse just applies a force with its magnitude * constants::DELTA_T
        void apply_impulse(vector::worldspace origin, vector::worldspace direction);
        void queue_impulse(vector::worldspace origin, vector::worldspace direction);
        void apply_impulse(vector::localspace origin, vector::localspace direction);
        void queue_impulse(vector::localspace origin, vector::localspace direction);

        // below is for force calculation
        void apply_queued_forces();

        void add_aerodynamic_surface(module::aerodynamic_surface a);
        void add_autocannon(module::autocannon a);
        void add_physical_structure(module::physical_structure s);
        void add_jet_engine(module::jet_engine j);
        void add_sensor_ir(module::sensor_ir s);
        void add_solid_rocket_motor(module::solid_rocket_motor s);

        void update_modules();

        void calculate_acceleration();
        double calculate_aoa();
        void log();
    };
}
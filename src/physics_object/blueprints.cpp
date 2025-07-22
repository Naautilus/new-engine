#pragma once
#include "../module/module.cpp"
#include "../module/aerodynamic_surface.cpp"
#include "../module/autocannon.cpp"
#include "../module/jet_engine.cpp"
#include "../module/physical_structure.cpp"
#include "../module/sensor_ir.cpp"
#include "../module/solid_rocket_motor.cpp"
#include "../renderer/glad.h"
#include "../math/random.cpp"
#include "../collision/meshes.cpp"
#include <GLFW/glfw3.h>

/*
physics_object_blueprints.cpp stores some commonly used physics objects so they don't
need to be entirely reconstructed every time they are created later on.

initialized at origin with 0 velocity, no name, and no control_bindings.
*/

namespace physics_object {
    namespace blueprints {

        object aim9x() {
            object o("aim9x");
            o.add_physical_structure(module::physical_structure(collision::collider(), models::aim9x));
            o.physics_state.mass = 85.3;
            o.physics_state.health = 85.3;
            o.physics_state.rotational_inertia = vector::localspace(0.171975*100, 64.916831, 64.916831);
            o.physics_state.base_signal_strength = 0;
            o.properties.ticks_lifetime_remaining = 30.0 / constants::DELTA_T;
            // raw lift/drag figures
            o.add_aerodynamic_surface(module::aerodynamic_surface( 0.3 * std::numbers::pi*0.25*0.127*0.127, vector::localspace(1, 0, 0), vector::localspace(0, 0, 0)));
            o.add_aerodynamic_surface(module::aerodynamic_surface( 0.3 * 3.02*0.127  , vector::localspace(0, 1, 0), vector::localspace(0, 0, 0)));
            o.add_aerodynamic_surface(module::aerodynamic_surface( 0.3 * 3.02*0.127  , vector::localspace(0, 0, 1), vector::localspace(0, 0, 0)));
            // control surfaces
            // front surfaces
            o.add_aerodynamic_surface(module::aerodynamic_surface( 1.0 * 0.0261506402, vector::localspace(0, 0, 1), vector::localspace( 1.0408421327,  0.11,     0), vector::localspace( 0.1, 1, 0), vector::localspace( 0, 1, 0), 22.5));
            o.add_aerodynamic_surface(module::aerodynamic_surface( 1.0 * 0.0261506402, vector::localspace(0, 0, 1), vector::localspace( 1.0408421327, -0.11,     0), vector::localspace(-0.1, 1, 0), vector::localspace( 0, 1, 0), 22.5));
            o.add_aerodynamic_surface(module::aerodynamic_surface( 1.0 * 0.0261506402, vector::localspace(0, 1, 0), vector::localspace( 1.0408421327,     0,  0.11), vector::localspace( 0.1, 0, 1), vector::localspace( 0, 0, 1), 22.5));
            o.add_aerodynamic_surface(module::aerodynamic_surface( 1.0 * 0.0261506402, vector::localspace(0, 1, 0), vector::localspace( 1.0408421327,     0, -0.11), vector::localspace(-0.1, 0, 1), vector::localspace( 0, 0, 1), 22.5));
            // rear surfaces
            o.add_aerodynamic_surface(module::aerodynamic_surface( 1.0 * 0.0192643050, vector::localspace(0, 0, 1), vector::localspace(-1.4129078725,  0.11,     0), vector::localspace( 0.1,-1, 0), vector::localspace( 0, 1, 0), 22.5));
            o.add_aerodynamic_surface(module::aerodynamic_surface( 1.0 * 0.0192643050, vector::localspace(0, 0, 1), vector::localspace(-1.4129078725, -0.11,     0), vector::localspace(-0.1,-1, 0), vector::localspace( 0, 1, 0), 22.5));
            o.add_aerodynamic_surface(module::aerodynamic_surface( 1.0 * 0.0192643050, vector::localspace(0, 1, 0), vector::localspace(-1.4129078725,     0,  0.11), vector::localspace( 0.1, 0,-1), vector::localspace( 0, 0, 1), 22.5));
            o.add_aerodynamic_surface(module::aerodynamic_surface( 1.0 * 0.0192643050, vector::localspace(0, 1, 0), vector::localspace(-1.4129078725,     0, -0.11), vector::localspace(-0.1, 0,-1), vector::localspace( 0, 0, 1), 22.5));

            controls::input pitch = controls::input(controls::pitch, controls::instant, -1, 1, 1);
            controls::input yaw = controls::input(controls::yaw, controls::instant, -1, 1, 1);
            controls::input roll = controls::input(controls::roll, controls::instant, -1, 1, 1);
            o.control_bindings.inputs.push_back(pitch);
            o.control_bindings.inputs.push_back(yaw);
            o.control_bindings.inputs.push_back(roll);

            o.add_solid_rocket_motor(module::solid_rocket_motor(12800, 5, vector::localspace(1, 0, 0), vector::localspace(-1.4, 0, 0), 0.127*0.5, 0.127*0.5, 100));
            o.add_sensor_ir(module::sensor_ir(60, 60, 10, Eigen::Quaterniond(1, 0, 0, 0), vector::localspace(0, 0, 0), 0.1, 0.1, 100));
            return o;
        }

        object bullet(double diameter) {
            object o("bullet");
            o.add_physical_structure(module::physical_structure(collision::collider(), models::bullet_octahedron));
            o.physics_state.mass = 0.1 * diameter*diameter*diameter / (0.02*0.02*0.02);
            o.physics_state.health = o.physics_state.mass;
            o.properties.ticks_lifetime_remaining = 3.0 / constants::DELTA_T;
            return o;
        }

        object bullet_20mm() {
            return bullet(0.02);
        }

        Eigen::Quaterniond random_quaternion() {
            double x,y,z, u,v,w, s;
            do { x = random(-1,1); y = random(-1,1); z = x*x + y*y; } while (z > 1);
            do { u = random(-1,1); v = random(-1,1); w = u*u + v*v; } while (w > 1);
            s = sqrt((1-z) / w);
            return Eigen::Quaterniond(x, y, s*u, s*v);
        }


        object debris(double mass) {
            bool hot = (random(0, 1) < 0.7);
            double drag_side_multiplier = 1;
            double drag_front_multiplier = 1; // when front and side differ, pieces tend to completely lose control and disperse immediately so it's not advised
            object o("");
            o.properties.functional = false;
            double scale = cbrt(mass * 0.003);//0.001
            if (hot) {
                o.add_physical_structure(module::physical_structure(collision::collider(), models::debris_1kg_bright, vector::localspace(0, 0, 0), vector::localspace(1, 1, 1) * 3 * scale));
                o.properties.ticks_lifetime_remaining = (0.015/random(0.05, 1)) / constants::DELTA_T;
            } else {
                o.add_physical_structure(module::physical_structure(collision::collider(), models::debris_1kg, vector::localspace(0, 0, 0), vector::localspace(1, 1, 1) * scale));
                o.properties.ticks_lifetime_remaining = 30.0 / constants::DELTA_T;
            }
            o.physics_state.mass = mass;
            o.physics_state.health = o.physics_state.mass;
            o.physics_state.angular_velocity.x() = 1.0 * random(-1.0, 1.0);
            o.physics_state.angular_velocity.y() = 1.0 * random(-1.0, 1.0);
            o.physics_state.angular_velocity.z() = 1.0 * random(-1.0, 1.0);
            double rotational_inertia = 0.4 * mass * scale * scale;
            o.physics_state.rotational_inertia.x() = rotational_inertia;
            o.physics_state.rotational_inertia.y() = rotational_inertia;
            o.physics_state.rotational_inertia.z() = rotational_inertia;
            o.physics_state.rotation = random_quaternion();
            o.add_aerodynamic_surface(module::aerodynamic_surface(drag_side_multiplier *scale*scale, vector::localspace(1,0,0), vector::localspace(0, random(-1,1)*0.2*scale,    random(-1,1)*0.2*scale   )));
            o.add_aerodynamic_surface(module::aerodynamic_surface(drag_side_multiplier *scale*scale, vector::localspace(0,1,0), vector::localspace(   random(-1,1)*0.2*scale, 0, random(-1,1)*0.2*scale   )));
            o.add_aerodynamic_surface(module::aerodynamic_surface(drag_front_multiplier*scale*scale, vector::localspace(0,0,1), vector::localspace(   random(-1,1)*0.2*scale,    random(-1,1)*0.2*scale, 0)));
            return o;
        }

        object debris_1kg() {
            return debris(1.0);
        }

        control_bindings plane_control_bindings_wasd() {
            control_bindings control_bindings_;
            controls::input pitch = controls::input(controls::pitch, controls::trim_resetting, -0.15, 0.15, 1);
            pitch.add_key(GLFW_KEY_W, -3);
            pitch.add_key(GLFW_KEY_S, 3);
            control_bindings_.inputs.push_back(pitch);
            controls::input yaw = controls::input(controls::yaw, controls::trim_resetting, -0.3, 0.3, 1);
            yaw.add_key(GLFW_KEY_E, -3);
            yaw.add_key(GLFW_KEY_Q, 3);
            control_bindings_.inputs.push_back(yaw);
            controls::input roll = controls::input(controls::roll, controls::trim_resetting, -0.2, 0.2, 1);
            roll.add_key(GLFW_KEY_A, -3);
            roll.add_key(GLFW_KEY_D, 3);
            control_bindings_.inputs.push_back(roll);
            controls::input engine1 = controls::input(controls::engine1, controls::trim_not_resetting, 0, 1, 1);
            engine1.add_key(GLFW_KEY_Z, 0.5);
            engine1.add_key(GLFW_KEY_X, -0.5);
            control_bindings_.inputs.push_back(engine1);
            controls::input gun1 = controls::input(controls::gun1, controls::instant, 0, 1, 1);
            gun1.add_key(GLFW_KEY_SPACE, 1);
            control_bindings_.inputs.push_back(gun1);
            return control_bindings_;
        }

        control_bindings plane_control_bindings_ijkl() {
            control_bindings control_bindings_;
            controls::input pitch = controls::input(controls::pitch, controls::trim_resetting, -0.15, 0.15, 1);
            pitch.add_key(GLFW_KEY_I, -3);
            pitch.add_key(GLFW_KEY_K, 3);
            control_bindings_.inputs.push_back(pitch);
            controls::input yaw = controls::input(controls::yaw, controls::trim_resetting, -0.3, 0.3, 1);
            yaw.add_key(GLFW_KEY_O, -3);
            yaw.add_key(GLFW_KEY_U, 3);
            control_bindings_.inputs.push_back(yaw);
            controls::input roll = controls::input(controls::roll, controls::trim_resetting, -0.2, 0.2, 1);
            roll.add_key(GLFW_KEY_J, -3);
            roll.add_key(GLFW_KEY_L, 3);
            control_bindings_.inputs.push_back(roll);
            controls::input engine1 = controls::input(controls::engine1, controls::trim_not_resetting, 0, 1, 1);
            engine1.add_key(GLFW_KEY_N, 0.5);
            engine1.add_key(GLFW_KEY_M, -0.5);
            control_bindings_.inputs.push_back(engine1);
            controls::input gun1 = controls::input(controls::gun1, controls::instant, 0, 1, 1);
            gun1.add_key(GLFW_KEY_B, 1);
            control_bindings_.inputs.push_back(gun1);
            return control_bindings_;
        }

        object f16_simple_forces_model() {
            object o("");
            o.add_physical_structure(module::physical_structure(collision::collider(collision::simple_jet_collider), models::f16));
            o.physics_state.mass = 9300;
            o.physics_state.health = 9300;
            o.physics_state.rotational_inertia = vector::localspace(12874.84, 75673.58, 85554.4);
            // raw lift/drag figures                    cd/cl, area           axis normal to plane        center of pressure
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.011 * 50.70     , vector::localspace(1, 0, 0), vector::localspace(0, 0, 0)));
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.65 * 40         , vector::localspace(0, 1, 0), vector::localspace(-3, 0, 0.1)));
            o.add_aerodynamic_surface(module::aerodynamic_surface(1.5*50 - 2*2.5    , vector::localspace(0, 0, 1), vector::localspace(0, 0, 0)));
            // control surfaces
            // elevators
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.5*3.8           , vector::localspace(0, 0, 1), vector::localspace(   -8, -3.2, 0), vector::localspace( 0, 1, 0), vector::localspace(0, 1, 0), 24));
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.5*3.8           , vector::localspace(0, 0, 1), vector::localspace(   -8,  3.2, 0), vector::localspace( 0, 1, 0), vector::localspace(0, 1, 0), 24));
            // ailerons
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.5*2.5           , vector::localspace(0, 0, 1), vector::localspace( -5.0, -6.5, 0), vector::localspace(-1, 0, 0), vector::localspace(0, 1, 0), 20));
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.5*2.5           , vector::localspace(0, 0, 1), vector::localspace( -5.0,  6.5, 0), vector::localspace( 1, 0, 0), vector::localspace(0, 1, 0), 20));
            // rudder
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.5*2.4           , vector::localspace(0, 1, 0), vector::localspace(   -8,  0, 3.5), vector::localspace( 0, 0, 1), vector::localspace(0, 0, 1), 30));
            // counteract control surfaces with the opposite wings
            // elevators
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.5*-3.8          , vector::localspace(0, 0, 1), vector::localspace(   -8, -3.2, 0)));
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.5*-3.8          , vector::localspace(0, 0, 1), vector::localspace(   -8,  3.2, 0)));
            // rudder
            o.add_aerodynamic_surface(module::aerodynamic_surface(0.5*-2.4          , vector::localspace(0, 1, 0), vector::localspace(   -8,  0, 3.5)));
            o.add_jet_engine(module::jet_engine(130000, 0, 1, vector::localspace(1, 0, 0), vector::localspace(-3, 0, 0), 4.85, 0.59, 500));
            //o.add_autocannon(autocannon(bullet_20mm, 1050, 100, vector::localspace(1, 0, 0), vector::localspace(0, 0, 0), 1, 0.1, 100));
            //o.add_autocannon(autocannon(debris_1kg, 100, 10, vector::localspace(1, 0, 0), vector::localspace(0, 0, 0), 1, 0.1, 100));
            o.add_autocannon(module::autocannon(aim9x, 0, 3, vector::localspace(1, 0, 0), vector::localspace(0, 0, 0), 1, 0.1, 100));
            return o;
        }

        object sun() {
            object o("sun");
            o.properties.fixed = true;
            o.add_physical_structure(module::physical_structure(collision::collider(), models::sphere, vector::localspace(0,0,0), vector::localspace(1,1,1) * 6.957e+8)); // sun radius in m
            o.physics_state.position = globals::SUN_DIRECTION * -1.495979e+11; // 1 au to m
            o.physics_state.mass = 1;
            o.physics_state.health = 1;
            o.physics_state.rotational_inertia = vector::localspace(1, 1, 1);
            o.physics_state.base_signal_strength = 1e+18;
            return o;
        }

        object axes(vector::worldspace position_) {
            object o("axes");
            o.properties.fixed = true;
            o.properties.functional = false;
            o.add_physical_structure(module::physical_structure(collision::collider(), models::axes, vector::localspace(0,0,0), vector::localspace(1,1,1) * 0.001));
            o.physics_state.position = position_;
            o.physics_state.mass = 1;
            o.physics_state.health = 1;
            o.physics_state.rotational_inertia = vector::localspace(1, 1, 1);
            o.properties.ticks_lifetime_remaining = round(0.1 / constants::DELTA_T);
            return o;
        }

        object collider_visual(vector::worldspace position_, collision::collider collider_) {
            object o("collider visual");
            o.properties.fixed = true;
            o.properties.functional = false;
            o.add_physical_structure(module::physical_structure(collision::collider(), std::shared_ptr<mesh>(new mesh(collider_, 1, 0.5, 0.2, 0.8)), vector::localspace(0,0,0), vector::localspace(1,1,1) * 0.001));
            o.physics_state.position = position_;
            o.physics_state.mass = 1;
            o.physics_state.health = 1;
            o.physics_state.rotational_inertia = vector::localspace(1, 1, 1);
            o.properties.ticks_lifetime_remaining = round(0.1 / constants::DELTA_T);
            return o;
        }
        
    }
}
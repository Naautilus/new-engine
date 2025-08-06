#pragma once
#include "module.cpp"
#include "../ground/ground_logic.cpp"

// x = forward distance, y and z = perspectivified
//struct vector::scopespace : vector::localspace {};

namespace module {
    struct signal_point {
        vector::scopespace position_scopespace;
        double signal_strength;
        double distance_weight = 0;
        signal_point() {
            position_scopespace.x() = 0;
            position_scopespace.y() = 0;
            position_scopespace.z() = 0;
            signal_strength = 0;
        }
        signal_point(vector::worldspace target_position, vector::worldspace sensor_position, Eigen::Quaterniond rotation, double base_signal_strength) {
            vector::worldspace position_relative_worldspace = target_position - sensor_position;
            position_scopespace = static_cast<vector::scopespace>(position_relative_worldspace.to_localspace(rotation));
            position_scopespace.y() /= position_scopespace.x();
            position_scopespace.z() /= position_scopespace.x();
            signal_strength = base_signal_strength / (position_scopespace.squaredNorm());
        }
        signal_point(double x_, double y_, double z_, double signal_strength_) {
            position_scopespace.x() = x_;
            position_scopespace.y() = y_;
            position_scopespace.z() = z_;
            signal_strength = signal_strength_;
        }
    };

    struct sensor_cell_grid {
        private:
        int size;
        double min;
        double max;
        public:
        std::vector<signal_point> points;
        sensor_cell_grid(int size_, double view_cone_halfarc) {
            double view_cone = tan(view_cone_halfarc * std::numbers::pi / 180);
            size = size_;
            min = -view_cone;
            max = view_cone;
            points.clear();
            for (int y = 0; y < size; y++) {
                for (int x = 0; x < size; x++) {
                    double x_ = 0;
                    double y_ = get_value_from_axis_index(x);
                    double z_ = get_value_from_axis_index(y);
                    points.push_back(signal_point(x_, y_, z_, 0.0));
                }
            }
            //std::cout << "min: " << min << std::endl;
            //std::cout << "max: " << max << std::endl;
        }
        void increase_signals_in_circle(double center_x, double center_y, double radius, double distance, double signal) {
            std::vector<int> indices = get_indices_in_circle(center_x, center_y, radius);
            for (int i : indices) {
                points[i].signal_strength += signal;
                points[i].position_scopespace.x() += distance * signal;
                points[i].distance_weight += signal;
            }
        }
        std::vector<int> get_indices_in_circle(double center_x, double center_y, double radius) {\
            std::vector<int> output;
            for (int i = 0; i < points.size(); i++) {
                vector::scopespace p_pos = points[i].position_scopespace;
                p_pos.y() -= center_x;
                p_pos.z() -= center_y;
                if (p_pos.y() * p_pos.y() + p_pos.z() * p_pos.z() < radius * radius) {
                    output.push_back(i);
                }
            }
            return output;
        }
        int index_from_coordinates(double x, double y) {
            x -= min;
            y -= min;
            x /= (max - min);
            y /= (max - min);
            x *= size;
            y *= size;
            int x_ = round(x);
            int y_ = round(y);
            int index = (x_ + y_*size);
            return index;
        }
        /*
        std::pair<double, double> coordinates_from_index(int index) {
            double x = index % size;
            double y = index / size;
            x *= (max - min);
            y *= (max - min);
            x += min;
            y += min;
            return std::pair<x, y>;
        }
        */
        double get_value_from_axis_index(int index) {
            double value = index;
            value /= size;
            value *= (max - min);
            value += min;
            return value;
        }
        void print(vector::scopespace target, vector::scopespace center) {
            std::string LEVELS = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
            int target_index = index_from_coordinates(target.y(), target.z());
            int center_index = index_from_coordinates(center.y(), center.z());
            double max = std::numeric_limits<double>::min();
            double min = std::numeric_limits<double>::max();
            for (signal_point& s : points) {
                max = fmax(s.signal_strength, max);
                min = fmin(s.signal_strength, min);
            }
            for (int y = size - 1; y >= 0; y--) {
                for (int x = 0; x < size; x++) {
                    int index = x + y*size;
                    double value = points[index].signal_strength;
                    value -= min;
                    value /= (max - min);
                    int level = floor(value * (LEVELS.size()-1));
                    if (index == target_index) {
                        std::cout << "()";
                        continue;
                    }
                    if (index == center_index) {
                        std::cout << "><";
                        continue;
                    }
                    std::cout << LEVELS[level];
                    std::cout << LEVELS[level];
                }
                std::cout << "|\n";
            }
            std::cout << "\n";
        }
    };

    struct sensor_ir : module {
        double gimbal_cone_halfarc;
        double view_cone_halfarc;
        double target_recognition_cone_halfarc;
        vector::worldspace last_detection_worldspace;
        vector::scopespace last_detection_scopespace;
        double last_detection_signal_strength;
        double record_target_distance = 1e10;
        pid pid_pitch = pid(    1,    0,  0.1,  0.5);
        pid pid_yaw   = pid(    1,    0,  0.1,  0.5);
        pid pid_roll  = pid(    1,    0,  0.1,  1.0); // fed angular velocity, so P is D really
        double time_since_launch = 0;
        sensor_ir(double gimbal_cone_halfarc_, double view_cone_halfarc_, double target_recognition_cone_halfarc_, Eigen::Quaterniond rotation_, vector::localspace position_, double length, double width, double health_) {
            gimbal_cone_halfarc = gimbal_cone_halfarc_;
            view_cone_halfarc = view_cone_halfarc_;
            target_recognition_cone_halfarc = target_recognition_cone_halfarc_;
            rotation = rotation_;
            position = position_;
            //std::vector<collision_model::triangle> model = collision_model::generate_cylinder(length, width);
            //collider = collision_model::collider(model);
            health = health_;
        }

        vector::worldspace get_worldspace_position(physics_object::object* parent) {
            return position.to_worldspace_positional(parent->physics_state.rotation, parent->physics_state.position);
        }

        void update(physics_object::object* parent) override {
            //if (health <= 0) return;
            time_since_launch += constants::DELTA_T;
            vector::worldspace current_detection_relative_worldspace = get_target_position(parent);
            double target_distance = current_detection_relative_worldspace.norm();
            if (target_distance != 0) record_target_distance = fmin(record_target_distance, target_distance);
            //std::cout << "record target distance: " << record_target_distance << std::endl;
            vector::localspace guidance_pid_inputs = vector::localspace(0, 0, 0);
            vector::localspace guidance_pid_inputs_prograde = get_guidance_prograde(current_detection_relative_worldspace, parent);

            double apn_gain = 5.0;
            apn_gain *= std::clamp((time_since_launch-0.1)*0.5, 0.0, 1.0);
            apn_gain *= 1-get_g_limit_fraction(parent->physics_state.recorded_acceleration.norm(), 40*constants::STANDARD_GRAVITY, 60*constants::STANDARD_GRAVITY);
            guidance_pid_inputs = get_guidance_proportional_navigation(current_detection_relative_worldspace, parent, apn_gain);
            //guidance_pid_inputs = get_guidance_first_degree_prediction(current_detection_relative_worldspace, parent);
            
            if (get_time_to_impact_first_degree_prediction(current_detection_relative_worldspace, parent) < 0.1) {
                guidance_pid_inputs = guidance_pid_inputs_prograde;
            }
            
            
            //std::cout << interp << std::endl;
            pid_roll.update(guidance_pid_inputs.x());
            pid_pitch.update(guidance_pid_inputs.y());
            pid_yaw.update(guidance_pid_inputs.z());
            
            if (std::isnan(guidance_pid_inputs.x())
            || std::isnan(guidance_pid_inputs.y())
            || std::isnan(guidance_pid_inputs.z())
            || std::isnan(pid_roll.output)
            || std::isnan(pid_pitch.output)
            || std::isnan(pid_yaw.output)) {
                std::cout << "bull shit\n";
                std::cout << *(int*)nullptr;
            }

            last_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
            controls::input* pitch = parent->control_bindings.get_input(controls::pitch);
            if (pitch) pitch->response_unmultiplied = pid_pitch.output;
            controls::input* yaw = parent->control_bindings.get_input(controls::yaw);
            if (yaw) yaw->response_unmultiplied = pid_yaw.output;
            controls::input* roll = parent->control_bindings.get_input(controls::roll);
            if (roll) roll->response_unmultiplied = pid_roll.output;
        }
        
        double get_g_limit_fraction(double current_acceleration, double g_limit_min, double g_limit_max) {
            double g_limit_fraction = (current_acceleration-g_limit_min) / (g_limit_max-g_limit_min);
            g_limit_fraction = std::clamp(g_limit_fraction, 0.0, 1.0);
            return g_limit_fraction;
        }
        
        vector::worldspace limit_g_forces(vector::worldspace unlimited_inputs, vector::localspace limited_inputs, double current_acceleration, double g_limit_min, double g_limit_max) {
            double g_limit_fraction = get_g_limit_fraction(current_acceleration, g_limit_min, g_limit_max);
            return (1 - g_limit_fraction) * unlimited_inputs + g_limit_fraction * limited_inputs;
        }
        
        vector::worldspace get_enemy_velocity(vector::worldspace current_detection_worldspace, vector::worldspace last_detection_worldspace) {
            return (1/constants::DELTA_T) * (current_detection_worldspace - last_detection_worldspace);
        }

        double get_time_to_impact_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent) {
            vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
            vector::worldspace missile_velocity = parent->physics_state.velocity;
            vector::worldspace enemy_velocity = get_enemy_velocity(current_detection_worldspace, last_detection_worldspace);
            vector::worldspace relative_velocity = enemy_velocity - missile_velocity;

            double time_to_impact = current_detection_relative_worldspace.norm() / relative_velocity.norm();
            return time_to_impact;
        }

        double find_smallest_positive_root(double a, double b, double c) {
            double determinant = b*b - 4*a*c;
            if (determinant < 0) return -1.0;
            if (a == 0) return -1.0;
            double root1 = (-b + sqrt(determinant)) / (2*a);
            double root2 = (-b - sqrt(determinant)) / (2*a);
            if (root1 < 0 && root2 < 0) return -1.0;
            if (root1 > 0 && root2 > 0) return fmin(root1, root2);
            if (root1 > 0) return root1;
            return root2;
        }

        vector::localspace get_guidance_prograde(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent) {
            double GAIN = 1.0;
            vector::scopespace current_detection = signal_point(
                parent->physics_state.velocity,
                vector::worldspace(0, 0, 0),
                rotation * parent->physics_state.rotation,
                1.0 // unimportant
            ).position_scopespace;
            if (current_detection.x() != current_detection.x() || current_detection.y() != current_detection.y() || current_detection.z() != current_detection.z()) {
                current_detection = static_cast<vector::scopespace>(vector::localspace(0, 0, 0));
            }
            current_detection *= GAIN;
            return vector::localspace(
                -parent->physics_state.angular_velocity.to_localspace(parent->physics_state.rotation).x(),
                -current_detection.z(),
                current_detection.y()
            );
        }

        vector::localspace get_guidance_direct(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent) {
            double GAIN = 1.0;
            vector::scopespace current_detection = signal_point(
                current_detection_relative_worldspace,
                vector::worldspace(0, 0, 0),
                rotation * parent->physics_state.rotation,
                1.0 // unimportant
            ).position_scopespace;
            if (current_detection.x() != current_detection.x() || current_detection.y() != current_detection.y() || current_detection.z() != current_detection.z()) {
                current_detection = static_cast<vector::scopespace>(vector::localspace(0, 0, 0));
            }
            current_detection *= GAIN;
            return vector::localspace(
                -parent->physics_state.angular_velocity.to_localspace(parent->physics_state.rotation).x(),
                -current_detection.z(),
                current_detection.y()
            );
        }

        vector::localspace get_guidance_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent) {
            double GAIN = 1.0;
            vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
            vector::worldspace missile_velocity = parent->physics_state.velocity;
            vector::worldspace enemy_velocity = get_enemy_velocity(current_detection_worldspace, last_detection_worldspace);
            vector::worldspace relative_position = current_detection_relative_worldspace;

            double quadratic_c = relative_position.dot(relative_position);
            double quadratic_b = 2 * relative_position.dot(enemy_velocity);
            double quadratic_a = enemy_velocity.dot(enemy_velocity) - missile_velocity.dot(missile_velocity);

            double time = find_smallest_positive_root(quadratic_a, quadratic_b, quadratic_c);
            if (time == -1.0) return vector::localspace(0, 0, 0);

            vector::worldspace aimpoint = current_detection_relative_worldspace + time * enemy_velocity;

            vector::scopespace current_detection = signal_point(
                aimpoint,
                vector::worldspace(0, 0, 0),
                rotation * parent->physics_state.rotation,
                1.0 // unimportant
            ).position_scopespace;
            if (current_detection.x() != current_detection.x() || current_detection.y() != current_detection.y() || current_detection.z() != current_detection.z()) {
                current_detection = static_cast<vector::scopespace>(vector::localspace(0, 0, 0));
            }
            current_detection *= GAIN;
            return vector::localspace(
                -parent->physics_state.angular_velocity.to_localspace(parent->physics_state.rotation).x(),
                -current_detection.z(),
                current_detection.y()
            );
        }

        vector::localspace get_guidance_proportional_navigation(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain) {
            double LOOK_AHEAD_TIME = 1; // for turning an acceleration request into an aimpoint request
            vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
            vector::worldspace missile_velocity = parent->physics_state.velocity;
            vector::worldspace enemy_velocity = get_enemy_velocity(current_detection_worldspace, last_detection_worldspace);
            vector::worldspace relative_velocity = enemy_velocity - missile_velocity;
            vector::worldspace relative_position = current_detection_relative_worldspace;
            vector::worldspace line_of_sight_rotation_vector = (relative_position.cross(relative_velocity)) / (relative_position.dot(relative_position));
            vector::worldspace desired_acceleration = -gain * ((relative_velocity.norm()) * (missile_velocity / missile_velocity.norm())).cross(line_of_sight_rotation_vector);
            if (desired_acceleration.x() != desired_acceleration.x() || desired_acceleration.y() != desired_acceleration.y() || desired_acceleration.z() != desired_acceleration.z()) {
                desired_acceleration = vector::localspace(0, 0, 0);
            }
            vector::worldspace aimpoint = LOOK_AHEAD_TIME * missile_velocity + 0.5 * LOOK_AHEAD_TIME * LOOK_AHEAD_TIME * desired_acceleration;
            vector::scopespace current_detection = signal_point(
                aimpoint,
                vector::worldspace(0, 0, 0),
                rotation * parent->physics_state.rotation,
                1.0 // unimportant
            ).position_scopespace;
            return vector::localspace(
                -parent->physics_state.angular_velocity.to_localspace(parent->physics_state.rotation).x(),
                -current_detection.z(),
                current_detection.y()
            );
        }

        std::vector<signal_point> get_signals_from_physics_objects(physics_object::object* parent) {
            std::vector<signal_point> output;
            double max_scopespace_offset = tan(view_cone_halfarc * std::numbers::pi / 180);
            for (physics_object::object& o : globals::physics_objects) {
                if (!o.properties.functional) continue;
                signal_point s = signal_point(
                    o.physics_state.position, 
                    get_worldspace_position(parent),
                    rotation * parent->physics_state.rotation,
                    o.physics_state.base_signal_strength
                );
                if (s.position_scopespace.x() <= 0) continue;
                double scopespace_offset_squared = s.position_scopespace.y() * s.position_scopespace.y() + s.position_scopespace.z() * s.position_scopespace.z();
                if (scopespace_offset_squared > max_scopespace_offset * max_scopespace_offset) continue;
                if (!ground::line_of_sight(get_worldspace_position(parent), o.physics_state.position)) continue;
                output.push_back(s);
            }
            return output;
        }

        signal_point get_largest_signal(std::vector<signal_point> v) {
            /*
            for (signal_point& p : v) {
                if (p.signal_strength == 0) continue;
                std::cout << "p: " << p.position_scopespace.transpose() << ", s: " << p.signal_strength << std::endl;
            }
            */
            if (v.size() == 0) return signal_point();
            double max_signal_strength = v[0].signal_strength;
            signal_point max_signal_point = v[0];
            for (int i = 1; i < v.size(); i++) {
                if (v[i].signal_strength > max_signal_strength) {
                    max_signal_strength = v[i].signal_strength;
                    max_signal_point = v[i];
                }
            }
            if (max_signal_strength != max_signal_strength || max_signal_strength == 0) {
                max_signal_point = signal_point(1, 0, 0, 123);
                max_signal_point.distance_weight = 1.0;
            }
            return max_signal_point;
        }

        vector::scopespace get_target_direction(physics_object::object* parent) {
            std::vector<signal_point> signals_unfiltered = get_signals_from_physics_objects(parent);
            const int GRID_WIDTH = 30;
            const double LAST_TARGET_BONUS = 0.4;
            sensor_cell_grid grid = sensor_cell_grid(GRID_WIDTH, view_cone_halfarc);
            double target_recognition_radius = tan(target_recognition_cone_halfarc * std::numbers::pi / 180);
            for (signal_point& p : signals_unfiltered) {
                grid.increase_signals_in_circle(p.position_scopespace.y(), p.position_scopespace.z(), target_recognition_radius, p.position_scopespace.x(), p.signal_strength);
            }
            if (
                !std::isnan(last_detection_scopespace.x()) &&
                !std::isnan(last_detection_scopespace.y()) &&
                !std::isnan(last_detection_scopespace.z()) &&
                !std::isnan(last_detection_signal_strength)
            ) {
                grid.increase_signals_in_circle(last_detection_scopespace.y(), last_detection_scopespace.z(), target_recognition_radius, last_detection_scopespace.x(), last_detection_signal_strength);
            }
            signal_point center = get_largest_signal(grid.points);
            std::vector<int> signal_indices_in_target_recognition_circle;
            for (int i = 0; i < signals_unfiltered.size(); i++) {
                double distance_x = signals_unfiltered[i].position_scopespace.y() - center.position_scopespace.y();
                double distance_y = signals_unfiltered[i].position_scopespace.z() - center.position_scopespace.z();
                if (distance_x * distance_x + distance_y * distance_y < target_recognition_radius * target_recognition_radius) {
                    signal_indices_in_target_recognition_circle.push_back(i);
                }
            }
            signal_point signals_averaged = signal_point(0, 0, 0, 0);
            for (int i : signal_indices_in_target_recognition_circle) {
                signal_point& p = signals_unfiltered[i];
                signals_averaged.distance_weight++;
                signals_averaged.position_scopespace += p.position_scopespace;
                signals_averaged.signal_strength += p.signal_strength;
            }
            signals_averaged.position_scopespace /= signals_averaged.distance_weight;
            signals_averaged.signal_strength /= signals_averaged.distance_weight;
            vector::scopespace result = signals_averaged.position_scopespace;
            last_detection_scopespace = signals_averaged.position_scopespace;
            last_detection_signal_strength = signals_averaged.signal_strength;
            //if (globals::tick % 20 == 0) grid.print(result, center.position_scopespace);
            //std::cout << "result: " << result.transpose() << std::endl;
            return result;
        }

        vector::worldspace get_target_position(physics_object::object* parent) {
            vector::scopespace result = get_target_direction(parent);
            result.y() *= result.x();
            result.z() *= result.x();
            return result.to_worldspace(parent->physics_state.rotation);
        }

    };
}

void physics_object::object::add_sensor_ir(module::sensor_ir s) {
	std::shared_ptr<module::module> m = std::make_shared<module::sensor_ir>(s);
	properties.modules.push_back(m);
}
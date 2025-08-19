#include "sensor_ir.hpp"
#include "../physics_object/object.hpp"

// x = forward distance, y and z = perspectivified
//struct vector::scopespace : vector::localspace {};

namespace module {

double signal_point::distance_weight = 0;
signal_point::signal_point() {
    position_scopespace.distance() = 0;
    position_scopespace.scope_x() = 0;
    position_scopespace.scope_y() = 0;
    signal_strength = 0;
}
signal_point::signal_point(vector::worldspace target_position, vector::worldspace sensor_position, Eigen::Quaterniond rotation, double base_signal_strength) {
    vector::worldspace position_relative_worldspace = target_position - sensor_position;
    //std::cout << "position_relative_worldspace: " << position_relative_worldspace.str() << "\n";
    vector::localspace position_localspace = position_relative_worldspace.to_localspace(rotation);
    //std::cout << "position_localspace: " << position_localspace.str() << "\n";
    position_scopespace.distance() = position_localspace.x();
    position_scopespace.scope_x() = position_localspace.y() / position_localspace.x();
    position_scopespace.scope_y() = position_localspace.z() / position_localspace.x();
    //std::cout << "position_scopespace: " << position_scopespace.str() << "\n";
    signal_strength = base_signal_strength / (position_localspace.squaredNorm());
    //std::cout << "signal_strength: " << signal_strength << "\n";
}
signal_point::signal_point(double distance_, double scope_x_, double scope_y_, double signal_strength_) {
    position_scopespace.distance() = distance_;
    position_scopespace.scope_x() = scope_x_;
    position_scopespace.scope_y() = scope_y_;
    signal_strength = signal_strength_;
}
std::string signal_point::str() {
    std::string output = "";
    output += position_scopespace.str();
    output += "[";
    output += signal_strength;
    output += "]";
    return output;
}

sensor_cell_grid::sensor_cell_grid(int size_, double view_cone_halfarc) {
    double view_cone = tan(view_cone_halfarc * std::numbers::pi / 180);
    size = size_;
    min = -view_cone;
    max = view_cone;
    for (int x = 0; x < size; x++) {
        std::vector<signal_point> row;
        for (int y = 0; y < size; y++) {
            double x_ = 0;
            double y_ = get_value_from_axis_index(x);
            double z_ = get_value_from_axis_index(y);
            row.push_back(signal_point(x_, y_, z_, 0.0));
        }
        points.push_back(row);
    }
    //std::cout << "min: " << min << std::endl;
    //std::cout << "max: " << max << std::endl;
}
void sensor_cell_grid::increase_signals_in_circle(double center_x, double center_y, double radius, double distance, double signal) {
    std::vector<sensor_point> indices = get_points_in_circle(center_x, center_y, radius);
    for (sensor_point p : indices) {
        points[p.x()][p.y()].signal_strength += signal;
        points[p.x()][p.y()].position_scopespace.distance() += distance * signal;
        points[p.x()][p.y()].distance_weight += signal;
    }
}
std::vector<sensor_point> sensor_cell_grid::remove_invalid_indices(std::vector<sensor_point> indices) {
    std::vector<sensor_point> output;
    for (sensor_point p : indices) {
        if (p.x() < 0) continue;
        if (p.x() >= size) continue;
        if (p.y() < 0) continue;
        if (p.y() >= size) continue;
        output.push_back(p);
    }
    return output;
}
void sensor_cell_grid::calculate_indices_in_circle(double radius) {
    std::vector<sensor_point> output;
    sensor_point origin = point_from_coordinates(0, 0);
    for (int x = 0; x < size; x++) {
        for (int y = 0; y < size; y++) {
            vector::scopespace p_pos = points[x][y].position_scopespace;
            if (p_pos.scope_x() * p_pos.scope_x() + p_pos.scope_y() * p_pos.scope_y() < radius * radius) {
                output.push_back(sensor_point(x, y));
            }
        }
    }
    circle_radius_indices_offsets[radius] = output;
}
std::vector<sensor_point> sensor_cell_grid::get_points_in_circle(double center_x, double center_y, double radius) {
    sensor_point shift = point_from_coordinates(center_x, center_y) - point_from_coordinates(0, 0);
    if(circle_radius_indices_offsets.find(radius) == circle_radius_indices_offsets.end()) calculate_indices_in_circle(radius);
    std::vector<sensor_point> output = circle_radius_indices_offsets[radius];
    for (sensor_point& p : output) p += shift;
    return remove_invalid_indices(output);
}
sensor_point sensor_cell_grid::point_from_coordinates(double x, double y) {
    x -= min;
    y -= min;
    x /= (max - min);
    y /= (max - min);
    x *= size;
    y *= size;
    int x_ = round(x);
    int y_ = round(y);
    sensor_point point = sensor_point{x_, y_};
    return point;
}
int sensor_cell_grid::index_from_coordinates(double x, double y) {
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
double sensor_cell_grid::get_value_from_axis_index(int index) {
    double value = index;
    value /= size;
    value *= (max - min);
    value += min;
    return value;
}
signal_point sensor_cell_grid::get_largest_signal() {
    double max_signal_strength = points[0][0].signal_strength;
    signal_point max_signal_point = points[0][0];
    for (int x = 0; x < size; x++) {
        for (int y = 0; y < size; y++) {
            if (points[x][y].signal_strength > max_signal_strength) {
                max_signal_strength = points[x][y].signal_strength;
                max_signal_point = points[x][y];
            }
        }
    }
    if (std::isnan(max_signal_strength) || max_signal_strength == 0) {
        max_signal_point = signal_point(1, 0, 0, 123);
        max_signal_point.distance_weight = 1.0;
    }
    //std::cout << "largest signal: [" << max_signal_point.position_scopespace.scope_x() << "][" << max_signal_point.position_scopespace.scope_y() << "] @ " << max_signal_strength << "\n";
    return max_signal_point;
}
double sensor_cell_grid::distance_between_grid_cells() {
    double output = points[1][0].position_scopespace.scope_x() - points[0][0].position_scopespace.scope_x();
    //std::cout << "distance_between_grid_cells: " << output << "\n";
    return output;
}
void sensor_cell_grid::print(vector::scopespace target, vector::scopespace center) {
    std::string LEVELS = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
    int target_index = index_from_coordinates(target.scope_x(), target.scope_y());
    int center_index = index_from_coordinates(center.scope_x(), center.scope_y());
    double max = std::numeric_limits<double>::min();
    double min = std::numeric_limits<double>::max();
    for (auto& row : points) {
        for (signal_point& s : row) {
            max = fmax(s.signal_strength, max);
            min = fmin(s.signal_strength, min);
        }
    }
    for (int y = size - 1; y >= 0; y--) {
        for (int x = 0; x < size; x++) {
            int index = x + y*size;
            double value = points[x][y].signal_strength;
            value -= min;
            value /= (max - min);
            value = sqrt(value);
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

double sensor_ir::record_target_distance = 1e10;
pid sensor_ir::pid_pitch = pid(    1,    0,  0.1,  0.5);
pid sensor_ir::pid_yaw   = pid(    1,    0,  0.1,  0.5);
pid sensor_ir::pid_roll  = pid(    1,    0,  0.1,  1.0); // fed angular velocity, so P is D really
double sensor_ir::time_since_launch = 0;

sensor_ir::sensor_ir(double gimbal_cone_halfarc_, double view_cone_halfarc_, double target_recognition_cone_halfarc_, Eigen::Quaterniond rotation_, vector::localspace position_, double length, double width, double health_) {
    gimbal_cone_halfarc = gimbal_cone_halfarc_;
    view_cone_halfarc = view_cone_halfarc_;
    target_recognition_cone_halfarc = target_recognition_cone_halfarc_;
    rotation = rotation_;
    position = position_;
    //std::vector<collision_model::triangle> model = collision_model::generate_cylinder(length, width);
    //collider = collision_model::collider(model);
    health = health_;
}

vector::worldspace sensor_ir::get_worldspace_position(physics_object::object* parent) {
    return position.to_worldspace_positional(parent->physics_state.rotation, parent->physics_state.position);
}

void sensor_ir::update(physics_object::object* parent) {
    //std::cout << "\n-----------------\n\n";
    //if (health <= 0) return;
    time_since_launch += constants::DELTA_T;
    vector::worldspace current_detection_relative_worldspace = get_target_position(parent);

    double target_distance = current_detection_relative_worldspace.norm();
    if (target_distance != 0) record_target_distance = fmin(record_target_distance, target_distance);
    //std::cout << "record target distance: " << record_target_distance << std::endl;
    vector::localspace guidance_pid_inputs = vector::localspace(0, 0, 0);
    vector::localspace guidance_pid_inputs_direct = get_guidance_direct(current_detection_relative_worldspace, parent, 1.0);

    double apn_gain = 5.0;
    apn_gain *= std::clamp((time_since_launch-0.1)*0.5, 0.0, 1.0);
    apn_gain *= 1-get_g_limit_fraction(parent->physics_state.recorded_acceleration.norm(), 40*constants::STANDARD_GRAVITY, 60*constants::STANDARD_GRAVITY);
    guidance_pid_inputs = get_guidance_proportional_navigation(current_detection_relative_worldspace, parent, apn_gain);
    //guidance_pid_inputs = get_guidance_first_degree_prediction(current_detection_relative_worldspace, parent);
    
    ///*
    if (get_time_to_impact_first_degree_prediction(current_detection_relative_worldspace, parent) < 0.1) {
        guidance_pid_inputs = guidance_pid_inputs_direct;
    }
    //*/
    
    
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

double sensor_ir::get_g_limit_fraction(double current_acceleration, double g_limit_min, double g_limit_max) {
    double g_limit_fraction = (current_acceleration-g_limit_min) / (g_limit_max-g_limit_min);
    g_limit_fraction = std::clamp(g_limit_fraction, 0.0, 1.0);
    return g_limit_fraction;
}

vector::worldspace sensor_ir::limit_g_forces(vector::worldspace unlimited_inputs, vector::localspace limited_inputs, double current_acceleration, double g_limit_min, double g_limit_max) {
    double g_limit_fraction = get_g_limit_fraction(current_acceleration, g_limit_min, g_limit_max);
    return (1 - g_limit_fraction) * unlimited_inputs + g_limit_fraction * limited_inputs;
}

vector::worldspace sensor_ir::get_enemy_velocity(vector::worldspace current_detection_worldspace, vector::worldspace last_detection_worldspace) {
    return (1/constants::DELTA_T) * (current_detection_worldspace - last_detection_worldspace);
}

double sensor_ir::get_time_to_impact_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent) {
    vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
    vector::worldspace missile_velocity = parent->physics_state.velocity;
    vector::worldspace enemy_velocity = get_enemy_velocity(current_detection_worldspace, last_detection_worldspace);
    vector::worldspace relative_velocity = enemy_velocity - missile_velocity;

    double time_to_impact = current_detection_relative_worldspace.norm() / relative_velocity.norm();
    return time_to_impact;
}

double sensor_ir::find_smallest_positive_root(double a, double b, double c) {
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

vector::localspace sensor_ir::get_guidance_direct(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain) {
    vector::scopespace current_detection = signal_point(
        current_detection_relative_worldspace,
        vector::worldspace(0, 0, 0),
        rotation * parent->physics_state.rotation,
        1.0 // unimportant
    ).position_scopespace;
    if (std::isnan(current_detection.distance()) || std::isnan(current_detection.scope_x()) || std::isnan(current_detection.scope_y())) {
        current_detection = vector::scopespace();
    }
    current_detection.scope_x() *= gain;
    current_detection.scope_y() *= gain;
    vector::localspace output(
        -parent->physics_state.angular_velocity.to_localspace(parent->physics_state.rotation).x(),
        -current_detection.scope_y(),
        current_detection.scope_x()
    );
    //std::cout << "get_guidance_direct output: " << output.str() << "\n";
    return output;
}

vector::localspace sensor_ir::get_guidance_first_degree_prediction(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent) {
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

    return get_guidance_direct(aimpoint, parent, 1.0);
}

vector::localspace sensor_ir::get_guidance_proportional_navigation(vector::worldspace current_detection_relative_worldspace, physics_object::object* parent, double gain) {
    double LOOK_AHEAD_TIME = 1; // for turning an acceleration request into an aimpoint request
    vector::worldspace current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
    //std::cout << "current_detection_worldspace: " << current_detection_worldspace.str() << "\n";
    vector::worldspace missile_velocity = parent->physics_state.velocity;
    vector::worldspace enemy_velocity = get_enemy_velocity(current_detection_worldspace, last_detection_worldspace);
    //std::cout << "enemy_velocity: " << enemy_velocity.str() << "\n";
    vector::worldspace relative_velocity = enemy_velocity - missile_velocity;
    vector::worldspace relative_position = current_detection_relative_worldspace;
    vector::worldspace line_of_sight_rotation_vector = (relative_position.cross(relative_velocity)) / (relative_position.dot(relative_position));
    vector::worldspace desired_acceleration = -gain * ((relative_velocity.norm()) * (missile_velocity / missile_velocity.norm())).cross(line_of_sight_rotation_vector);
    vector::worldspace aimpoint = LOOK_AHEAD_TIME * missile_velocity + 0.5 * LOOK_AHEAD_TIME * LOOK_AHEAD_TIME * desired_acceleration;
    //std::cout << "aimpoint: " << aimpoint.str() << "\n";
    vector::scopespace current_detection = signal_point(
        aimpoint,
        vector::worldspace(0, 0, 0),
        rotation * parent->physics_state.rotation,
        1.0 // unimportant
    ).position_scopespace;
    //std::cout << "current_detection: " << current_detection.str() << "\n";
    return get_guidance_direct(aimpoint, parent, gain);
}

std::vector<signal_point> sensor_ir::get_signals_from_physics_objects(physics_object::object* parent) {
    std::vector<signal_point> output;
    double max_scopespace_offset = tan(view_cone_halfarc * std::numbers::pi / 180);

    globals::functional_physics_objects_mutex.lock();
    auto functional_physics_objects_ = globals::functional_physics_objects;
    globals::functional_physics_objects_mutex.unlock();
    
    for (auto o : functional_physics_objects_) {
        if (o->mutex) std::lock_guard<std::mutex> lock(*o->mutex);
        if (!o->properties.functional) continue;
        signal_point s = signal_point(
            o->physics_state.position,
            get_worldspace_position(parent),
            rotation * parent->physics_state.rotation,
            o->physics_state.base_signal_strength
        );
        if (s.position_scopespace.distance() <= 0) continue;
        double scopespace_offset_squared = s.position_scopespace.scope_x() * s.position_scopespace.scope_x() + s.position_scopespace.scope_y() * s.position_scopespace.scope_y();
        if (scopespace_offset_squared > max_scopespace_offset * max_scopespace_offset) continue;
        if (!ground::line_of_sight(get_worldspace_position(parent), o->physics_state.position)) continue;
        output.push_back(s);
    }
    return output;
}

vector::scopespace sensor_ir::get_target_direction(physics_object::object* parent) {
    std::vector<signal_point> signals_unfiltered = get_signals_from_physics_objects(parent);
    /*
    std::cout << "signals_unfiltered [size " << signals_unfiltered.size() << "]:\n";
    for (signal_point& s : signals_unfiltered) {
        std::cout << s.str() << "\n";
    }
    */
    const int GRID_WIDTH = 30;
    const double LAST_TARGET_BONUS = 0.4;
    sensor_cell_grid grid = sensor_cell_grid(GRID_WIDTH, view_cone_halfarc);
    double target_recognition_radius = tan(target_recognition_cone_halfarc * std::numbers::pi / 180);
    for (signal_point& p : signals_unfiltered) {
        grid.increase_signals_in_circle(p.position_scopespace.scope_x(), p.position_scopespace.scope_y(), target_recognition_radius, p.position_scopespace.distance(), p.signal_strength);
    }
    if (
        !std::isnan(last_detection_scopespace.distance()) &&
        !std::isnan(last_detection_scopespace.scope_x()) &&
        !std::isnan(last_detection_scopespace.scope_y()) &&
        !std::isnan(last_detection_signal_strength)
    ) {
        grid.increase_signals_in_circle(last_detection_scopespace.scope_x(), last_detection_scopespace.scope_y(), target_recognition_radius, last_detection_scopespace.distance(), last_detection_signal_strength);
    }
    signal_point center = grid.get_largest_signal();

    /*
    since the get_largest_signal function will only select the
    first index of the strongest signal's circle, that value
    can be ever so slightly out of the bounds of the
    target_recognition_radius due to the elements being on a
    coarse grid where values are rounded all over the place;
    increasing the target_recognition_radius by 1 grid cell
    fixes that
    */
    target_recognition_radius += grid.distance_between_grid_cells();

    //std::cout << "center (get_largest_signal): " << center.str() << "\n";
    //std::cout << "target_recognition_radius: " << target_recognition_radius << "\n";
    std::vector<int> signal_indices_in_target_recognition_circle;
    for (int i = 0; i < signals_unfiltered.size(); i++) {
        //std::cout << "  signals_unfiltered[" << i << "].position_scopespace.scope_x(): " << signals_unfiltered[i].position_scopespace.scope_x() << "\n";
        //std::cout << "  signals_unfiltered[" << i << "].position_scopespace.scope_y(): " << signals_unfiltered[i].position_scopespace.scope_y() << "\n";
        //std::cout << "  center.position_scopespace.scope_x(): " << center.position_scopespace.scope_x() << "\n";
        //std::cout << "  center.position_scopespace.scope_y(): " << center.position_scopespace.scope_y() << "\n";
        double distance_x = signals_unfiltered[i].position_scopespace.scope_x() - center.position_scopespace.scope_x();
        double distance_y = signals_unfiltered[i].position_scopespace.scope_y() - center.position_scopespace.scope_y();
        //std::cout << "  distance_x: " << distance_x << "\n";
        //std::cout << "  distance_y: " << distance_y << "\n";
        if (distance_x * distance_x + distance_y * distance_y < target_recognition_radius * target_recognition_radius) {
            //std::cout << "  pushed";
            signal_indices_in_target_recognition_circle.push_back(i);
        }
        //std::cout << "\n\n";
    }
    /*
    std::cout << "signal_indices_in_target_recognition_circle: {";
    for (int i : signal_indices_in_target_recognition_circle) {
        std::cout << i << ", ";
    }
    std::cout << "\b\b}\n";
    */
    
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
    //std::cout << "get_target_direction result: " << result.str() << "\n";
    //if (globals::tick % 20 == 0) grid.print(result, center.position_scopespace);
    return result;
}

vector::worldspace sensor_ir::get_target_position(physics_object::object* parent) {
    vector::scopespace result = get_target_direction(parent);
    vector::localspace result_localspace = {
        result.distance(),
        result.scope_x() * result.distance(),
        result.scope_y() * result.distance()
    };
    return result_localspace.to_worldspace(parent->physics_state.rotation);
}

}

void physics_object::object::add_sensor_ir(module::sensor_ir s) {
	std::shared_ptr<module::module> m = std::make_shared<module::sensor_ir>(s);
	properties.modules.push_back(m);
}